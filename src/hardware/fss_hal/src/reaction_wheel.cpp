#include "fss_hal/reaction_wheel.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace fss_hal {
double monotonic_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void ReactionWheelConfig::validate() const {
  if (firmware_variant != "torque_current_ctrl_simple" || interface != "usb_cdc" ||
      torque_semantics != "wheel" || state_codes != std::array<int, 3>{{0, 1, 2}})
    throw std::invalid_argument("Unsupported reaction wheel firmware or protocol contract");
  for (const auto value : {command_rate_hz, telemetry_rate_hz, rx_timeout_sec,
       partial_line_timeout_sec, write_timeout_sec, max_torque_nm, torque_constant_nm_per_a,
       stop_epsilon_nm, command_timeout_sec, telemetry_timeout_sec}) {
    if (!std::isfinite(value) || value <= 0.0)
      throw std::invalid_argument("Reaction wheel rates, limits and timeouts must be finite and positive");
  }
  if (baudrate <= 0 || port.empty() || stop_epsilon_nm >= max_torque_nm ||
      !(1.0 / command_rate_hz < command_timeout_sec && command_timeout_sec < rx_timeout_sec))
    throw std::invalid_argument("Invalid reaction wheel serial, torque or watchdog configuration");
}

ReactionWheelConfig ReactionWheelConfig::from_yaml(const std::string& filename) {
  const auto config = YAML::LoadFile(filename)["reaction_wheel"];
  if (!config.IsMap()) throw std::invalid_argument("reaction_wheel YAML mapping is required");
  ReactionWheelConfig c;
  c.enabled = config["enabled"].as<bool>();
  c.allow_gnc_commands = config["allow_gnc_commands"].as<bool>();
  c.integration_state = config["integration_state"].as<std::string>();
  c.firmware_variant = config["firmware"]["variant"].as<std::string>();
  const auto connection = config["connection"];
  c.interface = connection["interface"].as<std::string>();
  c.port = connection["port"].as<std::string>();
  c.baudrate = connection["baudrate"].as<int>();
  c.command_rate_hz = connection["command_rate_hz"].as<double>();
  c.telemetry_rate_hz = connection["telemetry_rate_hz"].as<double>();
  c.rx_timeout_sec = connection["rx_timeout_ms"].as<double>() / 1000.0;
  c.partial_line_timeout_sec = connection["partial_line_timeout_ms"].as<double>() / 1000.0;
  c.write_timeout_sec = connection["write_timeout_ms"].as<double>(50.0) / 1000.0;
  const auto protocol = config["protocol"];
  c.torque_semantics = protocol["command_torque_semantics"].as<std::string>();
  const auto codes = protocol["state_codes"];
  if (!codes.IsMap() || codes.size() != 3) throw std::invalid_argument("Invalid wheel state codes");
  c.state_codes = {{codes["stop"].as<int>(), codes["active"].as<int>(), codes["fault"].as<int>()}};
  const auto electrical = config["electrical"];
  c.max_torque_nm = electrical["max_wheel_torque_nm"].as<double>();
  c.torque_constant_nm_per_a = electrical["torque_constant_nm_per_a"].as<double>();
  c.stop_epsilon_nm = electrical["torque_stop_epsilon_nm"].as<double>();
  c.command_timeout_sec = config["driver"]["command_timeout_sec"].as<double>();
  c.telemetry_timeout_sec = config["driver"]["telemetry_timeout_sec"].as<double>();
  c.validate();
  return c;
}

namespace {
[[noreturn]] void io_error(const std::string& operation) {
  throw std::runtime_error(operation + ": " + std::strerror(errno));
}

speed_t serial_speed(int baudrate) {
  switch (baudrate) {
    case 50: return B50;
    case 75: return B75;
    case 110: return B110;
    case 134: return B134;
    case 150: return B150;
    case 200: return B200;
    case 300: return B300;
    case 600: return B600;
    case 1200: return B1200;
    case 1800: return B1800;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 2500000: return B2500000;
    case 3000000: return B3000000;
    case 3500000: return B3500000;
    case 4000000: return B4000000;
    default: throw std::invalid_argument("Unsupported POSIX serial baudrate");
  }
}

class PosixWheelSerial final : public WheelSerial {
 public:
  explicit PosixWheelSerial(ReactionWheelConfig config) : config_(std::move(config)) {
    serial_speed(config_.baudrate);
  }
  ~PosixWheelSerial() override { close(); }
  void open() override {
    if (is_open()) return;
    fd_ = ::open(config_.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) io_error("Open reaction wheel serial");
    try {
      termios settings{};
      if (tcgetattr(fd_, &settings) < 0) io_error("Read reaction wheel serial settings");
      cfmakeraw(&settings);
      settings.c_cflag |= CLOCAL | CREAD;
      settings.c_cflag &= ~(CSTOPB | CRTSCTS);
      settings.c_cc[VMIN] = 0;
      settings.c_cc[VTIME] = 0;
      if (cfsetispeed(&settings, serial_speed(config_.baudrate)) < 0 ||
          cfsetospeed(&settings, serial_speed(config_.baudrate)) < 0 ||
          tcsetattr(fd_, TCSANOW, &settings) < 0) io_error("Configure reaction wheel serial");
    } catch (...) { close(); throw; }
  }
  void close() noexcept override { if (fd_ >= 0) ::close(fd_); fd_ = -1; }
  bool is_open() const override { return fd_ >= 0; }
  std::size_t available() override {
    int count = 0;
    if (ioctl(fd_, FIONREAD, &count) < 0) io_error("Read reaction wheel RX count");
    pollfd descriptor{fd_, POLLIN, 0};
    if (::poll(&descriptor, 1, 0) < 0 && errno != EINTR) io_error("Poll reaction wheel serial");
    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))
      throw std::runtime_error("Reaction wheel serial disconnected");
    return static_cast<std::size_t>(count);
  }
  std::string read(std::size_t count) override {
    std::string data(count, '\0');
    const auto received = ::read(fd_, data.data(), data.size());
    if (received < 0) {
      if (errno == EAGAIN || errno == EINTR) return {};
      io_error("Read reaction wheel serial");
    }
    data.resize(static_cast<std::size_t>(received));
    return data;
  }
  void write(const std::string& data) override {
    const double deadline = monotonic_seconds() + config_.write_timeout_sec;
    std::size_t sent = 0;
    while (sent < data.size()) {
      const auto count = ::write(fd_, data.data() + sent, data.size() - sent);
      if (count > 0) sent += static_cast<std::size_t>(count);
      else if (count < 0 && errno != EAGAIN && errno != EINTR) io_error("Write reaction wheel serial");
      if (sent == data.size()) return;
      const double remaining = deadline - monotonic_seconds();
      if (remaining <= 0.0) throw std::runtime_error("Reaction wheel serial write timed out");
      pollfd descriptor{fd_, POLLOUT, 0};
      const int milliseconds = static_cast<int>(std::min(remaining * 1000.0 + 1.0,
                                                       static_cast<double>(std::numeric_limits<int>::max())));
      const int result = ::poll(&descriptor, 1, milliseconds);
      if (result < 0 && errno != EINTR) io_error("Poll reaction wheel serial write");
      if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))
        throw std::runtime_error("Reaction wheel serial disconnected during write");
    }
  }
  void reset_input() override { if (tcflush(fd_, TCIFLUSH) < 0) io_error("Flush reaction wheel RX"); }
 private:
  ReactionWheelConfig config_;
  int fd_{-1};
};

std::vector<std::string> fields(const std::string& text) {
  std::vector<std::string> result;
  std::size_t start = 0;
  for (;;) {
    const auto comma = text.find(',', start);
    result.push_back(text.substr(start, comma == std::string::npos ? comma : comma - start));
    if (comma == std::string::npos) return result;
    start = comma + 1;
  }
}

double finite_number(const std::string& text) {
  std::istringstream stream(text);
  stream.imbue(std::locale::classic());
  double value;
  if (!(stream >> value) || !std::isfinite(value)) throw std::invalid_argument("Invalid wheel numeric field");
  stream >> std::ws;
  if (!stream.eof()) throw std::invalid_argument("Trailing wheel numeric data");
  return value;
}
}  // namespace

std::unique_ptr<WheelSerial> make_wheel_serial(const ReactionWheelConfig& config) {
  return std::make_unique<PosixWheelSerial>(config);
}

SerialReactionWheelHal::SerialReactionWheelHal(ReactionWheelConfig config,
    std::unique_ptr<WheelSerial> serial, WheelClock clock)
    : config_(std::move(config)), serial_(std::move(serial)), clock_(std::move(clock)) {
  config_.validate();
  if (!serial_) serial_ = make_wheel_serial(config_);
}
SerialReactionWheelHal::~SerialReactionWheelHal() { try { disconnect(); } catch (...) {} }

void SerialReactionWheelHal::connect() {
  if (is_connected()) return;
  try { serial_->open(); stop(); }
  catch (...) { try { disconnect(); } catch (...) {} throw; }
}
void SerialReactionWheelHal::write(const std::string& data) {
  if (!is_connected()) throw std::runtime_error("Reaction wheel is disconnected");
  serial_->write(data);
}
void SerialReactionWheelHal::trip(const std::string& reason) noexcept {
  if (fault_.empty()) fault_ = reason;
  stop_confirmed_ = rearm_pending_ = ready_for_torque_ = false;
  // Z interrupts a partially sent torque command without waiting for a newline.
  try { write("Z"); } catch (...) {}
}
void SerialReactionWheelHal::stop() {
  stop_confirmed_ = rearm_pending_ = ready_for_torque_ = false;
  buffer_.clear();
  partial_since_.reset();
  try {
    if (!is_connected()) throw std::runtime_error("Reaction wheel is disconnected");
    serial_->reset_input();
    write("Z");
  } catch (const std::exception& e) { trip(e.what()); throw; }
  rearm_pending_ = true;
}
void SerialReactionWheelHal::set_torque(double torque) {
  if (!std::isfinite(torque) || std::abs(torque) > config_.max_torque_nm) {
    trip("Invalid wheel torque");
    throw std::invalid_argument("Invalid wheel torque");
  }
  if (std::abs(torque) < config_.stop_epsilon_nm) { stop(); return; }
  if (fault_latched() || !ready_for_torque_)
    throw std::runtime_error("STOP telemetry must confirm recovery before torque commands");
  std::ostringstream command;
  command.imbue(std::locale::classic());
  command << "T," << std::setprecision(12) << torque << '\n';
  try { write(command.str()); stop_confirmed_ = false; }
  catch (const std::exception& e) { trip(e.what()); throw; }
}

std::optional<WheelStatus> SerialReactionWheelHal::parse(const std::string& line, double now) {
  if (std::any_of(line.begin(), line.end(), [](unsigned char c) { return c > 127; }))
    throw std::invalid_argument("Non-ASCII reaction wheel frame");
  const auto begin = line.find_first_not_of(" \t\r\v\f");
  const auto end = line.find_last_not_of(" \t\r\v\f");
  const std::string text = begin == std::string::npos ? "" : line.substr(begin, end - begin + 1);
  const auto parts = fields(text);
  if (parts.size() >= 2 && parts[0] == "FAULT" && !parts[1].empty()) {
    trip(text.substr(6));
    return std::nullopt;
  }
  if (parts.size() >= 3 && parts[0] == "ACK" && parts[1] == "STOP" && !parts[2].empty()) {
    last_ack_ = text;
    return std::nullopt;
  }
  if (parts.size() == 3 && parts[0] == "ACK" && parts[1] == "T") {
    if (std::abs(finite_number(parts[2])) > config_.max_torque_nm)
      throw std::invalid_argument("Invalid accepted wheel torque");
    last_ack_ = text;
    return std::nullopt;
  }
  if (parts.size() != 5 || parts[0] != "S")
    throw std::invalid_argument("Unknown reaction wheel frame");
  const double current = finite_number(parts[1]);
  const double duty = finite_number(parts[2]);
  const double torque = finite_number(parts[3]);
  std::size_t consumed = 0;
  const int state = std::stoi(parts[4], &consumed);
  if (parts[4].find_first_not_of(" \t\r\v\f", consumed) != std::string::npos || state < 0 || state > 2)
    throw std::invalid_argument("Invalid reaction wheel state");
  if (state == 2) trip("MCU FAULT state");
  else if (state == 0 && rearm_pending_) {
    fault_.clear();
    rearm_pending_ = false;
    stop_confirmed_ = ready_for_torque_ = true;
  } else if (state == 1) stop_confirmed_ = false;
  return WheelStatus{current, duty, torque, state, now, fault_};
}

std::optional<WheelStatus> SerialReactionWheelHal::read_status() {
  if (!is_connected()) throw std::runtime_error("Reaction wheel is disconnected");
  const double now = clock_();
  try {
    if (partial_since_ && now - *partial_since_ >= config_.partial_line_timeout_sec)
      throw std::runtime_error("Partial reaction wheel frame timed out");
    const auto available = serial_->available();
    if (available > 4096) throw std::runtime_error("Reaction wheel RX backlog exceeded poll limit");
    if (available) buffer_ += serial_->read(available);
    std::optional<WheelStatus> latest;
    for (;;) {
      const auto newline = buffer_.find('\n');
      if (newline == std::string::npos) break;
      if (newline > 256) throw std::runtime_error("Reaction wheel line too long");
      const auto line = buffer_.substr(0, newline);
      buffer_.erase(0, newline + 1);
      const auto status = parse(line, now);
      if (status) latest = status;
      partial_since_.reset();
    }
    if (buffer_.size() > 256) throw std::runtime_error("Reaction wheel line too long");
    if (!buffer_.empty() && !partial_since_) partial_since_ = now;
    return latest;
  } catch (const std::exception& e) {
    buffer_.clear(); partial_since_.reset(); trip(e.what()); throw;
  }
}

void SerialReactionWheelHal::disconnect() {
  stop_confirmed_ = rearm_pending_ = ready_for_torque_ = false;
  buffer_.clear(); partial_since_.reset();
  std::exception_ptr failure;
  if (is_connected()) try { write("Z"); } catch (...) { failure = std::current_exception(); }
  serial_->close();
  if (failure) std::rethrow_exception(failure);
}

SimReactionWheelHal::SimReactionWheelHal(ReactionWheelConfig config, WheelClock clock)
    : config_(std::move(config)), clock_(std::move(clock)) { config_.validate(); }
void SimReactionWheelHal::connect() { if (!connected_) { connected_ = true; stop(); } }
void SimReactionWheelHal::disconnect() {
  torque_ = 0.0;
  connected_ = stop_confirmed_ = rearm_pending_ = ready_for_torque_ = false;
}
void SimReactionWheelHal::stop() {
  if (!connected_) throw std::runtime_error("Reaction wheel is disconnected");
  torque_ = 0.0;
  stop_confirmed_ = ready_for_torque_ = false;
  rearm_pending_ = true;
}
void SimReactionWheelHal::set_torque(double torque) {
  if (!std::isfinite(torque) || std::abs(torque) > config_.max_torque_nm) {
    torque_ = 0.0;
    fault_ = "Invalid wheel torque";
    stop_confirmed_ = rearm_pending_ = ready_for_torque_ = false;
    throw std::invalid_argument(fault_);
  }
  if (std::abs(torque) < config_.stop_epsilon_nm) { stop(); return; }
  if (!connected_) throw std::runtime_error("Reaction wheel is disconnected");
  if (fault_latched() || !ready_for_torque_)
    throw std::runtime_error("STOP telemetry must confirm recovery before torque commands");
  torque_ = torque;
  stop_confirmed_ = false;
}
std::optional<WheelStatus> SimReactionWheelHal::read_status() {
  if (!connected_) throw std::runtime_error("Reaction wheel is disconnected");
  if (rearm_pending_) {
    fault_.clear(); rearm_pending_ = false; stop_confirmed_ = ready_for_torque_ = true;
  }
  return WheelStatus{torque_ / config_.torque_constant_nm_per_a, 0.0, torque_,
                     fault_latched() ? 2 : (torque_ == 0.0 ? 0 : 1), clock_(), fault_};
}
}  // namespace fss_hal
