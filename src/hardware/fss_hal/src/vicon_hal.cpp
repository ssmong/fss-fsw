#include "fss_hal/vicon_hal.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <system_error>
#include <utility>

#include <nlohmann/json.hpp>

namespace fss_hal {
namespace {
double number(const nlohmann::json& value) {
  if (!value.is_number()) throw std::invalid_argument("Expected a JSON number");
  const double result = value.get<double>();
  if (!std::isfinite(result)) throw std::invalid_argument("Expected a finite JSON number");
  return result;
}

void validate_ip(const std::string& ip) {
  in_addr address{};
  if (!ip.empty() && inet_pton(AF_INET, ip.c_str(), &address) != 1) {
    throw std::invalid_argument("Vicon address must be an IPv4 address");
  }
}
}  // namespace

ViconPacketDecoder::ViconPacketDecoder(std::string object_name)
    : object_name_(std::move(object_name)) {
  if (object_name_.find_first_not_of(" \t\r\n") == std::string::npos) {
    throw std::invalid_argument("Vicon object_name must not be empty");
  }
}

void ViconPacketDecoder::consume(std::string_view packet) {
  try {
    // Bound untrusted JSON nesting before the parser can exhaust its stack.
    const auto message = nlohmann::json::parse(
        packet.begin(), packet.end(),
        [](int depth, nlohmann::json::parse_event_t, nlohmann::json&) {
          if (depth > 64) throw std::invalid_argument("Vicon JSON nesting exceeds 64");
          return true;
        });
    const auto& frame = message.at("frame");
    const double source_time = number(message.at("t"));
    if (!frame.is_number_integer() ||
        (!frame.is_number_unsigned() && frame.get<std::int64_t>() < 0) ||
        source_time < 0.0) {
      throw std::invalid_argument("Vicon frame and t must be nonnegative");
    }
    const auto frame_number = frame.get<std::uint64_t>();
    if (last_time_ && (source_time <= *last_time_ || frame_number == *last_frame_)) return;
    const auto& objects = message.at("objects");
    if (!objects.is_object()) throw std::invalid_argument("Vicon objects must be an object");
    std::optional<PlanarPoseSample> sample;
    const auto object = objects.find(object_name_);
    if (object != objects.end() && !object->is_null()) {
      if (!object->is_object() || !object->at("valid").is_boolean()) {
        throw std::invalid_argument("Vicon valid must be a JSON boolean");
      }
      if (object->at("valid").get<bool>()) {
        const double yaw = number(object->at("yaw")) * (kPi / 180.0);
        sample = PlanarPoseSample{number(object->at("x")) * 0.001,
                                  number(object->at("y")) * 0.001,
                                  std::remainder(yaw, 2.0 * kPi),
                                  frame_number, source_time};
      }
    }
    last_frame_ = frame_number;
    last_time_ = source_time;
    latest_ = sample;
  } catch (...) {
    latest_.reset();
    throw;
  }
}

void ViconPacketDecoder::reset() noexcept {
  last_frame_.reset();
  last_time_.reset();
  latest_.reset();
}

ViconHAL::ViconHAL(ViconConfig config, std::function<void(const std::string&)> warning)
    : config_(std::move(config)), decoder_(config_.object_name), warning_(std::move(warning)) {
  if (config_.port < 1 || config_.port > 65535) {
    throw std::invalid_argument("Vicon port must be an integer in 1..65535");
  }
  validate_ip(config_.bind_ip);
  validate_ip(config_.source_ip);
}

ViconHAL::~ViconHAL() { disconnect(); }

void ViconHAL::connect() {
  if (socket_ >= 0) return;
  const int receiver = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (receiver < 0) throw std::system_error(errno, std::generic_category(), "Vicon socket");
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<std::uint16_t>(config_.port));
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  if (!config_.bind_ip.empty()) inet_pton(AF_INET, config_.bind_ip.c_str(), &address.sin_addr);
  if (::bind(receiver, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) < 0 ||
      ::fcntl(receiver, F_SETFL, O_NONBLOCK) < 0) {
    const int error = errno;
    ::close(receiver);
    throw std::system_error(error, std::generic_category(), "Vicon bind/nonblocking");
  }
  socket_ = receiver;
}

void ViconHAL::disconnect() noexcept {
  if (socket_ >= 0) ::close(socket_);
  socket_ = -1;
  decoder_.reset();
}

void ViconHAL::warn(const std::string& message) {
  const auto now = std::chrono::steady_clock::now();
  if (warning_ && (!last_warning_ || now - *last_warning_ >= std::chrono::seconds(2))) {
    warning_(message);
    last_warning_ = now;
  }
}

std::optional<PlanarPoseSample> ViconHAL::read() {
  if (socket_ < 0) throw std::logic_error("Vicon HAL is not connected");
  decoder_.begin_poll();
  std::array<char, 65535> data{};
  for (int count = 0; count < 256; ++count) {
    sockaddr_in sender{};
    socklen_t sender_size = sizeof(sender);
    const auto size = ::recvfrom(socket_, data.data(), data.size(), 0,
                                 reinterpret_cast<sockaddr*>(&sender), &sender_size);
    if (size < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) return decoder_.latest();
      if (errno == EINTR) continue;
      warn("Vicon UDP receive failed: " + std::string(std::strerror(errno)));
      return std::nullopt;
    }
    if (!config_.source_ip.empty()) {
      in_addr allowed{};
      inet_pton(AF_INET, config_.source_ip.c_str(), &allowed);
      if (sender.sin_addr.s_addr != allowed.s_addr) continue;
    }
    try {
      decoder_.consume(std::string_view(data.data(), static_cast<std::size_t>(size)));
    } catch (const std::exception&) {
      warn("Malformed FSS_VICON UDP packet discarded");
    }
  }
  warn("Vicon UDP read budget reached; waiting for latest frame next poll");
  return std::nullopt;
}

}  // namespace fss_hal
