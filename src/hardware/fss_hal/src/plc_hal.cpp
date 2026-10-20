#include "fss_hal/plc_hal.hpp"

#include <modbus.h>

#include <cerrno>
#include <chrono>
#include <exception>
#include <limits>

namespace fss_hal {
namespace {
void validate(const PlcConfig& c) {
  if (c.ip.empty() || c.port < 1 || c.port > 65535 || !std::isfinite(c.timeout_sec) ||
      c.timeout_sec <= 0.0 || c.timeout_sec > 3600.0 || c.valve_count < 1 ||
      c.valve_count > MODBUS_MAX_WRITE_REGISTERS || c.valve_register_start < 0 ||
      c.valve_register_start + c.valve_count > 65536 || c.main_valve_register < 0 ||
      c.main_valve_register > 65535 || c.max_value < 1 || c.max_value > 65535 ||
      (c.main_valve_register >= c.valve_register_start &&
       c.main_valve_register < c.valve_register_start + c.valve_count))
    throw std::invalid_argument("Invalid PLC register, connection or output configuration");
}

class LibmodbusTransport final : public ModbusTransport {
 public:
  explicit LibmodbusTransport(const PlcConfig& config) {
    validate(config);
    context_ = modbus_new_tcp(config.ip.c_str(), config.port);
    if (!context_) throw std::runtime_error("Cannot allocate Modbus TCP context");
    const auto micros = static_cast<std::uint64_t>(config.timeout_sec * 1000000.0);
    if (modbus_set_response_timeout(context_, static_cast<std::uint32_t>(micros / 1000000),
                                    static_cast<std::uint32_t>(micros % 1000000)) == -1 ||
        modbus_set_byte_timeout(context_, 0, 0) == -1 || modbus_set_slave(context_, 1) == -1) {
      const std::string error = modbus_strerror(errno);
      modbus_free(context_);
      context_ = nullptr;
      throw std::runtime_error(error);
    }
    // Keep libmodbus's automatic link recovery disabled: reconnect must never replay a command.
  }
  ~LibmodbusTransport() override { close(); modbus_free(context_); }
  void connect() override {
    if (modbus_connect(context_) == -1) fail("Modbus TCP connect");
    opened_ = true;
  }
  void close() noexcept override {
    if (opened_) modbus_close(context_);
    opened_ = false;
  }
  void write_registers(int address, const std::vector<std::uint16_t>& values) override {
    if (modbus_write_registers(context_, address, static_cast<int>(values.size()), values.data()) !=
        static_cast<int>(values.size())) fail("Modbus FC16 write");
  }
  void write_register(int address, std::uint16_t value) override {
    if (modbus_write_register(context_, address, value) != 1) fail("Modbus FC06 write");
  }
  std::vector<bool> read_discrete_inputs(int address, int count) override {
    std::vector<std::uint8_t> values(static_cast<std::size_t>(count));
    if (modbus_read_input_bits(context_, address, count, values.data()) != count)
      fail("Modbus FC02 read");
    return std::vector<bool>(values.begin(), values.end());
  }
 private:
  [[noreturn]] static void fail(const char* operation) {
    throw std::runtime_error(std::string(operation) + ": " + modbus_strerror(errno));
  }
  modbus_t* context_{nullptr};
  bool opened_{false};
};
}  // namespace

std::unique_ptr<ModbusTransport> make_modbus_transport(const PlcConfig& config) {
  return std::make_unique<LibmodbusTransport>(config);
}

PlcHal::PlcHal(PlcConfig config, std::unique_ptr<ModbusTransport> transport)
    : config_(std::move(config)), transport_(std::move(transport)) {
  validate(config_);
  if (!transport_) transport_ = make_modbus_transport(config_);
}
PlcHal::~PlcHal() { try { disconnect(); } catch (...) {} }

void PlcHal::connect() {
  if (connected_) return;
  transport_->connect();
  connected_ = true;
  try {
    emergency_stop();
    fault_latched_ = false;
  } catch (...) {
    transport_->close();
    connected_ = false;
    throw;
  }
}

void PlcHal::disconnect() {
  if (!connected_) return;
  std::exception_ptr failure;
  try { emergency_stop(); } catch (...) { failure = std::current_exception(); }
  transport_->close();
  connected_ = false;
  if (failure) std::rethrow_exception(failure);
}

void PlcHal::require_connection() const {
  if (!connected_) throw std::runtime_error("PLC is disconnected");
}

void PlcHal::set_valves(const std::vector<double>& values) {
  require_connection();
  if (values.size() != static_cast<std::size_t>(config_.valve_count))
    throw std::invalid_argument("Valve count mismatch");
  std::vector<std::uint16_t> registers;
  registers.reserve(values.size());
  for (double value : values) registers.push_back(valve_register_value(value, config_.max_value));
  if (fault_latched_ && std::any_of(registers.begin(), registers.end(), [](auto v) { return v != 0; }))
    throw std::runtime_error("PLC fault is latched; restart and confirm zero before rearming");
  const auto start = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    transport_->write_registers(config_.valve_register_start, registers);
  } catch (...) {
    fault_latched_ = true;
    throw;
  }
  last_rtt_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

void PlcHal::set_main_valve(bool open) {
  require_connection();
  if (open && fault_latched_) throw std::runtime_error("PLC fault is latched");
  std::lock_guard<std::mutex> lock(mutex_);
  try { transport_->write_register(config_.main_valve_register, open ? 1 : 0); }
  catch (...) { fault_latched_ = true; throw; }
}

void PlcHal::emergency_stop() {
  require_connection();
  std::lock_guard<std::mutex> lock(mutex_);
  std::exception_ptr valve_failure, main_failure;
  try {
    transport_->write_registers(config_.valve_register_start,
                               std::vector<std::uint16_t>(config_.valve_count, 0));
  } catch (...) { valve_failure = std::current_exception(); }
  // A failed FC16 must never skip the independent main-valve close attempt.
  try { transport_->write_register(config_.main_valve_register, 0); }
  catch (...) { main_failure = std::current_exception(); }
  if (valve_failure || main_failure) fault_latched_ = true;
  if (valve_failure) std::rethrow_exception(valve_failure);
  if (main_failure) std::rethrow_exception(main_failure);
}

std::optional<std::vector<bool>> PlcHal::read_digital_inputs() {
  if (!connected_) return std::nullopt;
  std::lock_guard<std::mutex> lock(mutex_);
  try { return transport_->read_discrete_inputs(0, 8); }
  catch (...) { return std::nullopt; }
}
}  // namespace fss_hal
