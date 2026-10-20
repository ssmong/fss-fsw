#pragma once

#include "fss_hal/valve_hal.hpp"

#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace fss_hal {
struct PlcConfig {
  std::string ip{"192.168.1.2"};
  int port{502};
  double timeout_sec{1.0};
  int valve_register_start{100};
  int valve_count{8};
  int main_valve_register{99};
  int max_value{4000};
};

// The transport seam permits protocol failures to be tested without contacting a PLC.
class ModbusTransport {
 public:
  virtual ~ModbusTransport() = default;
  virtual void connect() = 0;
  virtual void close() noexcept = 0;
  virtual void write_registers(int address, const std::vector<std::uint16_t>& values) = 0;
  virtual void write_register(int address, std::uint16_t value) = 0;
  virtual std::vector<bool> read_discrete_inputs(int address, int count) = 0;
};

std::unique_ptr<ModbusTransport> make_modbus_transport(const PlcConfig& config);

class PlcHal final : public ValveHal {
 public:
  explicit PlcHal(PlcConfig config, std::unique_ptr<ModbusTransport> transport = nullptr);
  ~PlcHal() override;
  void connect() override;
  void disconnect() override;
  void set_valves(const std::vector<double>& values) override;
  void set_main_valve(bool open) override;
  void emergency_stop() override;
  bool is_connected() const override { return connected_; }
  double last_rtt_ms() const override { return last_rtt_ms_; }
  bool fault_latched() const { return fault_latched_; }
  std::optional<std::vector<bool>> read_digital_inputs();
 private:
  void require_connection() const;
  PlcConfig config_;
  std::unique_ptr<ModbusTransport> transport_;
  std::mutex mutex_;
  bool connected_{false};
  bool fault_latched_{false};
  double last_rtt_ms_{0.0};
};
}  // namespace fss_hal
