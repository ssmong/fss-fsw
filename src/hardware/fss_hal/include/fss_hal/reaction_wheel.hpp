#pragma once

#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace fss_hal {
double monotonic_seconds();
using WheelClock = std::function<double()>;

struct ReactionWheelConfig {
  bool enabled{false};
  bool allow_gnc_commands{false};
  std::string integration_state{"bench_only"};
  std::string firmware_variant{"torque_current_ctrl_simple"};
  std::string interface{"usb_cdc"};
  std::string torque_semantics{"wheel"};
  std::array<int, 3> state_codes{{0, 1, 2}};
  std::string port{"/dev/ttyACM0"};
  int baudrate{921600};
  double command_rate_hz{20.0};
  double telemetry_rate_hz{100.0};
  double rx_timeout_sec{0.5};
  double partial_line_timeout_sec{0.05};
  double write_timeout_sec{0.05};
  double max_torque_nm{0.013488};
  double torque_constant_nm_per_a{0.01405};
  double stop_epsilon_nm{1e-6};
  double command_timeout_sec{0.25};
  double telemetry_timeout_sec{0.1};
  static ReactionWheelConfig from_yaml(const std::string& filename);
  void validate() const;
};

struct WheelStatus {
  double current_a;
  double duty;
  double generated_torque_nm;
  int state;
  double received_at;
  std::string fault_reason;
};

class ReactionWheelHal {
 public:
  virtual ~ReactionWheelHal() = default;
  virtual void connect() = 0;
  virtual void disconnect() = 0;
  virtual void set_torque(double torque_nm) = 0;
  virtual void stop() = 0;
  virtual std::optional<WheelStatus> read_status() = 0;
  virtual bool is_connected() const = 0;
  virtual bool fault_latched() const = 0;
  virtual bool stop_confirmed() const = 0;
  virtual const std::string& fault_reason() const = 0;
};

class WheelSerial {
 public:
  virtual ~WheelSerial() = default;
  virtual void open() = 0;
  virtual void close() noexcept = 0;
  virtual bool is_open() const = 0;
  virtual std::size_t available() = 0;
  virtual std::string read(std::size_t count) = 0;
  virtual void write(const std::string& data) = 0;
  virtual void reset_input() = 0;
};
std::unique_ptr<WheelSerial> make_wheel_serial(const ReactionWheelConfig& config);

class SerialReactionWheelHal final : public ReactionWheelHal {
 public:
  explicit SerialReactionWheelHal(ReactionWheelConfig config,
      std::unique_ptr<WheelSerial> serial = nullptr, WheelClock clock = monotonic_seconds);
  ~SerialReactionWheelHal() override;
  void connect() override;
  void disconnect() override;
  void set_torque(double torque_nm) override;
  void stop() override;
  std::optional<WheelStatus> read_status() override;
  bool is_connected() const override { return serial_->is_open(); }
  bool fault_latched() const override { return !fault_.empty(); }
  bool stop_confirmed() const override { return stop_confirmed_; }
  const std::string& fault_reason() const override { return fault_; }
  const std::string& last_ack() const { return last_ack_; }
 private:
  void write(const std::string& data);
  void trip(const std::string& reason) noexcept;
  std::optional<WheelStatus> parse(const std::string& line, double now);
  ReactionWheelConfig config_;
  std::unique_ptr<WheelSerial> serial_;
  WheelClock clock_;
  std::string buffer_;
  std::optional<double> partial_since_;
  std::string fault_;
  std::string last_ack_;
  bool stop_confirmed_{false};
  bool ready_for_torque_{false};
  bool rearm_pending_{false};
};

class SimReactionWheelHal final : public ReactionWheelHal {
 public:
  explicit SimReactionWheelHal(ReactionWheelConfig config, WheelClock clock = monotonic_seconds);
  void connect() override;
  void disconnect() override;
  void set_torque(double torque_nm) override;
  void stop() override;
  std::optional<WheelStatus> read_status() override;
  bool is_connected() const override { return connected_; }
  bool fault_latched() const override { return !fault_.empty(); }
  bool stop_confirmed() const override { return stop_confirmed_; }
  const std::string& fault_reason() const override { return fault_; }
 private:
  ReactionWheelConfig config_;
  WheelClock clock_;
  bool connected_{false};
  double torque_{0.0};
  std::string fault_;
  bool stop_confirmed_{false};
  bool ready_for_torque_{false};
  bool rearm_pending_{false};
};
}  // namespace fss_hal
