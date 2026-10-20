#pragma once

#include "fss_actuators/valve_thrust_map.hpp"
#include "fss_hal/valve_hal.hpp"

#include <array>
#include <memory>
#include <optional>

namespace fss_actuators {
class ValveControl {
 public:
  static constexpr std::size_t kCount = 8;
  static constexpr std::uint8_t kActive = 3;
  static constexpr std::uint8_t kSafeHold = 4;
  static constexpr std::uint8_t kEmergencyStop = 5;

  ValveControl(fss_hal::ValveHal& hal, int maximum, int deadband_minimum,
               double max_thrust, bool use_thrusters, std::unique_ptr<ValveThrustMap> map = nullptr)
      : hal_(hal), maximum_(maximum), max_thrust_(max_thrust), use_thrusters_(use_thrusters),
        map_(std::move(map)) {
    if (maximum < 1 || maximum > 65535 || deadband_minimum < 0 || deadband_minimum > maximum ||
        (!map_ && (!std::isfinite(max_thrust) || max_thrust <= 0.0)))
      throw std::invalid_argument("Invalid valve output scale, deadband or maximum thrust");
    threshold_ = static_cast<double>(deadband_minimum) / maximum;
  }

  void command(const std::array<double, kCount>& thrust, const std::array<bool, kCount>& enabled) {
    std::vector<double> commanded(thrust.begin(), thrust.end());
    for (auto& v : commanded) {
      if (!std::isfinite(v)) v = 0.0;
      if (!map_) v = std::clamp(v / max_thrust_, 0.0, 1.0);
    }
    if (map_) commanded = map_->duty_from_thrust(commanded);
    std::vector<double> applied(kCount, 0.0);
    for (std::size_t i = 0; i < kCount; ++i) {
      if (use_thrusters_ && enabled[i] && mode_ == kActive && commanded[i] >= threshold_)
        applied[i] = commanded[i];
    }
    hal_.set_valves(applied);
    std::copy(commanded.begin(), commanded.end(), commanded_.begin());
    for (std::size_t i = 0; i < kCount; ++i)
      registers_[i] = fss_hal::valve_register_value(applied[i], maximum_);
  }

  bool mode(std::uint8_t value) {
    mode_ = value;
    if (value == kActive) return false;
    if (value == kEmergencyStop) {
      // Drop the heartbeat intent even if closing the hardware cannot be confirmed.
      main_requested_ = false;
      hal_.emergency_stop();
      main_applied_ = false;
    } else {
      hal_.set_valves(std::vector<double>(kCount, 0.0));
    }
    commanded_.fill(0.0);
    registers_.fill(0);
    return true;
  }

  void main_valve(bool open) {
    if (open && (mode_ == kEmergencyStop || mode_ == kSafeHold))
      throw std::runtime_error("Main valve opening is forbidden in the current system mode");
    if (!open) main_requested_ = false;
    hal_.set_main_valve(open);
    main_applied_ = main_requested_ = open;
  }
  void heartbeat() { if (main_requested_ && hal_.is_connected()) hal_.set_main_valve(true); }
  const std::array<double, kCount>& commanded() const { return commanded_; }
  const std::array<std::uint16_t, kCount>& registers() const { return registers_; }
  bool main_valve_open() const { return main_applied_; }
  std::array<double, kCount> applied_duty() const {
    std::array<double, kCount> values{};
    for (std::size_t i = 0; i < kCount; ++i) values[i] = static_cast<double>(registers_[i]) / maximum_;
    return values;
  }
  std::array<double, kCount> estimated_thrust() const {
    std::array<double, kCount> result{};
    if (map_) {
      const auto duty = applied_duty();
      const auto values = map_->thrust_from_duty(std::vector<double>(duty.begin(), duty.end()));
      std::copy(values.begin(), values.end(), result.begin());
    }
    return result;
  }
 private:
  fss_hal::ValveHal& hal_;
  int maximum_;
  double threshold_{0.0};
  double max_thrust_;
  bool use_thrusters_;
  std::unique_ptr<ValveThrustMap> map_;
  std::optional<std::uint8_t> mode_;
  std::array<double, kCount> commanded_{};
  std::array<std::uint16_t, kCount> registers_{};
  bool main_requested_{false};
  bool main_applied_{false};
};
}  // namespace fss_actuators
