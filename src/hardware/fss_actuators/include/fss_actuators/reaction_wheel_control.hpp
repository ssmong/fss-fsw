#pragma once

#include "fss_hal/reaction_wheel.hpp"

#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

namespace fss_actuators {
class ReactionWheelControl {
 public:
  ReactionWheelControl(fss_hal::ReactionWheelHal& hal, fss_hal::ReactionWheelConfig config,
                       fss_hal::WheelClock clock = fss_hal::monotonic_seconds)
      : hal_(hal), config_(std::move(config)), clock_(std::move(clock)) { config_.validate(); }
  void enable() {
    if (enabled_) return;
    const double now = clock_();
    if (!hal_.is_connected() || !fresh_status(now) || status_->state != 0 ||
        !hal_.stop_confirmed() || hal_.fault_latched())
      throw std::runtime_error("Fresh STOP telemetry is required before enabling");
    enabled_ = true;
    last_error_.clear();
    enabled_at_ = now;
    clear_command();
  }
  void disable(const std::string& reason = "") {
    enabled_ = false;
    clear_command();
    last_error_ = reason;
    hal_.stop();
  }
  void command(double torque) {
    if (!std::isfinite(torque) || std::abs(torque) > config_.max_torque_nm) {
      disable("Wheel torque command is outside the valid range");
      throw std::invalid_argument(last_error_);
    }
    if (enabled_) { command_ = torque; command_at_ = clock_(); }
  }
  void step() {
    try {
      // Check before RX: queued frames after a host stall cannot conceal an expired measurement.
      if (enabled_ && !fresh_status(clock_())) disable("Reaction wheel telemetry timeout");
      if (const auto sample = hal_.read_status()) status_ = sample;
      if (hal_.fault_latched() || (status_ && status_->state == 2)) {
        const std::string reason = hal_.fault_reason().empty() ? "MCU FAULT" : hal_.fault_reason();
        if (enabled_) disable(reason);
        else last_error_ = reason;
        return;
      }
      if (!enabled_) return;
      const double now = clock_();
      if (!fresh_status(now)) { disable("Reaction wheel telemetry timeout"); return; }
      if (now - command_at_.value_or(*enabled_at_) >= config_.command_timeout_sec) {
        disable("Reaction wheel command timeout");
        return;
      }
      if (command_ && (!last_write_ || now - *last_write_ >= 1.0 / config_.command_rate_hz)) {
        hal_.set_torque(*command_);
        last_write_ = now;
      }
    } catch (const std::exception& e) {
      enabled_ = false;
      clear_command();
      last_error_ = e.what();
      try { hal_.stop(); }
      catch (const std::exception& stop_error) { last_error_ += "; STOP failed: " + std::string(stop_error.what()); }
      throw;
    }
  }
  bool enabled() const { return enabled_; }
  const std::optional<fss_hal::WheelStatus>& status() const { return status_; }
  const std::string& last_error() const { return last_error_; }
 private:
  bool fresh_status(double now) const {
    return status_ && now >= status_->received_at && now - status_->received_at < config_.telemetry_timeout_sec;
  }
  void clear_command() { command_.reset(); command_at_.reset(); last_write_.reset(); }
  fss_hal::ReactionWheelHal& hal_;
  fss_hal::ReactionWheelConfig config_;
  fss_hal::WheelClock clock_;
  bool enabled_{false};
  std::optional<fss_hal::WheelStatus> status_;
  std::string last_error_;
  std::optional<double> command_, command_at_, enabled_at_, last_write_;
};
}  // namespace fss_actuators
