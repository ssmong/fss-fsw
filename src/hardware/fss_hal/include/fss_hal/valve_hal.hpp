#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace fss_hal {

inline std::uint16_t valve_register_value(double value, int maximum) {
  return std::isfinite(value)
      ? static_cast<std::uint16_t>(std::clamp(value, 0.0, 1.0) * maximum) : 0;
}

class ValveHal {
 public:
  virtual ~ValveHal() = default;
  virtual void connect() = 0;
  virtual void disconnect() = 0;
  virtual void set_valves(const std::vector<double>& values) = 0;
  virtual void set_main_valve(bool open) = 0;
  virtual void emergency_stop() = 0;
  virtual bool is_connected() const = 0;
  virtual double last_rtt_ms() const { return 0.0; }
};

class SimHal final : public ValveHal {
 public:
  explicit SimHal(std::size_t count = 8, int maximum = 4000)
      : values_(count, 0), maximum_(maximum) {
    if (count == 0 || maximum <= 0 || maximum > 65535)
      throw std::invalid_argument("Invalid simulated valve configuration");
  }
  void connect() override { connected_ = true; emergency_stop(); }
  void disconnect() override { emergency_stop(); connected_ = false; }
  void set_valves(const std::vector<double>& values) override {
    require_connection();
    if (values.size() != values_.size()) throw std::invalid_argument("Valve count mismatch");
    for (std::size_t i = 0; i < values.size(); ++i)
      values_[i] = valve_register_value(values[i], maximum_);
  }
  void set_main_valve(bool open) override { require_connection(); main_open_ = open; }
  void emergency_stop() override { std::fill(values_.begin(), values_.end(), 0); main_open_ = false; }
  bool is_connected() const override { return connected_; }
  const std::vector<std::uint16_t>& values() const { return values_; }
  bool main_valve_open() const { return main_open_; }
 private:
  void require_connection() const {
    if (!connected_) throw std::runtime_error("Simulated valves are disconnected");
  }
  std::vector<std::uint16_t> values_;
  int maximum_;
  bool connected_{false};
  bool main_open_{false};
};
}  // namespace fss_hal
