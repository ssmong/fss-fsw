#include "fss_actuators/valve_thrust_map.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace fss_actuators {
ValveThrustMap::ValveThrustMap(std::vector<double> duty, std::vector<std::vector<double>> thrust,
                             std::size_t count)
    : duty_(std::move(duty)), thrust_(std::move(thrust)) {
  if (count == 0 || duty_.size() < 2 || thrust_.size() != count ||
      duty_.front() != 0.0 || duty_.back() != 1.0)
    throw std::invalid_argument("Invalid valve thrust map dimensions or input endpoints");
  for (std::size_t i = 0; i < duty_.size(); ++i) {
    if (!std::isfinite(duty_[i]) || (i > 0 && duty_[i] <= duty_[i - 1]))
      throw std::invalid_argument("Valve map duty must increase strictly from zero to one");
  }
  for (const auto& row : thrust_) {
    if (row.size() != duty_.size() || row.front() != 0.0 || row.back() <= 0.0)
      throw std::invalid_argument("Invalid valve thrust row dimensions or endpoints");
    for (std::size_t i = 0; i < row.size(); ++i) {
      if (!std::isfinite(row[i]) || (i > 0 && row[i] < row[i - 1]))
        throw std::invalid_argument("Valve thrust curves must be finite and nondecreasing");
    }
  }
}

ValveThrustMap ValveThrustMap::from_yaml(const std::string& filename, std::size_t count) {
  const auto data = YAML::LoadFile(filename);
  if (!data.IsMap() || !data["duty"] || !data["thrust_n"])
    throw std::invalid_argument("Valve map requires duty and thrust_n");
  return ValveThrustMap(data["duty"].as<std::vector<double>>(),
                       data["thrust_n"].as<std::vector<std::vector<double>>>(), count);
}

void ValveThrustMap::validate_values(const std::vector<double>& values) const {
  if (values.size() != thrust_.size() ||
      !std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); }))
    throw std::invalid_argument("Valve map inputs must be finite and match the valve count");
}

std::vector<double> ValveThrustMap::thrust_from_duty(const std::vector<double>& values) const {
  validate_values(values);
  std::vector<double> result(values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    const double value = std::clamp(values[i], 0.0, 1.0);
    const auto hi = std::lower_bound(duty_.begin() + 1, duty_.end(), value);
    const auto index = static_cast<std::size_t>(hi - duty_.begin());
    const double fraction = (value - duty_[index - 1]) / (duty_[index] - duty_[index - 1]);
    result[i] = thrust_[i][index - 1] + fraction * (thrust_[i][index] - thrust_[i][index - 1]);
  }
  return result;
}

std::vector<double> ValveThrustMap::duty_from_thrust(const std::vector<double>& values) const {
  validate_values(values);
  std::vector<double> result(values.size(), 0.0);
  for (std::size_t i = 0; i < values.size(); ++i) {
    const auto& row = thrust_[i];
    const double target = std::clamp(values[i], 0.0, row.back());
    if (target == 0.0) continue;
    // lower_bound chooses the smallest input on a flat thrust section.
    const auto hi = std::lower_bound(row.begin(), row.end(), target);
    const auto index = static_cast<std::size_t>(hi - row.begin());
    const double fraction = (target - row[index - 1]) / (row[index] - row[index - 1]);
    result[i] = duty_[index - 1] + fraction * (duty_[index] - duty_[index - 1]);
  }
  return result;
}
}  // namespace fss_actuators
