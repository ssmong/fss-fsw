#pragma once

#include <string>
#include <vector>

namespace fss_actuators {
class ValveThrustMap {
 public:
  ValveThrustMap(std::vector<double> duty, std::vector<std::vector<double>> thrust,
                 std::size_t valve_count = 8);
  static ValveThrustMap from_yaml(const std::string& filename, std::size_t valve_count = 8);
  std::vector<double> thrust_from_duty(const std::vector<double>& values) const;
  std::vector<double> duty_from_thrust(const std::vector<double>& values) const;
 private:
  void validate_values(const std::vector<double>& values) const;
  std::vector<double> duty_;
  std::vector<std::vector<double>> thrust_;
};
}  // namespace fss_actuators
