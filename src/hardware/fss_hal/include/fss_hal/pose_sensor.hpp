#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>

namespace fss_hal {

inline constexpr double kPi = 3.14159265358979323846;

struct PlanarPoseSample {
  double x_m{};
  double y_m{};
  double yaw_rad{};
  std::uint64_t frame_number{};
  double source_time_sec{};
};

class PoseSensorHAL {
 public:
  virtual ~PoseSensorHAL() = default;
  virtual void connect() = 0;
  virtual void disconnect() noexcept = 0;
  virtual std::optional<PlanarPoseSample> read() = 0;
};

inline double wrap_planar_angle(double angle) {
  double wrapped = std::fmod(angle + kPi, 2.0 * kPi);
  if (wrapped < 0.0) wrapped += 2.0 * kPi;
  return wrapped - kPi;
}

struct PlanarState {
  double x{}, y{}, theta{}, vx{}, vy{}, omega{};
};

// Vicon source time supplies derivatives; the ROS bridge supplies its receive stamp.
class PlanarPoseTransform {
 public:
  PlanarPoseTransform(double offset_x, double offset_y, double rotation_rad)
      : offset_x_(offset_x), offset_y_(offset_y), rotation_(rotation_rad) {
    if (!std::isfinite(offset_x_) || !std::isfinite(offset_y_) ||
        !std::isfinite(rotation_)) {
      throw std::invalid_argument("Vicon frame transform must be finite");
    }
  }

  PlanarState convert(const PlanarPoseSample& sample) {
    if (!std::isfinite(sample.x_m) || !std::isfinite(sample.y_m) ||
        !std::isfinite(sample.yaw_rad) || !std::isfinite(sample.source_time_sec)) {
      throw std::invalid_argument("Vicon pose and source time must be finite");
    }
    const double c = std::cos(rotation_), s = std::sin(rotation_);
    PlanarState state;
    state.x = c * sample.x_m - s * sample.y_m + offset_x_;
    state.y = s * sample.x_m + c * sample.y_m + offset_y_;
    state.theta = wrap_planar_angle(sample.yaw_rad + rotation_);
    if (previous_) {
      const double dt = sample.source_time_sec - previous_time_;
      if (dt > 1e-6 && dt < 1.0) {
        state.vx = (state.x - previous_->x) / dt;
        state.vy = (state.y - previous_->y) / dt;
        state.omega = wrap_planar_angle(state.theta - previous_->theta) / dt;
      }
    }
    if (!std::isfinite(state.x) || !std::isfinite(state.y) ||
        !std::isfinite(state.theta) || !std::isfinite(state.vx) ||
        !std::isfinite(state.vy) || !std::isfinite(state.omega)) {
      throw std::invalid_argument("Vicon transform overflow");
    }
    previous_ = state;
    previous_time_ = sample.source_time_sec;
    return state;
  }

 private:
  double offset_x_, offset_y_, rotation_;
  std::optional<PlanarState> previous_;
  double previous_time_{};
};

}  // namespace fss_hal
