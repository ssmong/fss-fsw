#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

namespace fss_hal {

using ImuVector3 = std::array<double, 3>;
using ImuQuaternion = std::array<double, 4>;  // x, y, z, w
using ImuCovariance = std::array<double, 9>;  // row-major
using ImuMatrix3 = std::array<ImuVector3, 3>;

template <std::size_t N>
inline bool imu_finite(const std::array<double, N>& values) {
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

inline ImuQuaternion normalize_imu_quaternion(ImuQuaternion q) {
  const double norm = std::hypot(std::hypot(q[0], q[1]), std::hypot(q[2], q[3]));
  if (!imu_finite(q) || !std::isfinite(norm) || norm < 1e-12) {
    throw std::invalid_argument("IMU quaternion must have finite, nonzero norm");
  }
  for (double& value : q) value /= norm;
  return q;
}

inline ImuMatrix3 imu_rotation_matrix(ImuQuaternion q) {
  q = normalize_imu_quaternion(q);
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  return {{{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
           {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
           {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}}};
}

inline ImuVector3 rotate_imu_vector(const ImuVector3& input, const ImuMatrix3& rotation) {
  ImuVector3 result{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) result[row] += rotation[row][col] * input[col];
  }
  return result;
}

inline ImuCovariance rotate_imu_covariance(const ImuCovariance& input, const ImuMatrix3& rotation) {
  if (input[0] < 0.0) return input;
  ImuCovariance result{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
          result[row * 3 + col] += rotation[row][i] * input[i * 3 + j] * rotation[col][j];
        }
      }
    }
  }
  return result;
}

struct ImuSample {
  std::int64_t stamp_ns{};
  std::string frame_id;
  ImuVector3 angular_velocity{};       // rad/s
  ImuVector3 linear_acceleration{};    // m/s^2
  ImuQuaternion orientation_xyzw{};
  ImuCovariance angular_velocity_covariance{};
  ImuCovariance linear_acceleration_covariance{};
  ImuCovariance orientation_covariance{};
};

class ImuHAL {
 public:
  virtual ~ImuHAL() = default;
  virtual ImuSample convert(const ImuSample& sample) const = 0;
};

// USB and device decoding remain with the external Xsens ROS driver.
class XsensImuHAL final : public ImuHAL {
 public:
  XsensImuHAL(ImuQuaternion mounting_xyzw, std::string frame_id,
              std::function<void(const std::string&)> warn = {})
      : mounting_(normalize_imu_quaternion(mounting_xyzw)),
        rotation_(imu_rotation_matrix(mounting_)), frame_id_(std::move(frame_id)),
        warn_(std::move(warn)) {}

  ImuSample convert(const ImuSample& sample) const override {
    if (!imu_finite(sample.angular_velocity) || !imu_finite(sample.linear_acceleration) ||
        !imu_finite(sample.angular_velocity_covariance) ||
        !imu_finite(sample.linear_acceleration_covariance) ||
        !std::isfinite(sample.orientation_covariance[0])) {
      throw std::invalid_argument("IMU vectors and covariances must be finite");
    }
    ImuSample out = sample;
    if (out.orientation_covariance[0] < 0.0) {
      // Ignore an unavailable orientation payload; valid gyro/accel still pass through.
      out.orientation_xyzw = {};
      out.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    } else if (!imu_finite(out.orientation_xyzw) || !imu_finite(out.orientation_covariance)) {
      throw std::invalid_argument("IMU orientation and covariance must be finite");
    }
    if (warn_) {
      for (double value : sample.angular_velocity) {
        if (std::abs(value) > 34.0) { warn_("IMU gyro saturation warning (>34 rad/s)"); break; }
      }
      for (double value : sample.linear_acceleration) {
        if (std::abs(value) > 190.0) { warn_("IMU acceleration saturation warning (>190 m/s^2)"); break; }
      }
    }
    const auto& q = out.orientation_xyzw;
    const double norm = std::hypot(std::hypot(q[0], q[1]), std::hypot(q[2], q[3]));
    if (out.orientation_covariance[0] < 0.0 || norm < 1e-12) {
      out.orientation_covariance[0] = -1.0;
    } else {
      const auto source = normalize_imu_quaternion(q);
      const ImuQuaternion inverse{-mounting_[0], -mounting_[1], -mounting_[2], mounting_[3]};
      const double x = source[0], y = source[1], z = source[2], w = source[3];
      const double rx = inverse[0], ry = inverse[1], rz = inverse[2], rw = inverse[3];
      // q_world_from_body = q_world_from_imu * q_imu_from_body.
      out.orientation_xyzw = normalize_imu_quaternion(
          {w * rx + x * rw + y * rz - z * ry, w * ry - x * rz + y * rw + z * rx,
           w * rz + x * ry - y * rx + z * rw, w * rw - x * rx - y * ry - z * rz});
      out.orientation_covariance = rotate_imu_covariance(out.orientation_covariance, rotation_);
    }
    out.frame_id = frame_id_;
    out.angular_velocity = rotate_imu_vector(sample.angular_velocity, rotation_);
    out.linear_acceleration = rotate_imu_vector(sample.linear_acceleration, rotation_);
    out.angular_velocity_covariance = rotate_imu_covariance(sample.angular_velocity_covariance, rotation_);
    out.linear_acceleration_covariance = rotate_imu_covariance(sample.linear_acceleration_covariance, rotation_);
    if (!imu_finite(out.angular_velocity) || !imu_finite(out.linear_acceleration) ||
        !imu_finite(out.angular_velocity_covariance) ||
        !imu_finite(out.linear_acceleration_covariance) || !imu_finite(out.orientation_covariance)) {
      throw std::invalid_argument("IMU mounting transform overflow");
    }
    return out;
  }

 private:
  ImuQuaternion mounting_;
  ImuMatrix3 rotation_;
  std::string frame_id_;
  std::function<void(const std::string&)> warn_;
};

}  // namespace fss_hal
