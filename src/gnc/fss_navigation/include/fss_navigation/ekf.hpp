#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <deque>

namespace fss_navigation {

struct EKFParameters {
    double gyro_noise_density;
    double accel_noise_density;
    double gyro_bias_random_walk;
    double accel_bias_random_walk;
    double vicon_position_noise;
    double vicon_heading_noise;
    double gate_threshold = 11.345;
    std::size_t nis_window = 50;
    double table_accel_bias_random_walk = 1.0e-5;
    double vicon_timestamp_std_ms = 0.0;
    double accel_model_noise_density = 0.0;
};

double covariance_to_std(double variance);
double wrap_angle(double angle);

// State and covariance use the same order and units as the Python research reference.
class PlanarEKF {
public:
    enum Index { IPX, IPY, ITHETA, IVX, IVY, IOMEGA, IBG, IBAX, IBAY, IBTX, IBTY, N_STATES };
    using State = Eigen::Matrix<double, N_STATES, 1>;
    using Covariance = Eigen::Matrix<double, N_STATES, N_STATES>;

    explicit PlanarEKF(const EKFParameters& parameters);
    void initialize(double px, double py, double theta, bool reset_nis = false);
    void predict(double dt, double omega_imu, double ax_imu, double ay_imu);
    bool update_vicon(double px, double py, double theta, bool gate_only = false);
    double nis_mean() const;
    bool filter_consistent() const;
    bool initialized() const { return initialized_; }

    State x = State::Zero();
    Covariance P = Covariance::Identity() * 0.1;
    double last_vicon_nis = 0.0;
    Eigen::Vector3d last_vicon_innovation = Eigen::Vector3d::Zero();

private:
    void record_nis(double nis);
    EKFParameters parameters_;
    Eigen::Matrix3d measurement_noise_;
    std::deque<double> nis_history_;
    bool initialized_ = false;
};

}  // namespace fss_navigation
