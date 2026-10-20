#include "fss_navigation/ekf.hpp"

#include <Eigen/Cholesky>

#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace fss_navigation {
namespace {
constexpr double kPi = 3.14159265358979323846;
double square(double value) { return value * value; }
}  // namespace

double covariance_to_std(double variance) {
    if (!std::isfinite(variance)) {
        return std::numeric_limits<double>::infinity();
    }
    return variance < 0.0 ? 0.0 : std::sqrt(variance);
}

double wrap_angle(double angle) {
    double wrapped = std::fmod(angle + kPi, 2.0 * kPi);
    if (wrapped < 0.0) {
        wrapped += 2.0 * kPi;
    }
    return wrapped - kPi;
}

PlanarEKF::PlanarEKF(const EKFParameters& parameters) : parameters_(parameters) {
    for (double noise : {parameters.gyro_noise_density, parameters.accel_noise_density,
                         parameters.gyro_bias_random_walk, parameters.accel_bias_random_walk,
                         parameters.table_accel_bias_random_walk}) {
        if (!std::isfinite(noise) || noise < 0.0) {
            throw std::invalid_argument("EKF noise densities must be finite and nonnegative");
        }
    }
    for (double positive : {parameters.vicon_position_noise, parameters.vicon_heading_noise,
                            parameters.gate_threshold}) {
        if (!std::isfinite(positive) || positive <= 0.0) {
            throw std::invalid_argument("Vicon noise and gate threshold must be finite and positive");
        }
    }
    measurement_noise_ = Eigen::Vector3d(square(parameters.vicon_position_noise),
                                        square(parameters.vicon_position_noise),
                                        square(parameters.vicon_heading_noise)).asDiagonal();
}

void PlanarEKF::initialize(double px, double py, double theta) {
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(theta)) {
        return;
    }
    x.setZero();
    x.head<3>() << px, py, theta;
    P.setZero();
    P.diagonal() << 1e-4, 1e-4, 1e-4, 0.01, 0.01, 1e-4, 1e-5, 1e-4, 1e-4, 2.5e-3, 2.5e-3;
    initialized_ = true;
}

void PlanarEKF::predict(double dt, double omega_imu, double ax_imu, double ay_imu) {
    if (!initialized_ || !std::isfinite(dt) || dt <= 0.0 || !std::isfinite(omega_imu) ||
        !std::isfinite(ax_imu) || !std::isfinite(ay_imu)) {
        return;
    }
    const double omega_c = omega_imu - x[IBG];
    const double ax_c = ax_imu - x[IBAX];
    const double ay_c = ay_imu - x[IBAY];
    const double cosine = std::cos(x[ITHETA]);
    const double sine = std::sin(x[ITHETA]);
    const double acc_x = ax_c * cosine - ay_c * sine + x[IBTX];
    const double acc_y = ax_c * sine + ay_c * cosine + x[IBTY];
    const double theta_mid = x[ITHETA] + 0.5 * dt * omega_c;
    const double cosine_mid = std::cos(theta_mid);
    const double sine_mid = std::sin(theta_mid);

    x[IPX] += dt * (x[IVX] + 0.5 * dt * acc_x);
    x[IPY] += dt * (x[IVY] + 0.5 * dt * acc_y);
    x[ITHETA] = wrap_angle(x[ITHETA] + dt * omega_c);
    x[IVX] += dt * (ax_c * cosine_mid - ay_c * sine_mid + x[IBTX]);
    x[IVY] += dt * (ax_c * sine_mid + ay_c * cosine_mid + x[IBTY]);
    x[IOMEGA] = omega_c;

    Covariance F = Covariance::Identity();
    F(IPX, IVX) = dt;
    F(IPY, IVY) = dt;
    F(ITHETA, IBG) = -dt;
    F(IVX, ITHETA) = (-ax_c * sine - ay_c * cosine) * dt;
    F(IVY, ITHETA) = (ax_c * cosine - ay_c * sine) * dt;
    F(IVX, IBAX) = -cosine * dt;
    F(IVX, IBAY) = sine * dt;
    F(IVY, IBAX) = -sine * dt;
    F(IVY, IBAY) = -cosine * dt;
    F(IVX, IBTX) = dt;
    F(IVY, IBTY) = dt;
    F.row(IOMEGA).setZero();
    F(IOMEGA, IBG) = -1.0;

    Covariance Q = Covariance::Zero();
    Q.diagonal() << 1e-10 * dt, 1e-10 * dt, square(parameters_.gyro_noise_density) * dt,
        square(parameters_.accel_noise_density) * dt,
        square(parameters_.accel_noise_density) * dt,
        square(parameters_.gyro_noise_density) / dt,
        square(parameters_.gyro_bias_random_walk) * dt,
        square(parameters_.accel_bias_random_walk) * dt,
        square(parameters_.accel_bias_random_walk) * dt,
        square(parameters_.table_accel_bias_random_walk) * dt,
        square(parameters_.table_accel_bias_random_walk) * dt;
    P = (F * P * F.transpose() + Q).eval();
    P = (0.5 * (P + P.transpose())).eval();
}

bool PlanarEKF::update_vicon(double px, double py, double theta) {
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(theta)) {
        return false;
    }
    if (!initialized_) {
        initialize(px, py, theta);
        return true;
    }
    Eigen::Vector3d innovation = Eigen::Vector3d(px, py, theta) - x.head<3>();
    innovation[2] = wrap_angle(innovation[2]);
    const Eigen::Matrix3d S = P.topLeftCorner<3, 3>() + measurement_noise_;
    const Eigen::LDLT<Eigen::Matrix3d> factor(S);
    if (!S.allFinite() || !x.allFinite() || !P.allFinite() ||
        factor.info() != Eigen::Success || !factor.isPositive() ||
        (factor.vectorD().array() <= 0.0).any()) {
        record_nis(std::numeric_limits<double>::infinity());
        return false;
    }
    const double nis = innovation.dot(factor.solve(innovation));
    record_nis(nis);
    if (!std::isfinite(nis) || nis > parameters_.gate_threshold) {
        return false;
    }
    const Eigen::Matrix<double, N_STATES, 3> K =
        P.leftCols<3>() * factor.solve(Eigen::Matrix3d::Identity());
    x += K * innovation;
    x[ITHETA] = wrap_angle(x[ITHETA]);
    Covariance I_KH = Covariance::Identity();
    I_KH.leftCols<3>() -= K;
    P = (I_KH * P * I_KH.transpose() + K * measurement_noise_ * K.transpose()).eval();
    P = (0.5 * (P + P.transpose())).eval();
    return true;
}

void PlanarEKF::record_nis(double nis) {
    if (parameters_.nis_window == 0) {
        return;
    }
    nis_history_.push_back(nis);
    if (nis_history_.size() > parameters_.nis_window) {
        nis_history_.pop_front();
    }
}

double PlanarEKF::nis_mean() const {
    return nis_history_.empty() ? 0.0 :
        std::accumulate(nis_history_.begin(), nis_history_.end(), 0.0) / nis_history_.size();
}

bool PlanarEKF::filter_consistent() const {
    return nis_history_.size() < 10 || (nis_mean() > 0.5 && nis_mean() < 8.0);
}

}  // namespace fss_navigation
