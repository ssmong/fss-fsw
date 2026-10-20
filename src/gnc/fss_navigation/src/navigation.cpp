#include "fss_navigation/navigation.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace fss_navigation {

Navigation::Navigation(const EKFParameters& parameters, double timeout_ms,
                       std::int64_t max_prediction_cycles)
    : filter_(parameters), max_prediction_cycles_(max_prediction_cycles) {
    if (!std::isfinite(timeout_ms) || timeout_ms <= 0.0 ||
        timeout_ms * 1e6 >= static_cast<double>(std::numeric_limits<std::int64_t>::max()) ||
        max_prediction_cycles < 0) {
        throw std::invalid_argument("Navigation timeout must be positive and cycles nonnegative");
    }
    timeout_ns_ = static_cast<std::int64_t>(timeout_ms * 1e6);
    if (timeout_ns_ == 0) {
        throw std::invalid_argument("Navigation timeout must be at least one nanosecond");
    }
}

bool Navigation::fresh(std::int64_t now_ns, std::int64_t stamp_ns) const {
    return std::abs(static_cast<long double>(now_ns) - stamp_ns) <= timeout_ns_;
}

InputResult Navigation::on_imu(std::int64_t now_ns, std::int64_t stamp_ns, double omega,
                               double ax, double ay, bool gyro_available, bool accel_available) {
    if (!gyro_available || !accel_available || !std::isfinite(omega) || !std::isfinite(ax) ||
        !std::isfinite(ay)) {
        return InputResult::Invalid;
    }
    if (!fresh(now_ns, stamp_ns)) {
        return InputResult::Stale;
    }
    if (last_imu_time_ns_) {
        const auto gap_ns = stamp_ns - *last_imu_time_ns_;
        if (gap_ns <= 0) {
            return InputResult::Reordered;
        }
        if (gap_ns > timeout_ns_ ||
            (last_prediction_time_ns_ && now_ns - *last_prediction_time_ns_ > timeout_ns_)) {
            needs_vicon_initialization_ = true;
            last_prediction_time_ns_.reset();
        }
        if (!needs_vicon_initialization_) {
            filter_.predict(static_cast<double>(gap_ns) * 1e-9, omega, ax, ay);
            if (last_vicon_time_ns_) {
                last_prediction_time_ns_ = stamp_ns;
            }
        }
    }
    last_imu_time_ns_ = stamp_ns;
    return needs_vicon_initialization_ ? InputResult::Resynchronizing : InputResult::Accepted;
}

InputResult Navigation::on_vicon(std::int64_t now_ns, std::int64_t stamp_ns, double px,
                                 double py, double theta) {
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(theta)) {
        return InputResult::Invalid;
    }
    if (!fresh(now_ns, stamp_ns)) {
        return InputResult::Stale;
    }
    if (last_vicon_stamp_ns_ && stamp_ns <= *last_vicon_stamp_ns_ && !needs_vicon_initialization_) {
        return InputResult::Reordered;
    }
    if (needs_vicon_initialization_) {
        filter_.initialize(px, py, theta);
        needs_vicon_initialization_ = false;
        last_imu_time_ns_.reset();
    } else if (!filter_.update_vicon(px, py, theta)) {
        return InputResult::Rejected;
    }
    last_vicon_time_ns_ = now_ns;
    last_vicon_stamp_ns_ = stamp_ns;
    prediction_only_count_ = 0;
    navigation_degraded_ = false;
    return InputResult::Accepted;
}

std::optional<NavigationState> Navigation::state(std::int64_t now_ns) {
    if (!last_prediction_time_ns_) {
        return std::nullopt;
    }
    const auto age_ns = now_ns - *last_prediction_time_ns_;
    if (age_ns < 0 || age_ns > timeout_ns_) {
        needs_vicon_initialization_ = true;
        last_prediction_time_ns_.reset();
        return std::nullopt;
    }
    if (last_vicon_time_ns_ && now_ns - *last_vicon_time_ns_ > timeout_ns_) {
        if (prediction_only_count_ < std::numeric_limits<std::int64_t>::max()) {
            ++prediction_only_count_;
        }
        if (prediction_only_count_ > max_prediction_cycles_) {
            navigation_degraded_ = true;
        }
    }
    return NavigationState{*last_prediction_time_ns_, filter_.x,
                           covariance_to_std(filter_.P(PlanarEKF::IPX, PlanarEKF::IPX)),
                           covariance_to_std(filter_.P(PlanarEKF::IPY, PlanarEKF::IPY)),
                           covariance_to_std(filter_.P(PlanarEKF::ITHETA, PlanarEKF::ITHETA))};
}

}  // namespace fss_navigation
