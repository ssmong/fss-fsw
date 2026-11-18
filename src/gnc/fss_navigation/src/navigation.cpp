#include "fss_navigation/navigation.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace fss_navigation {
namespace {
std::int64_t checked_timeout(double timeout_ms, std::int64_t cycles) {
    if (!std::isfinite(timeout_ms) || timeout_ms <= 0.0 ||
        timeout_ms * 1e6 >= static_cast<double>(std::numeric_limits<std::int64_t>::max()) ||
        cycles < 0 || static_cast<std::int64_t>(timeout_ms * 1e6) == 0) {
        throw std::invalid_argument("Navigation timeout must be positive and cycles nonnegative");
    }
    return static_cast<std::int64_t>(timeout_ms * 1e6);
}
}  // namespace

Navigation::Navigation(const EKFParameters& parameters, double timeout_ms,
                       std::int64_t max_prediction_cycles, const NavigationTiming& timing)
    : filter_(parameters), timeout_ns_(checked_timeout(timeout_ms, max_prediction_cycles)),
      max_prediction_cycles_(max_prediction_cycles), delay_ns_(0), fusion_(filter_, timing, timeout_ns_) {
    delay_ns_ = std::llround(timing.delay_ms * 1e6);
}

bool Navigation::fresh(std::int64_t now_ns, std::int64_t stamp_ns) const {
    const long double age = static_cast<long double>(now_ns) - stamp_ns;
    return age >= 0 && age <= timeout_ns_;
}

void Navigation::suspend() {
    needs_vicon_initialization_ = true;
    last_prediction_time_ns_.reset();
    fusion_.clear();
    pending_vicon_.clear();
}

InputResult Navigation::on_imu(std::int64_t now_ns, std::int64_t stamp_ns, double omega,
                               double ax, double ay, bool gyro_available, bool accel_available) {
    if (!gyro_available || !accel_available || !std::isfinite(omega) || !std::isfinite(ax) ||
        !std::isfinite(ay)) {
        clear_recovery();
        return InputResult::Invalid;
    }
    if (!fresh(now_ns, stamp_ns)) {
        clear_recovery();
        return InputResult::Stale;
    }
    if (last_imu_time_ns_ && stamp_ns <= *last_imu_time_ns_) {
        return InputResult::Reordered;
    }
    if ((fusion_.time_ns() && stamp_ns - *fusion_.time_ns() > timeout_ns_) ||
        (last_imu_time_ns_ && stamp_ns - *last_imu_time_ns_ > timeout_ns_) ||
        (last_prediction_time_ns_ && now_ns - *last_prediction_time_ns_ > timeout_ns_)) {
        suspend();
    }
    if (!needs_vicon_initialization_) {
        fusion_.predict(stamp_ns, Eigen::Vector3d(omega, ax, ay));
        drain_vicon(now_ns);
        if (last_imu_time_ns_ && last_vicon_time_ns_) {
            last_prediction_time_ns_ = fusion_.time_ns();
        }
    }
    last_imu_time_ns_ = stamp_ns;
    return needs_vicon_initialization_ ? InputResult::Resynchronizing : InputResult::Accepted;
}

InputResult Navigation::on_vicon(std::int64_t now_ns, std::int64_t stamp_ns, double px,
                                 double py, double theta) {
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(theta)) {
        clear_recovery();
        return InputResult::Invalid;
    }
    if (!fresh(now_ns, stamp_ns)) {
        clear_recovery();
        return InputResult::Stale;
    }
    if (last_vicon_seen_ns_ && stamp_ns <= *last_vicon_seen_ns_) {
        return InputResult::Reordered;
    }
    last_vicon_seen_ns_ = stamp_ns;
    const auto measurement_ns = stamp_ns - delay_ns_;
    const Eigen::Vector3d pose(px, py, theta);
    if (needs_vicon_initialization_ || !fusion_.time_ns()) {
        fusion_.initialize(measurement_ns, pose);
        pending_vicon_.clear();
        needs_vicon_initialization_ = false;
        last_imu_time_ns_.reset();
        TimedUpdate result;
        result.accepted = true;
        record_vicon(now_ns, stamp_ns, result);
        return InputResult::Accepted;
    }
    if (pending_vicon_.size() == 10) {
        pending_vicon_.pop_front();
        clear_recovery();
    }
    pending_vicon_.push_back(PendingVicon{measurement_ns, stamp_ns, pose});
    const bool queued = measurement_ns > *fusion_.time_ns();
    drain_vicon(now_ns);
    return queued || last_update_.accepted ? InputResult::Accepted : InputResult::Rejected;
}

void Navigation::drain_vicon(std::int64_t now_ns) {
    while (!pending_vicon_.empty()) {
        const auto sample = pending_vicon_.front();
        if (!fresh(now_ns, sample.stamp)) {
            pending_vicon_.pop_front();
            clear_recovery();
            continue;
        }
        if (!fusion_.time_ns() || sample.measurement > *fusion_.time_ns()) {
            break;
        }
        pending_vicon_.pop_front();
        record_vicon(now_ns, sample.stamp, fusion_.update(sample.measurement, sample.pose));
    }
}

void Navigation::record_vicon(std::int64_t now_ns, std::int64_t stamp_ns, const TimedUpdate& result) {
    ++update_count_;
    last_update_ = result;
    if (!result.accepted) { return; }
    last_vicon_time_ns_ = now_ns;
    last_vicon_stamp_ns_ = stamp_ns;
    prediction_only_count_ = 0;
    navigation_degraded_ = false;
}

std::optional<NavigationState> Navigation::state(std::int64_t now_ns) {
    if (!last_prediction_time_ns_) { return std::nullopt; }
    if (!fresh(now_ns, *last_prediction_time_ns_)) {
        suspend();
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
