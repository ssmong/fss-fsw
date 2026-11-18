#include "fss_navigation/timed_ekf.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace fss_navigation {
namespace {
constexpr double kTwoPi = 6.28318530717958647692;
Eigen::Vector3d pose_delta(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    Eigen::Vector3d delta = a - b;
    delta[2] = std::remainder(delta[2], kTwoPi);
    return delta;
}
}  // namespace

TimedEKF::TimedEKF(PlanarEKF& filter, const NavigationTiming& timing, std::int64_t timeout_ns)
    : filter_(filter), timing_(timing), timeout_ns_(timeout_ns) {
    if (!std::isfinite(timing.delay_ms) || timing.delay_ms < 0.0 ||
        timing.delay_ms * 1e6 >= timeout_ns || !std::isfinite(timing.history_ms) ||
        timing.history_ms * 1e6 < timeout_ns + timing.delay_ms * 1e6 ||
        timing.history_ms * 1e6 >= static_cast<double>(std::numeric_limits<std::int64_t>::max()) ||
        timing.recovery_samples < 3) {
        throw std::invalid_argument("Invalid Vicon delay/history/recovery parameters");
    }
    for (double limit : {timing.recovery_max_position, timing.recovery_max_heading,
                         timing.recovery_max_speed, timing.recovery_max_yaw_rate}) {
        if (!std::isfinite(limit) || limit <= 0.0) {
            throw std::invalid_argument("Vicon recovery limits must be finite and positive");
        }
    }
    history_ns_ = std::llround(timing.history_ms * 1e6);
}

void TimedEKF::clear() {
    time_ns_.reset();
    last_measurement_ns_.reset();
    last_accepted_ns_.reset();
    steps_.clear();
    clear_recovery();
}

void TimedEKF::clear_recovery() { rejected_.clear(); }

void TimedEKF::initialize(std::int64_t stamp_ns, const Eigen::Vector3d& pose) {
    clear();
    filter_.initialize(pose[0], pose[1], pose[2], true);
    time_ns_ = last_measurement_ns_ = last_accepted_ns_ = stamp_ns;
}

void TimedEKF::predict(std::int64_t stamp_ns, const Eigen::Vector3d& inputs) {
    if (!time_ns_ || stamp_ns <= *time_ns_) {
        return;
    }
    steps_.push_back(Step{*time_ns_, stamp_ns, inputs, filter_.x, filter_.P});
    filter_.predict((stamp_ns - *time_ns_) * 1e-9, inputs[0], inputs[1], inputs[2]);
    time_ns_ = stamp_ns;
    while (!steps_.empty() && (steps_.front().end < stamp_ns - history_ns_ || steps_.size() > 512)) {
        steps_.pop_front();
    }
}

TimedUpdate TimedEKF::update(std::int64_t stamp_ns, const Eigen::Vector3d& pose) {
    if (!time_ns_ || stamp_ns > *time_ns_ ||
        (last_measurement_ns_ && stamp_ns <= *last_measurement_ns_) ||
        (stamp_ns < *time_ns_ && (steps_.empty() || stamp_ns < steps_.front().start))) {
        clear_recovery();
        TimedUpdate result;
        result.outside_history = true;
        return result;
    }
    const auto original_x = filter_.x;
    const auto original_P = filter_.P;
    const auto original_time = time_ns_;
    auto original_steps = steps_;
    std::vector<std::pair<std::int64_t, Eigen::Vector3d>> replay;
    if (stamp_ns < *time_ns_) {
        std::size_t index = 0;
        while (original_steps[index].end <= stamp_ns) { ++index; }
        const auto& step = original_steps[index];
        filter_.x = step.x;
        filter_.P = step.P;
        time_ns_ = step.start;
        steps_.erase(steps_.begin() + index, steps_.end());
        predict(stamp_ns, step.inputs);
        for (; index < original_steps.size(); ++index) {
            replay.emplace_back(original_steps[index].end, original_steps[index].inputs);
        }
    }
    const Eigen::Vector3d predicted = filter_.x.head<3>();
    const auto correction = pose_delta(pose, predicted);
    const bool inconsistent = !rejected_.empty() && last_accepted_ns_ &&
        stamp_ns - *last_accepted_ns_ > timeout_ns_ &&
        (!consistent_step(stamp_ns, pose, predicted) ||
         correction.head<2>().norm() > timing_.recovery_max_position ||
         std::abs(correction[2]) > timing_.recovery_max_heading);
    bool accepted = filter_.update_vicon(pose[0], pose[1], pose[2], inconsistent);
    TimedUpdate result;
    result.nis = filter_.last_vicon_nis;
    result.innovation = filter_.last_vicon_innovation;
    if (accepted) {
        clear_recovery();
    } else {
        result.recovered = try_recovery(stamp_ns, pose, predicted);
        accepted = result.recovered;
    }
    last_measurement_ns_ = stamp_ns;
    if (accepted) {
        last_accepted_ns_ = stamp_ns;
        for (const auto& [end, inputs] : replay) { predict(end, inputs); }
    } else {
        // A rejected delayed measurement must leave no split-step numerical drift.
        filter_.x = original_x;
        filter_.P = original_P;
        time_ns_ = original_time;
        steps_ = std::move(original_steps);
    }
    result.accepted = accepted;
    return result;
}

bool TimedEKF::consistent_step(std::int64_t stamp_ns, const Eigen::Vector3d& pose,
                              const Eigen::Vector3d& predicted) const {
    const auto& previous = rejected_.back();
    const auto gap = stamp_ns - previous.stamp;
    const double dt = gap * 1e-9;
    const auto delta = pose_delta(pose, previous.pose);
    const Eigen::Vector3d prediction_delta = predicted - previous.prediction;
    return gap > 0 && gap <= timeout_ns_ &&
        delta.head<2>().norm() <= timing_.recovery_max_speed * dt &&
        std::abs(delta[2]) <= timing_.recovery_max_yaw_rate * dt &&
        (delta.head<2>() - prediction_delta.head<2>()).norm() <= 0.02 &&
        std::abs(std::remainder(delta[2] - prediction_delta[2], kTwoPi)) <= 0.05;
}

bool TimedEKF::try_recovery(std::int64_t stamp_ns, const Eigen::Vector3d& pose,
                           const Eigen::Vector3d& predicted) {
    if (!rejected_.empty() && !consistent_step(stamp_ns, pose, predicted)) {
        clear_recovery();
    }
    rejected_.push_back(Rejected{stamp_ns, pose, predicted});
    if (rejected_.size() > timing_.recovery_samples) { rejected_.pop_front(); }
    if (rejected_.size() < timing_.recovery_samples) { return false; }
    const auto& first = rejected_.front();
    const double span = (stamp_ns - first.stamp) * 1e-9;
    const auto correction = pose_delta(pose, predicted);
    const auto motion_error = pose_delta(pose - first.pose, predicted - first.prediction);
    if (span < 0.2 || correction.head<2>().norm() > timing_.recovery_max_position ||
        std::abs(correction[2]) > timing_.recovery_max_heading ||
        motion_error.head<2>().norm() > 0.05 || std::abs(motion_error[2]) > 0.08) {
        return false;
    }
    const Eigen::Matrix<double, 5, 1> bias = filter_.x.tail<5>();
    const Eigen::Matrix<double, 5, 5> bias_cov = filter_.P.bottomRightCorner<5, 5>();
    filter_.initialize(pose[0], pose[1], pose[2], true);
    filter_.x.tail<5>() = bias;
    filter_.P.bottomRightCorner<5, 5>() = bias_cov;
    filter_.x.segment<2>(PlanarEKF::IVX) = (pose.head<2>() - first.pose.head<2>()) / span;
    filter_.x[PlanarEKF::IOMEGA] = std::remainder(pose[2] - first.pose[2], kTwoPi) / span;
    clear_recovery();
    return true;
}

}  // namespace fss_navigation
