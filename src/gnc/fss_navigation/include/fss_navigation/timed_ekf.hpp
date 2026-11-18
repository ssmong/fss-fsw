#pragma once

#include "fss_navigation/ekf.hpp"

#include <cstdint>
#include <deque>
#include <optional>

namespace fss_navigation {

struct NavigationTiming {
    double delay_ms = 0.0;
    double history_ms = 200.0;
    std::size_t recovery_samples = 25;
    double recovery_max_position = 0.25;
    double recovery_max_heading = 0.35;
    double recovery_max_speed = 1.0;
    double recovery_max_yaw_rate = 2.0;
};

struct TimedUpdate {
    bool accepted = false;
    bool recovered = false;
    bool outside_history = false;
    double nis = 0.0;
    Eigen::Vector3d innovation = Eigen::Vector3d::Zero();
};

// Correct at acquisition time and replay each saved IMU interval to the present.
class TimedEKF {
public:
    TimedEKF(PlanarEKF& filter, const NavigationTiming& timing, std::int64_t timeout_ns);
    void clear();
    void clear_recovery();
    void initialize(std::int64_t stamp_ns, const Eigen::Vector3d& pose);
    void predict(std::int64_t stamp_ns, const Eigen::Vector3d& inputs);
    TimedUpdate update(std::int64_t stamp_ns, const Eigen::Vector3d& pose);
    std::optional<std::int64_t> time_ns() const { return time_ns_; }
    std::size_t history_size() const { return steps_.size(); }

private:
    struct Step {
        std::int64_t start, end;
        Eigen::Vector3d inputs;
        PlanarEKF::State x;
        PlanarEKF::Covariance P;
    };
    struct Rejected {
        std::int64_t stamp;
        Eigen::Vector3d pose, prediction;
    };
    bool consistent_step(std::int64_t stamp_ns, const Eigen::Vector3d& pose,
                         const Eigen::Vector3d& predicted) const;
    bool try_recovery(std::int64_t stamp_ns, const Eigen::Vector3d& pose,
                      const Eigen::Vector3d& predicted);
    PlanarEKF& filter_;
    NavigationTiming timing_;
    std::int64_t history_ns_, timeout_ns_;
    std::optional<std::int64_t> time_ns_, last_measurement_ns_, last_accepted_ns_;
    std::deque<Step> steps_;
    std::deque<Rejected> rejected_;
};

}  // namespace fss_navigation
