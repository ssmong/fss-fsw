#pragma once

#include "fss_navigation/ekf.hpp"
#include "fss_navigation/timed_ekf.hpp"

#include <cstdint>
#include <optional>

namespace fss_navigation {

enum class InputResult { Accepted, Invalid, Stale, Reordered, Rejected, Resynchronizing };

struct NavigationState {
    std::int64_t stamp_ns;
    PlanarEKF::State x;
    double std_x;
    double std_y;
    double std_theta;
};

// Sensor timestamps drive integration; receipt time only checks freshness.
class Navigation {
public:
    Navigation(const EKFParameters& parameters, double timeout_ms, std::int64_t max_prediction_cycles,
               const NavigationTiming& timing = {});
    InputResult on_imu(std::int64_t now_ns, std::int64_t stamp_ns, double omega, double ax,
                       double ay, bool gyro_available = true, bool accel_available = true);
    InputResult on_vicon(std::int64_t now_ns, std::int64_t stamp_ns, double px, double py,
                         double theta);
    std::optional<NavigationState> state(std::int64_t now_ns);

    const PlanarEKF& filter() const { return filter_; }
    bool needs_vicon_initialization() const { return needs_vicon_initialization_; }
    bool navigation_degraded() const { return navigation_degraded_; }
    void clear_recovery() { fusion_.clear_recovery(); }
    const TimedUpdate& last_update() const { return last_update_; }
    std::uint64_t update_count() const { return update_count_; }

private:
    bool fresh(std::int64_t now_ns, std::int64_t stamp_ns) const;
    void drain_vicon(std::int64_t now_ns);
    void record_vicon(std::int64_t now_ns, std::int64_t stamp_ns, const TimedUpdate& result);
    void suspend();
    PlanarEKF filter_;
    std::int64_t timeout_ns_;
    std::int64_t max_prediction_cycles_;
    std::int64_t delay_ns_;
    TimedEKF fusion_;
    struct PendingVicon {
        std::int64_t measurement, stamp;
        Eigen::Vector3d pose;
    };
    std::deque<PendingVicon> pending_vicon_;
    std::optional<std::int64_t> last_vicon_seen_ns_;
    TimedUpdate last_update_;
    std::uint64_t update_count_ = 0;
    std::optional<std::int64_t> last_imu_time_ns_;
    std::optional<std::int64_t> last_prediction_time_ns_;
    std::optional<std::int64_t> last_vicon_time_ns_;
    std::optional<std::int64_t> last_vicon_stamp_ns_;
    std::int64_t prediction_only_count_ = 0;
    bool needs_vicon_initialization_ = false;
    bool navigation_degraded_ = false;
};

}  // namespace fss_navigation
