#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace fss_manager {

enum class Mode : std::uint8_t {
    IDLE = 0,
    PRESSURIZING = 1,
    READY = 2,
    ACTIVE = 3,
    SAFE_HOLD = 4,
    EMERGENCY_STOP = 5,
};

const char* mode_name(Mode mode);

struct Transition {
    Mode from;
    Mode to;
    std::string reason;
};

class ModeFSM {
public:
    Mode mode{Mode::IDLE};
    std::vector<Transition> transition_log;

    bool can_transition(Mode target) const;
    bool transition(Mode target, const std::string& reason = "");
    void emergency_stop(const std::string& reason = "긴급 정지");
};

class HealthStatus {
public:
    explicit HealthStatus(double nav_age_limit_ms = 1000.0, double control_rate_min_hz = 15.0);
    void update(double nav_age_ms = 0.0, double ctrl_rate_hz = 20.0, double modbus_rtt_ms = 0.0,
                bool main_valve = false, bool vicon_ok = true);
    bool navigation_ok() const;
    bool has_fault() const { return !errors.empty(); }

    double navigation_age_ms{0.0};
    double control_loop_rate_hz{0.0};
    double modbus_rtt_ms{0.0};
    bool main_valve_open{false};
    bool vicon_connected{false};
    std::vector<std::string> warnings;
    std::vector<std::string> errors;

private:
    double nav_age_limit_ms_;
    double control_rate_min_hz_;
};

struct ManagerConfig {
    double nav_age_limit_ms{1000.0};
    double control_rate_min_hz{15.0};
    bool auto_safe_hold{true};
    double nav_pos_std_limit{0.005};
    double nav_heading_std_limit{0.02};
};

struct ModeResponse {
    bool success;
    Mode actual_mode;
    std::string message;
};

// Receipt times come from the ROS adapter; tests advance them without sleeping.
class ManagerCore {
public:
    explicit ManagerCore(const ManagerConfig& config);
    void on_nav(std::int64_t now_ns, double std_x, double std_y, double std_theta);
    void on_vicon(std::int64_t now_ns);
    void on_valve(std::int64_t now_ns, bool main_valve_open, double modbus_rtt_ms);
    void measure_nav_hz();
    bool health_check(std::int64_t now_ns);
    ModeResponse set_mode(std::uint8_t requested_mode);

    Mode mode() const { return fsm_.mode; }
    const HealthStatus& health() const { return health_; }
    bool navigation_degraded() const { return !nav_uncertainty_ok_ || !health_.navigation_ok(); }
    bool actuators_ready() const { return act_ready_; }
    bool nav_ready() const { return nav_ready_; }
    bool vicon_ready() const { return vicon_ready_; }
    const std::vector<Transition>& transition_log() const { return fsm_.transition_log; }

private:
    std::string arming_failure(Mode target) const;
    static double age_ms(std::int64_t now_ns, std::int64_t then_ns);
    static bool fresh(std::int64_t now_ns, const std::optional<std::int64_t>& then_ns);

    ManagerConfig config_;
    ModeFSM fsm_;
    HealthStatus health_;
    std::optional<std::int64_t> last_nav_ns_;
    std::optional<std::int64_t> last_vicon_ns_;
    std::optional<std::int64_t> last_valve_ns_;
    double nav_hz_{0.0};
    std::uint64_t nav_count_{0};
    std::array<double, 3> nav_std_{};
    bool main_valve_open_{false};
    double modbus_rtt_ms_{0.0};
    bool nav_ready_{false};
    bool vicon_ready_{false};
    bool act_ready_{false};
    bool nav_uncertainty_ok_{true};
};

}  // namespace fss_manager
