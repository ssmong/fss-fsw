#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "fss_manager/mode_fsm.hpp"

using fss_manager::HealthStatus;
using fss_manager::ManagerConfig;
using fss_manager::ManagerCore;
using fss_manager::Mode;
using fss_manager::ModeFSM;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition))                                                      \
            throw std::runtime_error(std::string(__func__) + ": " #condition); \
    } while (false)

namespace {
struct FakeClock {
    std::int64_t now_ns{0};
    void advance_ms(std::int64_t ms) { now_ns += ms * 1000000; }
};

struct Fixture {
    FakeClock clock;
    ManagerCore manager;
    explicit Fixture(ManagerConfig config = {}) : manager(config) {}
    void sensors(bool main_open = true) {
        manager.on_nav(clock.now_ns, 0.001, 0.001, 0.002);
        manager.on_vicon(clock.now_ns);
        manager.on_valve(clock.now_ns, main_open, 1.5);
    }
    void check() { manager.health_check(clock.now_ns); }
    bool request(Mode mode) { return manager.set_mode(static_cast<std::uint8_t>(mode)).success; }
    void active() {
        sensors();
        check();
        CHECK(request(Mode::PRESSURIZING));
        CHECK(request(Mode::READY));
        CHECK(request(Mode::ACTIVE));
    }
};

void all_transition_guards() {
    const bool allowed[6][6] = {
        {false, true, false, false, false, true}, {true, false, true, false, false, true},
        {true, false, false, true, false, true},  {false, false, true, false, true, true},
        {true, false, true, false, false, true},  {true, false, false, false, false, false},
    };
    for (int from = 0; from < 6; ++from) {
        for (int to = 0; to < 6; ++to) {
            ModeFSM fsm;
            fsm.mode = static_cast<Mode>(from);
            CHECK(fsm.can_transition(static_cast<Mode>(to)) == allowed[from][to]);
            CHECK(fsm.transition(static_cast<Mode>(to), "matrix") == allowed[from][to]);
            CHECK(fsm.mode == static_cast<Mode>(allowed[from][to] ? to : from));
            CHECK(fsm.transition_log.size() == (allowed[from][to] ? 1U : 0U));
            CHECK(!fsm.can_transition(static_cast<Mode>(255)));
        }
    }
}

void initial_nominal_and_history() {
    ModeFSM fsm;
    CHECK(fsm.mode == Mode::IDLE);
    CHECK(std::string(fss_manager::mode_name(fsm.mode)) == "IDLE");
    CHECK(fsm.transition(Mode::PRESSURIZING, "pressurize"));
    CHECK(fsm.transition(Mode::READY, "ready"));
    CHECK(fsm.transition(Mode::ACTIVE, "activate"));
    CHECK(fsm.transition(Mode::SAFE_HOLD, "fault"));
    CHECK(fsm.transition(Mode::READY, "reset"));
    CHECK(fsm.transition_log.size() == 5);
    CHECK(fsm.transition_log[0].from == Mode::IDLE);
    CHECK(fsm.transition_log[0].to == Mode::PRESSURIZING);
    CHECK(fsm.transition_log[0].reason == "pressurize");
}

void emergency_latch_and_manual_reset() {
    for (int initial = 0; initial < 6; ++initial) {
        ModeFSM fsm;
        fsm.mode = static_cast<Mode>(initial);
        fsm.emergency_stop("trip");
        CHECK(fsm.mode == Mode::EMERGENCY_STOP);
        CHECK(fsm.transition_log.back().from == static_cast<Mode>(initial));
        CHECK(fsm.transition_log.back().reason == "trip");
        for (int target = 1; target < 6; ++target) {
            CHECK(!fsm.transition(static_cast<Mode>(target)));
        }
        CHECK(fsm.transition(Mode::IDLE, "manual reset"));
        CHECK(fsm.transition(Mode::PRESSURIZING));
    }
}

void health_thresholds_and_message_reset() {
    HealthStatus health;
    health.update(500.0, 15.0, 50.0, true, true);
    CHECK(health.navigation_ok());
    CHECK(!health.has_fault());
    CHECK(health.warnings.empty());
    CHECK(health.main_valve_open);
    health.update(500.001, 14.99, 50.01, false, true);
    CHECK(health.warnings.size() == 3);
    CHECK(!health.has_fault());
    health.update(1000.0, 20.0);
    CHECK(health.navigation_ok());
    CHECK(!health.has_fault());
    CHECK(health.warnings.size() == 1);
    health.update(1000.001, 20.0, 0.0, false, false);
    CHECK(!health.navigation_ok());
    CHECK(health.has_fault());
    CHECK(health.errors.size() == 2);
    CHECK(health.warnings.empty());
    health.update();
    CHECK(health.warnings.empty());
    CHECK(health.errors.empty());
    CHECK(health.vicon_connected);
}

void unreceived_data_never_ready() {
    Fixture f;
    CHECK(!f.request(Mode::PRESSURIZING));
    f.check();
    CHECK(!f.manager.health().navigation_ok());
    CHECK(f.manager.health().navigation_age_ms == 9999.0);
    CHECK(f.manager.health().has_fault());
    CHECK(f.manager.navigation_degraded());
    CHECK(!f.manager.actuators_ready());
    CHECK(!f.manager.nav_ready());
    CHECK(!f.manager.vicon_ready());
    CHECK(!f.request(Mode::PRESSURIZING));
}

void first_health_tick_required_and_zero_timestamp_valid() {
    Fixture f;
    f.sensors();
    CHECK(!f.request(Mode::PRESSURIZING));
    f.check();
    CHECK(f.manager.nav_ready());
    CHECK(f.manager.vicon_ready());
    CHECK(f.manager.actuators_ready());
    CHECK(f.request(Mode::PRESSURIZING));
}

void readiness_gates_and_error_order() {
    Fixture f;
    f.manager.on_valve(f.clock.now_ns, false, 0.0);
    f.check();
    CHECK(f.request(Mode::PRESSURIZING));
    auto response = f.manager.set_mode(2);
    CHECK(!response.success);
    CHECK(response.message.find("메인밸브") != std::string::npos);
    f.manager.on_valve(f.clock.now_ns, true, 0.0);
    f.check();
    CHECK(f.request(Mode::READY));
    response = f.manager.set_mode(3);
    CHECK(!response.success);
    CHECK(response.message.find("항법") != std::string::npos);
    f.manager.on_nav(f.clock.now_ns, 0.001, 0.001, 0.002);
    f.check();
    response = f.manager.set_mode(3);
    CHECK(!response.success);
    CHECK(response.message.find("Vicon") != std::string::npos);
    f.manager.on_vicon(f.clock.now_ns);
    f.manager.on_valve(f.clock.now_ns, false, 0.0);
    f.check();
    response = f.manager.set_mode(3);
    CHECK(!response.success);
    CHECK(response.message.find("메인밸브") != std::string::npos);
    f.manager.on_valve(f.clock.now_ns, true, 0.0);
    CHECK(f.request(Mode::ACTIVE));
}

void valve_and_vicon_freshness_boundaries() {
    Fixture f;
    f.sensors();
    f.clock.advance_ms(499);
    f.check();
    CHECK(f.manager.actuators_ready());
    CHECK(f.manager.vicon_ready());
    CHECK(f.manager.nav_ready());
    f.clock.advance_ms(1);
    f.check();
    CHECK(!f.manager.actuators_ready());
    CHECK(!f.manager.vicon_ready());
    CHECK(!f.request(Mode::PRESSURIZING));
    f.sensors();
    f.check();
    CHECK(f.manager.actuators_ready());
    CHECK(f.manager.vicon_ready());
}

void navigation_age_and_uncertainty_boundaries() {
    Fixture f;
    f.manager.on_nav(0, 0.005, 0.005, 0.02);
    f.clock.advance_ms(1000);
    f.check();
    CHECK(f.manager.health().navigation_ok());
    CHECK(!f.manager.nav_ready());
    CHECK(!f.manager.navigation_degraded());
    ++f.clock.now_ns;
    f.check();
    CHECK(!f.manager.nav_ready());
    CHECK(f.manager.navigation_degraded());
    f.manager.on_nav(f.clock.now_ns, 0.00501, 0.001, 0.002);
    f.check();
    CHECK(f.manager.health().navigation_ok());
    CHECK(!f.manager.nav_ready());
    CHECK(f.manager.navigation_degraded());
    CHECK(f.manager.health().has_fault());
    f.manager.on_nav(f.clock.now_ns, 0.001, 0.00501, 0.002);
    f.check();
    CHECK(f.manager.navigation_degraded());
    f.manager.on_nav(f.clock.now_ns, 0.001, 0.001, 0.02001);
    f.check();
    CHECK(f.manager.navigation_degraded());
    f.manager.on_nav(f.clock.now_ns, std::numeric_limits<double>::quiet_NaN(), 0.001, 0.002);
    f.check();
    CHECK(!f.manager.nav_ready());
    f.manager.on_nav(f.clock.now_ns, 0.001, 0.001, 0.002);
    f.check();
    CHECK(f.manager.nav_ready());
    CHECK(!f.manager.health().has_fault());
}

void nav_count_measurement_resets_each_second() {
    Fixture f;
    for (int i = 0; i < 20; ++i)
        f.manager.on_nav(0, 0.001, 0.001, 0.002);
    f.manager.measure_nav_hz();
    f.check();
    CHECK(f.manager.health().control_loop_rate_hz == 20.0);
    CHECK(f.manager.health().warnings.empty());
    f.manager.measure_nav_hz();
    f.check();
    CHECK(f.manager.health().control_loop_rate_hz == 0.0);
    CHECK(f.manager.health().warnings.size() == 1);
}

void active_fault_holds_once_and_requires_rearming() {
    Fixture f;
    f.active();
    f.clock.advance_ms(1000);
    CHECK(!f.manager.health_check(f.clock.now_ns));
    CHECK(f.manager.mode() == Mode::ACTIVE);
    ++f.clock.now_ns;
    CHECK(f.manager.health_check(f.clock.now_ns));
    CHECK(f.manager.mode() == Mode::SAFE_HOLD);
    CHECK(f.manager.transition_log().back().reason.find("항법 데이터 지연") != std::string::npos);
    CHECK(!f.manager.health_check(f.clock.now_ns));
    CHECK(!f.request(Mode::ACTIVE));
    f.sensors();
    f.check();
    CHECK(f.manager.mode() == Mode::SAFE_HOLD);
    CHECK(!f.request(Mode::ACTIVE));
    CHECK(f.request(Mode::READY));
    CHECK(f.request(Mode::ACTIVE));
    f.manager.on_nav(f.clock.now_ns, 0.1, 0.001, 0.002);
    CHECK(f.manager.health_check(f.clock.now_ns));
    CHECK(f.manager.mode() == Mode::SAFE_HOLD);
}

void short_navigation_gap_recovers_without_latching() {
    Fixture f;
    f.active();
    f.clock.advance_ms(101);
    CHECK(!f.manager.health_check(f.clock.now_ns));
    CHECK(f.manager.mode() == Mode::ACTIVE);
    f.clock.advance_ms(799);
    CHECK(!f.manager.health_check(f.clock.now_ns));
    CHECK(f.manager.mode() == Mode::ACTIVE);
    CHECK(f.manager.health().navigation_ok());
    f.sensors();
    f.check();
    CHECK(f.manager.mode() == Mode::ACTIVE);
    CHECK(!f.manager.health().has_fault());
    CHECK(f.manager.nav_ready());
}

void auto_hold_can_be_disabled() {
    ManagerConfig config;
    config.auto_safe_hold = false;
    Fixture f(config);
    f.active();
    f.clock.advance_ms(1001);
    CHECK(!f.manager.health_check(f.clock.now_ns));
    CHECK(f.manager.health().has_fault());
    CHECK(f.manager.mode() == Mode::ACTIVE);
}

void safety_transitions_ignore_readiness() {
    Fixture f;
    CHECK(f.request(Mode::EMERGENCY_STOP));
    f.clock.advance_ms(10000);
    f.check();
    CHECK(f.manager.mode() == Mode::EMERGENCY_STOP);
    CHECK(!f.request(Mode::READY));
    CHECK(!f.request(Mode::ACTIVE));
    CHECK(f.request(Mode::IDLE));
    CHECK(!f.request(Mode::PRESSURIZING));
    Fixture active;
    active.active();
    active.clock.advance_ms(1001);
    active.check();
    CHECK(active.request(Mode::IDLE));
    Fixture manual_hold;
    manual_hold.active();
    CHECK(manual_hold.request(Mode::SAFE_HOLD));
    CHECK(manual_hold.request(Mode::EMERGENCY_STOP));
    CHECK(manual_hold.request(Mode::IDLE));
}

void invalid_wire_values_never_change_state() {
    Fixture f;
    f.active();
    const auto log_size = f.manager.transition_log().size();
    for (int value = 6; value < 256; ++value) {
        auto response = f.manager.set_mode(static_cast<std::uint8_t>(value));
        CHECK(!response.success);
        CHECK(response.actual_mode == Mode::ACTIVE);
        CHECK(response.message.find(std::to_string(value)) != std::string::npos);
    }
    CHECK(f.manager.transition_log().size() == log_size);
}

void inherited_vicon_uncertainty_fault_semantics() {
    Fixture f;
    f.active();
    f.clock.advance_ms(501);
    f.manager.on_nav(f.clock.now_ns, 0.001, 0.001, 0.002);
    f.check();
    CHECK(!f.manager.vicon_ready());
    CHECK(!f.manager.actuators_ready());
    CHECK(f.manager.health().vicon_connected);
    CHECK(!f.manager.health().has_fault());
    CHECK(f.manager.mode() == Mode::ACTIVE);
}
}  // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests = {
        {"all_transition_guards", all_transition_guards},
        {"initial_nominal_and_history", initial_nominal_and_history},
        {"emergency_latch_and_manual_reset", emergency_latch_and_manual_reset},
        {"health_thresholds_and_message_reset", health_thresholds_and_message_reset},
        {"unreceived_data_never_ready", unreceived_data_never_ready},
        {"first_health_tick_required_and_zero_timestamp_valid",
         first_health_tick_required_and_zero_timestamp_valid},
        {"readiness_gates_and_error_order", readiness_gates_and_error_order},
        {"valve_and_vicon_freshness_boundaries", valve_and_vicon_freshness_boundaries},
        {"navigation_age_and_uncertainty_boundaries", navigation_age_and_uncertainty_boundaries},
        {"nav_count_measurement_resets_each_second", nav_count_measurement_resets_each_second},
        {"active_fault_holds_once_and_requires_rearming",
         active_fault_holds_once_and_requires_rearming},
        {"short_navigation_gap_recovers_without_latching",
         short_navigation_gap_recovers_without_latching},
        {"auto_hold_can_be_disabled", auto_hold_can_be_disabled},
        {"safety_transitions_ignore_readiness", safety_transitions_ignore_readiness},
        {"invalid_wire_values_never_change_state", invalid_wire_values_never_change_state},
        {"inherited_vicon_uncertainty_fault_semantics",
         inherited_vicon_uncertainty_fault_semantics},
    };
    for (const auto& test : tests) {
        try {
            test.second();
            std::cout << "PASS " << test.first << '\n';
        } catch (const std::exception& error) {
            std::cerr << "FAIL " << test.first << ": " << error.what() << '\n';
            return 1;
        }
    }
    std::cout << tests.size() << " manager tests passed\n";
    return 0;
}
