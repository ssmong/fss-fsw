#include "fss_manager/mode_fsm.hpp"

#include <iomanip>
#include <sstream>

namespace fss_manager {
namespace {
std::string fixed(double value, int precision) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}
}  // namespace

const char* mode_name(Mode mode) {
    switch (mode) {
        case Mode::IDLE:
            return "IDLE";
        case Mode::PRESSURIZING:
            return "PRESSURIZING";
        case Mode::READY:
            return "READY";
        case Mode::ACTIVE:
            return "ACTIVE";
        case Mode::SAFE_HOLD:
            return "SAFE_HOLD";
        case Mode::EMERGENCY_STOP:
            return "EMERGENCY_STOP";
    }
    return "UNKNOWN";
}

bool ModeFSM::can_transition(Mode target) const {
    switch (mode) {
        case Mode::IDLE:
            return target == Mode::PRESSURIZING || target == Mode::EMERGENCY_STOP;
        case Mode::PRESSURIZING:
            return target == Mode::READY || target == Mode::IDLE || target == Mode::EMERGENCY_STOP;
        case Mode::READY:
            return target == Mode::ACTIVE || target == Mode::IDLE || target == Mode::EMERGENCY_STOP;
        case Mode::ACTIVE:
            return target == Mode::SAFE_HOLD || target == Mode::READY ||
                   target == Mode::EMERGENCY_STOP;
        case Mode::SAFE_HOLD:
            return target == Mode::READY || target == Mode::IDLE || target == Mode::EMERGENCY_STOP;
        case Mode::EMERGENCY_STOP:
            return target == Mode::IDLE;
    }
    return false;
}

bool ModeFSM::transition(Mode target, const std::string& reason) {
    if (!can_transition(target))
        return false;
    transition_log.push_back({mode, target, reason});
    mode = target;
    return true;
}

void ModeFSM::emergency_stop(const std::string& reason) {
    transition_log.push_back({mode, Mode::EMERGENCY_STOP, reason});
    mode = Mode::EMERGENCY_STOP;
}

HealthStatus::HealthStatus(double nav_age_limit_ms, double control_rate_min_hz)
    : nav_age_limit_ms_(nav_age_limit_ms), control_rate_min_hz_(control_rate_min_hz) {}

void HealthStatus::update(double nav_age_ms, double ctrl_rate_hz, double rtt_ms, bool main_valve,
                          bool vicon_ok) {
    navigation_age_ms = nav_age_ms;
    control_loop_rate_hz = ctrl_rate_hz;
    modbus_rtt_ms = rtt_ms;
    main_valve_open = main_valve;
    vicon_connected = vicon_ok;
    warnings.clear();
    errors.clear();

    if (nav_age_ms > nav_age_limit_ms_) {
        errors.push_back("항법 데이터 지연: " + fixed(nav_age_ms, 0) + "ms");
    } else if (nav_age_ms > nav_age_limit_ms_ * 0.5) {
        warnings.push_back("항법 데이터 경고: " + fixed(nav_age_ms, 0) + "ms");
    }
    if (ctrl_rate_hz < control_rate_min_hz_) {
        warnings.push_back("제어 루프 저하: " + fixed(ctrl_rate_hz, 1) + "Hz");
    }
    if (rtt_ms > 50.0)
        warnings.push_back("Modbus RTT 높음: " + fixed(rtt_ms, 1) + "ms");
    if (!vicon_ok)
        errors.push_back("항법 보정 상실 (Vicon 미수신 또는 불확실도 한계 초과)");
}

bool HealthStatus::navigation_ok() const {
    return navigation_age_ms <= nav_age_limit_ms_;
}

ManagerCore::ManagerCore(const ManagerConfig& config)
    : config_(config), health_(config.nav_age_limit_ms, config.control_rate_min_hz) {}

void ManagerCore::on_nav(std::int64_t now_ns, double std_x, double std_y, double std_theta) {
    last_nav_ns_ = now_ns;
    ++nav_count_;
    nav_std_ = {std_x, std_y, std_theta};
}

void ManagerCore::on_vicon(std::int64_t now_ns) {
    last_vicon_ns_ = now_ns;
}

void ManagerCore::on_valve(std::int64_t now_ns, bool main_valve_open, double modbus_rtt_ms) {
    last_valve_ns_ = now_ns;
    main_valve_open_ = main_valve_open;
    modbus_rtt_ms_ = modbus_rtt_ms;
}

void ManagerCore::measure_nav_hz() {
    nav_hz_ = static_cast<double>(nav_count_);
    nav_count_ = 0;
}

double ManagerCore::age_ms(std::int64_t now_ns, std::int64_t then_ns) {
    return static_cast<double>((static_cast<long double>(now_ns) - then_ns) * 1e-6L);
}

bool ManagerCore::fresh(std::int64_t now_ns, const std::optional<std::int64_t>& then_ns) {
    return then_ns && age_ms(now_ns, *then_ns) < 500.0;
}

bool ManagerCore::health_check(std::int64_t now_ns) {
    const double nav_age_ms = last_nav_ns_ ? age_ms(now_ns, *last_nav_ns_) : 9999.0;
    nav_uncertainty_ok_ = nav_std_[0] <= config_.nav_pos_std_limit &&
                          nav_std_[1] <= config_.nav_pos_std_limit &&
                          nav_std_[2] <= config_.nav_heading_std_limit;
    const bool vicon_ok = last_nav_ns_.has_value() && nav_uncertainty_ok_;
    health_.update(nav_age_ms, nav_hz_, modbus_rtt_ms_, main_valve_open_, vicon_ok);
    act_ready_ = fresh(now_ns, last_valve_ns_);
    vicon_ready_ = fresh(now_ns, last_vicon_ns_);
    nav_ready_ = fresh(now_ns, last_nav_ns_) && nav_uncertainty_ok_ && health_.navigation_ok();

    if (config_.auto_safe_hold && fsm_.mode == Mode::ACTIVE && health_.has_fault()) {
        std::string reason;
        for (const auto& error : health_.errors) {
            if (!reason.empty())
                reason += "; ";
            reason += error;
        }
        fsm_.transition(Mode::SAFE_HOLD, reason);
        return true;
    }
    return false;
}

std::string ManagerCore::arming_failure(Mode target) const {
    if (target == Mode::PRESSURIZING) {
        if (!act_ready_)
            return "액추에이터(PLC) 미수신 — 메인밸브 명령 불가";
    } else if (target == Mode::READY) {
        if (!act_ready_)
            return "액추에이터(PLC) 미수신";
        if (!main_valve_open_)
            return "에어베어링(메인밸브) 미개방";
    } else if (target == Mode::ACTIVE) {
        if (!nav_ready_)
            return "항법 미수렴/미수신 (Vicon·EKF 확인)";
        if (!vicon_ready_)
            return "Vicon 미수신";
        if (!main_valve_open_)
            return "에어베어링(메인밸브) 미개방";
    }
    return "";
}

ModeResponse ManagerCore::set_mode(std::uint8_t requested_mode) {
    if (requested_mode > static_cast<std::uint8_t>(Mode::EMERGENCY_STOP)) {
        return {false, fsm_.mode, "알 수 없는 모드 값: " + std::to_string(requested_mode)};
    }
    const auto target = static_cast<Mode>(requested_mode);
    const auto reason = arming_failure(target);
    if (!reason.empty())
        return {false, fsm_.mode, "준비 미충족 — " + reason};
    if (!fsm_.transition(target, "서비스 요청")) {
        return {false, fsm_.mode,
                std::string("전이 불가: ") + mode_name(fsm_.mode) + " → " + mode_name(target)};
    }
    return {true, fsm_.mode, std::string("모드 전이: ") + mode_name(fsm_.mode)};
}

}  // namespace fss_manager
