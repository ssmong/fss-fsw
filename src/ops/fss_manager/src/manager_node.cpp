#include "fss_manager/manager_node.hpp"

#include <chrono>
#include <cmath>
#include <rclcpp/create_timer.hpp>
#include <stdexcept>
#include <nlohmann/json.hpp>

namespace fss_manager {

ManagerConfig ManagerNode::declare_config() {
    const auto required_double = [this](const char* name, bool allow_zero = false) {
        declare_parameter(name, rclcpp::ParameterType::PARAMETER_DOUBLE);
        const double value = get_parameter(name).as_double();
        if (!std::isfinite(value) || (allow_zero ? value < 0.0 : value <= 0.0)) {
            throw std::invalid_argument(std::string(name) + " must be finite and " +
                                        (allow_zero ? "nonnegative" : "positive"));
        }
        return value;
    };
    health_check_rate_hz_ = required_double("health_check_rate_hz");
    ManagerConfig config;
    config.nav_age_limit_ms = required_double("nav_age_limit_ms");
    config.control_rate_min_hz = required_double("control_rate_min_hz", true);
    declare_parameter("auto_safe_hold", rclcpp::ParameterType::PARAMETER_BOOL);
    config.auto_safe_hold = get_parameter("auto_safe_hold").as_bool();
    config.nav_pos_std_limit = required_double("nav_pos_std_limit");
    config.nav_heading_std_limit = required_double("nav_heading_std_limit");
    rcl_interfaces::msg::ParameterDescriptor immutable;
    immutable.read_only = true;
    config.nav_correction_loss_timeout_s = declare_parameter("nav_correction_loss_timeout_s", 10.0, immutable);
    config.dry_bench_solenoid = declare_parameter("dry_bench_solenoid", false, immutable);
    config.require_arm_status = declare_parameter("require_arm_status", false, immutable);
    config.toss_nav_uncertainty_grace = declare_parameter("toss_nav_uncertainty_grace", false, immutable);
    return config;
}

ManagerNode::ManagerNode(const rclcpp::NodeOptions& options)
    : Node("manager_node", options), core_(declare_config()) {
    dry_bench_solenoid_ = get_parameter("dry_bench_solenoid").as_bool();
    control_rate_min_hz_ = get_parameter("control_rate_min_hz").as_double();
    auto_activate_ = declare_parameter("auto_activate", false);
    auto_activate_pending_ = auto_activate_;
    auto_open_pending_ = declare_parameter("auto_open_main_valve", false) || auto_activate_;
    auto_open_deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    auto_activate_deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    main_valve_client_ = create_client<fss_interfaces::srv::MainValveControl>("/actuators/main_valve");
    if (get_parameter("require_arm_status").as_bool()) {
        arm_sub_ = create_subscription<std_msgs::msg::String>("/arm/operation_status", rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::String& msg) {
                try {
                    const auto data = nlohmann::json::parse(msg.data);
                    if (!data.is_object() || !data.contains("api_version") || data["api_version"] != 1 ||
                        !data.contains("status_stamp") || !data["status_stamp"].is_number()) return;
                    const double stamp = data["status_stamp"].get<double>();
                    if (std::isfinite(stamp) && stamp >= 0.0 && stamp < 9.22e9)
                        core_.on_arm_status(static_cast<std::int64_t>(stamp * 1e9));
                } catch (const nlohmann::json::exception&) { }
            });
    }
    nav_sub_ = create_subscription<fss_interfaces::msg::SpacecraftState>(
        "/navigation/state", 10, [this](const fss_interfaces::msg::SpacecraftState::SharedPtr msg) {
            core_.on_nav(now().nanoseconds(), msg->std_x, msg->std_y, msg->std_theta,
                static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL + msg->header.stamp.nanosec);
        });
    vicon_sub_ = create_subscription<fss_interfaces::msg::SpacecraftState>(
        "/sensors/vicon_state", 10, [this](const fss_interfaces::msg::SpacecraftState::SharedPtr msg) {
            core_.on_vicon(now().nanoseconds(),
                static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL + msg->header.stamp.nanosec,
                std::isfinite(msg->x) && std::isfinite(msg->y) && std::isfinite(msg->theta));
        });
    valve_sub_ = create_subscription<fss_interfaces::msg::ValveStatus>(
        dry_bench_solenoid_ ? "/actuators/dry_bench/valve_status" : "/actuators/valve_status", 10,
        [this](const fss_interfaces::msg::ValveStatus::SharedPtr msg) {
            core_.on_valve(now().nanoseconds(), msg->main_valve_open, msg->modbus_rtt_ms);
        });
    mode_pub_ = create_publisher<fss_interfaces::msg::SystemMode>("/system/mode", 10);
    health_pub_ = create_publisher<fss_interfaces::msg::SystemHealth>("/system/health", 10);
    mode_service_ = create_service<fss_interfaces::srv::SetSystemMode>(
        "/system/set_mode",
        [this](const std::shared_ptr<fss_interfaces::srv::SetSystemMode::Request> request,
               std::shared_ptr<fss_interfaces::srv::SetSystemMode::Response> response) {
            auto_activate_pending_ = auto_open_pending_ = false;
            const auto result = set_mode(request->requested_mode);
            response->success = result.success;
            response->actual_mode = static_cast<std::uint8_t>(result.actual_mode);
            response->message = result.message;
        });
    health_timer_ = rclcpp::create_timer(this, get_clock(),
                                         std::chrono::duration<double>(1.0 / health_check_rate_hz_),
                                         [this]() { health_check(); });
    rate_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::seconds(1),
                                       [this]() { core_.measure_nav_hz(); });
    RCLCPP_INFO(get_logger(), "시스템 관리자 시작 (모드: %s)", mode_name(core_.mode()));
}

void ManagerNode::publish_mode() {
    fss_interfaces::msg::SystemMode msg;
    msg.header.stamp = now();
    msg.mode = static_cast<std::uint8_t>(core_.mode());
    msg.mode_name = mode_name(core_.mode());
    mode_pub_->publish(msg);
}

void ManagerNode::health_check() {
    if (core_.health_check(now().nanoseconds())) {
        RCLCPP_ERROR(get_logger(), "자동 SAFE_HOLD: %s",
                     core_.transition_log().back().reason.c_str());
        if (get_parameter("auto_activate").as_bool()) {
            auto_activate_pending_ = true;
            auto_expected_mode_ = Mode::SAFE_HOLD;
            auto_ready_since_.reset();
            auto_activate_deadline_.reset();
        }
    }
    publish_mode();
    const auto& health = core_.health();
    fss_interfaces::msg::SystemHealth msg;
    msg.header.stamp = now();
    msg.navigation_ok = health.navigation_ok();
    msg.actuators_ok = core_.actuators_ready();
    msg.main_valve_open = health.main_valve_open;
    msg.navigation_degraded = core_.navigation_degraded();
    msg.control_loop_rate_hz = health.control_loop_rate_hz;
    msg.navigation_age_sec = health.navigation_age_ms * 1e-3;
    msg.modbus_rtt_ms = health.modbus_rtt_ms;
    msg.warnings = health.warnings;
    msg.errors = health.errors;
    health_pub_->publish(msg);
    auto_open_main_valve();
    auto_advance_active();
}

ModeResponse ManagerNode::set_mode(std::uint8_t requested_mode) {
    const auto result = core_.set_mode(requested_mode, now().nanoseconds());
    if (result.success) {
        publish_mode();
        RCLCPP_INFO(get_logger(), "%s", result.message.c_str());
    } else {
        RCLCPP_WARN(get_logger(), "%s", result.message.c_str());
    }
    return result;
}

void ManagerNode::auto_open_main_valve() {
    if (!auto_open_pending_) return;
    if (core_.mode() != (auto_open_future_ ? Mode::PRESSURIZING : Mode::IDLE)) {
        auto_open_pending_ = false;
        RCLCPP_WARN(get_logger(), "메인밸브 자동 개방 중단 — 시스템 모드 변경");
        return;
    }
    if (auto_open_future_ && auto_open_future_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        auto_open_pending_ = false;
        try {
            const auto response = auto_open_future_->get();
            auto_open_succeeded_ = response->success;
            if (response->success) RCLCPP_INFO(get_logger(), "메인밸브 기동 설정 완료");
            else RCLCPP_ERROR(get_logger(), "메인밸브 자동 개방 실패: %s", response->message.c_str());
        } catch (const std::exception& error) {
            RCLCPP_ERROR(get_logger(), "메인밸브 자동 개방 서비스 오류: %s", error.what());
        }
        return;
    }
    if (std::chrono::steady_clock::now() >= auto_open_deadline_) {
        auto_open_pending_ = false;
        main_valve_client_->prune_pending_requests();
        RCLCPP_ERROR(get_logger(), "메인밸브 자동 개방 30초 시간 초과");
        return;
    }
    if (auto_open_future_ || !core_.arming_failure(Mode::PRESSURIZING).empty() ||
        !main_valve_client_->service_is_ready()) return;
    if (!set_mode(static_cast<std::uint8_t>(Mode::PRESSURIZING)).success) {
        auto_open_pending_ = false;
        return;
    }
    auto request = std::make_shared<fss_interfaces::srv::MainValveControl::Request>();
    request->open = !dry_bench_solenoid_;
    try {
        auto_open_future_ = main_valve_client_->async_send_request(request).future.share();
    } catch (const std::exception& error) {
        auto_open_pending_ = false;
        RCLCPP_ERROR(get_logger(), "메인밸브 자동 개방 요청 오류: %s", error.what());
    }
}

void ManagerNode::auto_advance_active() {
    if (!auto_activate_pending_) return;
    const auto tick = std::chrono::steady_clock::now();
    if (auto_activate_deadline_ && tick >= *auto_activate_deadline_) {
        auto_activate_pending_ = false;
        RCLCPP_ERROR(get_logger(), "자동 ACTIVE 진입 60초 시간 초과");
        return;
    }
    if (auto_open_pending_) return;
    if ((auto_activate_deadline_ && !auto_open_succeeded_) || core_.mode() != auto_expected_mode_) {
        auto_activate_pending_ = false;
        RCLCPP_WARN(get_logger(), "자동 ACTIVE 진입 중단 — 밸브 설정 또는 시스템 모드 확인 필요");
        return;
    }
    if (!core_.arming_failure(Mode::ACTIVE, now().nanoseconds()).empty() || !core_.actuators_ready() ||
        core_.health().has_fault() || core_.health().control_loop_rate_hz < control_rate_min_hz_) {
        auto_ready_since_.reset();
        return;
    }
    if (!auto_ready_since_) {
        auto_ready_since_ = tick;
        return;
    }
    if (tick - *auto_ready_since_ < std::chrono::seconds(1)) return;
    const auto target = core_.mode() == Mode::READY ? Mode::ACTIVE : Mode::READY;
    if (!set_mode(static_cast<std::uint8_t>(target)).success) {
        auto_activate_pending_ = false;
        return;
    }
    auto_ready_since_ = tick;
    auto_expected_mode_ = target;
    if (target == Mode::ACTIVE) auto_activate_pending_ = false;
}

}  // namespace fss_manager
