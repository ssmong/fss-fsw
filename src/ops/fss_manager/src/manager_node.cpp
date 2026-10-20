#include "fss_manager/manager_node.hpp"

#include <chrono>
#include <cmath>
#include <rclcpp/create_timer.hpp>
#include <stdexcept>

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
    return config;
}

ManagerNode::ManagerNode(const rclcpp::NodeOptions& options)
    : Node("manager_node", options), core_(declare_config()) {
    nav_sub_ = create_subscription<fss_interfaces::msg::SpacecraftState>(
        "/navigation/state", 10, [this](const fss_interfaces::msg::SpacecraftState::SharedPtr msg) {
            core_.on_nav(now().nanoseconds(), msg->std_x, msg->std_y, msg->std_theta);
        });
    vicon_sub_ = create_subscription<fss_interfaces::msg::SpacecraftState>(
        "/sensors/vicon_state", 10, [this](const fss_interfaces::msg::SpacecraftState::SharedPtr) {
            core_.on_vicon(now().nanoseconds());
        });
    valve_sub_ = create_subscription<fss_interfaces::msg::ValveStatus>(
        "/actuators/valve_status", 10,
        [this](const fss_interfaces::msg::ValveStatus::SharedPtr msg) {
            core_.on_valve(now().nanoseconds(), msg->main_valve_open, msg->modbus_rtt_ms);
        });
    mode_pub_ = create_publisher<fss_interfaces::msg::SystemMode>("/system/mode", 10);
    health_pub_ = create_publisher<fss_interfaces::msg::SystemHealth>("/system/health", 10);
    mode_service_ = create_service<fss_interfaces::srv::SetSystemMode>(
        "/system/set_mode",
        [this](const std::shared_ptr<fss_interfaces::srv::SetSystemMode::Request> request,
               std::shared_ptr<fss_interfaces::srv::SetSystemMode::Response> response) {
            const auto result = core_.set_mode(request->requested_mode);
            response->success = result.success;
            response->actual_mode = static_cast<std::uint8_t>(result.actual_mode);
            response->message = result.message;
            if (result.success) {
                publish_mode();
                RCLCPP_INFO(get_logger(), "%s", result.message.c_str());
            } else {
                RCLCPP_WARN(get_logger(), "%s", result.message.c_str());
            }
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
}

}  // namespace fss_manager
