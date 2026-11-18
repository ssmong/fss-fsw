#pragma once

#include <fss_interfaces/msg/spacecraft_state.hpp>
#include <fss_interfaces/msg/system_health.hpp>
#include <fss_interfaces/msg/system_mode.hpp>
#include <fss_interfaces/msg/valve_status.hpp>
#include <fss_interfaces/srv/set_system_mode.hpp>
#include <fss_interfaces/srv/main_valve_control.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fss_manager/mode_fsm.hpp"

namespace fss_manager {

class ManagerNode final : public rclcpp::Node {
public:
    explicit ManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    ManagerConfig declare_config();
    void health_check();
    void publish_mode();
    ModeResponse set_mode(std::uint8_t requested_mode);
    void auto_open_main_valve();
    void auto_advance_active();

    double health_check_rate_hz_{0.0};
    ManagerCore core_;
    bool dry_bench_solenoid_{false};
    bool auto_activate_{false};
    bool auto_activate_pending_{false};
    bool auto_open_pending_{false};
    bool auto_open_succeeded_{false};
    double control_rate_min_hz_{0.0};
    Mode auto_expected_mode_{Mode::PRESSURIZING};
    std::optional<std::chrono::steady_clock::time_point> auto_activate_deadline_;
    std::optional<std::chrono::steady_clock::time_point> auto_ready_since_;
    std::chrono::steady_clock::time_point auto_open_deadline_;
    rclcpp::Client<fss_interfaces::srv::MainValveControl>::SharedPtr main_valve_client_;
    std::optional<rclcpp::Client<fss_interfaces::srv::MainValveControl>::SharedFuture> auto_open_future_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_sub_;
    rclcpp::Subscription<fss_interfaces::msg::SpacecraftState>::SharedPtr nav_sub_;
    rclcpp::Subscription<fss_interfaces::msg::SpacecraftState>::SharedPtr vicon_sub_;
    rclcpp::Subscription<fss_interfaces::msg::ValveStatus>::SharedPtr valve_sub_;
    rclcpp::Publisher<fss_interfaces::msg::SystemMode>::SharedPtr mode_pub_;
    rclcpp::Publisher<fss_interfaces::msg::SystemHealth>::SharedPtr health_pub_;
    rclcpp::Service<fss_interfaces::srv::SetSystemMode>::SharedPtr mode_service_;
    rclcpp::TimerBase::SharedPtr health_timer_;
    rclcpp::TimerBase::SharedPtr rate_timer_;
};

}  // namespace fss_manager
