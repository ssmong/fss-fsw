#pragma once

#include <fss_interfaces/msg/spacecraft_state.hpp>
#include <fss_interfaces/msg/system_health.hpp>
#include <fss_interfaces/msg/system_mode.hpp>
#include <fss_interfaces/msg/valve_status.hpp>
#include <fss_interfaces/srv/set_system_mode.hpp>
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

    double health_check_rate_hz_{0.0};
    ManagerCore core_;
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
