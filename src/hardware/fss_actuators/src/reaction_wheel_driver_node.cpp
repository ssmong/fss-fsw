#include "fss_actuators/reaction_wheel_control.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <memory>

namespace fss_actuators {
class ReactionWheelDriverNode final : public rclcpp::Node {
 public:
  ReactionWheelDriverNode() : Node("reaction_wheel_driver_node") {
    const char* config_override = std::getenv("FSS_CONFIG_DIR");
    const auto directory = config_override && *config_override
        ? std::filesystem::path(config_override)
        : std::filesystem::path(ament_index_cpp::get_package_share_directory("fss_bringup")) / "config";
    const std::string path = declare_parameter("config_file",
        (directory / "hardware/reaction_wheel.yaml").string());
    dry_run_ = declare_parameter("dry_run", true);
    const auto config = fss_hal::ReactionWheelConfig::from_yaml(path);
    if (!std::isfinite(1.0 / config.telemetry_rate_hz) || 1.0 / config.telemetry_rate_hz < 1e-9 ||
        1.0 / config.telemetry_rate_hz >= static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1e9)
      throw std::invalid_argument("telemetry_rate_hz must produce a finite positive timer period");
    if (config.allow_gnc_commands || (config.integration_state != "bench_only" &&
                                      config.integration_state != "diagnostic"))
      throw std::invalid_argument("Reaction wheel driver only supports bench_only/diagnostic commands");
    if (!dry_run_ && !config.enabled)
      throw std::invalid_argument("Real hardware requires reaction_wheel.enabled: true");
    if (dry_run_) hal_ = std::make_unique<fss_hal::SimReactionWheelHal>(config);
    else hal_ = std::make_unique<fss_hal::SerialReactionWheelHal>(config);
    control_ = std::make_unique<ReactionWheelControl>(*hal_, config);
    hal_->connect();
    torque_sub_ = create_subscription<std_msgs::msg::Float64>("/reaction_wheel/torque_cmd", 1,
        [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
          try { control_->command(msg->data); }
          catch (const std::exception& e) { RCLCPP_ERROR(get_logger(), "Wheel command rejected: %s", e.what()); }
        });
    status_pub_ = create_publisher<std_msgs::msg::String>("/reaction_wheel/status", 10);
    enable_service_ = create_service<std_srvs::srv::SetBool>("/reaction_wheel/enable", [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      try {
        if (request->data) control_->enable(); else control_->disable();
        response->success = true;
        response->message = request->data ? "Output enabled" : "STOP sent; awaiting measured state";
      } catch (const std::exception& e) { response->success = false; response->message = e.what(); }
    });
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / config.telemetry_rate_hz)), [this]() { poll(); });
    RCLCPP_INFO(get_logger(), "C++ reaction wheel ready (dry_run=%s, wheel torque in Nm)",
                dry_run_ ? "true" : "false");
  }
  ~ReactionWheelDriverNode() override {
    if (hal_) try { hal_->disconnect(); }
    catch (const std::exception& e) { RCLCPP_ERROR(get_logger(), "Wheel shutdown failed: %s", e.what()); }
  }
 private:
  void poll() {
    try { control_->step(); }
    catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Wheel communication failed: %s", e.what());
    }
    nlohmann::json state = {
      {"dry_run", dry_run_}, {"enabled", control_->enabled()}, {"connected", hal_->is_connected()},
      {"fault_latched", hal_->fault_latched()}, {"stop_confirmed", hal_->stop_confirmed()},
      {"last_error", control_->last_error()}, {"telemetry_age_sec", nullptr}, {"telemetry", nullptr}};
    if (const auto& status = control_->status()) {
      state["telemetry_age_sec"] = std::max(0.0, fss_hal::monotonic_seconds() - status->received_at);
      state["telemetry"] = {{"current_a", status->current_a}, {"duty", status->duty},
          {"generated_torque_nm", status->generated_torque_nm}, {"state", status->state},
          {"received_at", status->received_at}, {"fault_reason", status->fault_reason}};
    }
    std_msgs::msg::String msg;
    msg.data = state.dump();
    status_pub_->publish(msg);
  }
  bool dry_run_{true};
  std::unique_ptr<fss_hal::ReactionWheelHal> hal_;
  std::unique_ptr<ReactionWheelControl> control_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr torque_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace fss_actuators

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<fss_actuators::ReactionWheelDriverNode>()); }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("reaction_wheel_driver_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
