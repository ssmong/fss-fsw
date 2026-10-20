#include "fss_actuators/valve_control.hpp"
#include "fss_hal/plc_hal.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fss_interfaces/msg/system_mode.hpp>
#include <fss_interfaces/msg/thrust_commands.hpp>
#include <fss_interfaces/msg/valve_commands.hpp>
#include <fss_interfaces/msg/valve_status.hpp>
#include <fss_interfaces/srv/main_valve_control.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>

namespace fss_actuators {
class ValveDriverNode final : public rclcpp::Node {
 public:
  ValveDriverNode() : Node("valve_driver_node") {
    const bool use_sim = declare_parameter("use_sim", false);
    rcl_interfaces::msg::ParameterDescriptor immutable;
    immutable.read_only = true;
    const bool use_thrusters = declare_parameter("use_thrusters", true, immutable);
    declare_parameter("plc.ip", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("plc.port", rclcpp::ParameterType::PARAMETER_INTEGER);
    declare_parameter("plc.timeout_sec", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter("plc.valve_register_start", rclcpp::ParameterType::PARAMETER_INTEGER);
    declare_parameter("plc.valve_count", rclcpp::ParameterType::PARAMETER_INTEGER);
    declare_parameter("plc.main_valve_register", rclcpp::ParameterType::PARAMETER_INTEGER);
    declare_parameter("plc.value_range", rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY);
    declare_parameter("plc.deadband_minimum", rclcpp::ParameterType::PARAMETER_INTEGER);
    declare_parameter("plc.control_rate_hz", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter("thrust_map_file", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("max_thrust_per_nozzle", rclcpp::ParameterType::PARAMETER_DOUBLE);

    fss_hal::PlcConfig config;
    config.ip = get_parameter("plc.ip").as_string();
    config.port = checked_integer("plc.port", 1, 65535);
    config.timeout_sec = get_parameter("plc.timeout_sec").as_double();
    config.valve_register_start = checked_integer("plc.valve_register_start", 0, 65535);
    config.valve_count = checked_integer("plc.valve_count", 8, 8);
    config.main_valve_register = checked_integer("plc.main_valve_register", 0, 65535);
    const auto range = get_parameter("plc.value_range").as_integer_array();
    if (range.size() != 2 || range[0] != 0 || range[1] < 1 || range[1] > 65535)
      throw std::invalid_argument("plc.value_range must be [0, positive uint16 maximum]");
    config.max_value = static_cast<int>(range[1]);
    const int deadband = checked_integer("plc.deadband_minimum", 0, config.max_value);
    const double rate = get_parameter("plc.control_rate_hz").as_double();
    if (!std::isfinite(rate) || rate <= 0.0 || !std::isfinite(1.0 / rate) ||
        1.0 / rate < 1e-9 || 1.0 / rate >= static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1e9)
      throw std::invalid_argument("plc.control_rate_hz must produce a finite positive timer period");
    std::unique_ptr<ValveThrustMap> map;
    const std::string map_file = get_parameter("thrust_map_file").as_string();
    if (!map_file.empty()) {
      std::string normalized = map_file;
      std::replace(normalized.begin(), normalized.end(), '\\', '/');
      std::filesystem::path path(normalized);
      if (!path.is_absolute()) {
        const char* config_override = std::getenv("FSS_CONFIG_DIR");
        const auto directory = config_override && *config_override
            ? std::filesystem::path(config_override)
            : std::filesystem::path(ament_index_cpp::get_package_share_directory("fss_bringup")) / "config";
        path = directory / path;
      }
      map = std::make_unique<ValveThrustMap>(ValveThrustMap::from_yaml(path.string()));
    }
    if (use_sim) hal_ = std::make_unique<fss_hal::SimHal>(config.valve_count, config.max_value);
    else hal_ = std::make_unique<fss_hal::PlcHal>(config);
    control_ = std::make_unique<ValveControl>(*hal_, config.max_value, deadband,
        get_parameter("max_thrust_per_nozzle").as_double(), use_thrusters, std::move(map));
    hal_->connect();

    applied_pub_ = create_publisher<fss_interfaces::msg::ValveCommands>("/actuators/valve_commands", 10);
    status_pub_ = create_publisher<fss_interfaces::msg::ValveStatus>("/actuators/valve_status", 10);
    thrust_sub_ = create_subscription<fss_interfaces::msg::ThrustCommands>(
        "/actuators/thrust_commands", 10, [this](fss_interfaces::msg::ThrustCommands::ConstSharedPtr msg) {
          try { control_->command(msg->thrust_n, msg->enabled); publish_applied(); publish_status(); }
          catch (const std::exception& e) { RCLCPP_ERROR(get_logger(), "Valve write failed: %s", e.what()); }
        });
    mode_sub_ = create_subscription<fss_interfaces::msg::SystemMode>(
        "/system/mode", 10, [this](fss_interfaces::msg::SystemMode::ConstSharedPtr msg) {
          try { if (control_->mode(msg->mode)) { publish_applied(); publish_status(); } }
          catch (const std::exception& e) { RCLCPP_ERROR(get_logger(), "Mode stop failed: %s", e.what()); }
        });
    main_service_ = create_service<fss_interfaces::srv::MainValveControl>(
        "/actuators/main_valve", [this](
            const std::shared_ptr<fss_interfaces::srv::MainValveControl::Request> request,
            std::shared_ptr<fss_interfaces::srv::MainValveControl::Response> response) {
          try {
            control_->main_valve(request->open);
            response->success = true;
            response->message = request->open ? "Main valve opened" : "Main valve closed";
          } catch (const std::exception& e) { response->success = false; response->message = e.what(); }
        });
    timer_ = rclcpp::create_timer(this, get_clock(),
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate)), [this]() {
      try { control_->heartbeat(); }
      catch (const std::exception& e) { RCLCPP_ERROR(get_logger(), "Valve heartbeat failed: %s", e.what()); }
      if (hal_->is_connected()) publish_status();
    });
    RCLCPP_INFO(get_logger(), "C++ valve driver ready (sim=%s, thrusters=%s)",
                use_sim ? "true" : "false", use_thrusters ? "true" : "false");
  }
  ~ValveDriverNode() override {
    if (hal_) try { hal_->disconnect(); }
    catch (const std::exception& e) { RCLCPP_ERROR(get_logger(), "Valve shutdown failed: %s", e.what()); }
  }
 private:
  int checked_integer(const char* name, int minimum, int maximum) {
    const auto value = get_parameter(name).as_int();
    if (value < minimum || value > maximum) throw std::invalid_argument(std::string("Invalid ") + name);
    return static_cast<int>(value);
  }
  void publish_applied() {
    fss_interfaces::msg::ValveCommands msg;
    msg.header.stamp = now();
    msg.duty_cycles = control_->applied_duty();
    msg.enabled.fill(true);
    applied_pub_->publish(msg);
  }
  void publish_status() {
    fss_interfaces::msg::ValveStatus msg;
    msg.header.stamp = now();
    msg.commanded_duty_cycles = control_->commanded();
    std::copy(control_->registers().begin(), control_->registers().end(), msg.applied_values.begin());
    msg.estimated_thrust = control_->estimated_thrust();
    msg.main_valve_open = control_->main_valve_open();
    msg.modbus_rtt_ms = hal_->last_rtt_ms();
    status_pub_->publish(msg);
  }
  std::unique_ptr<fss_hal::ValveHal> hal_;
  std::unique_ptr<ValveControl> control_;
  rclcpp::Publisher<fss_interfaces::msg::ValveCommands>::SharedPtr applied_pub_;
  rclcpp::Publisher<fss_interfaces::msg::ValveStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<fss_interfaces::msg::ThrustCommands>::SharedPtr thrust_sub_;
  rclcpp::Subscription<fss_interfaces::msg::SystemMode>::SharedPtr mode_sub_;
  rclcpp::Service<fss_interfaces::srv::MainValveControl>::SharedPtr main_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace fss_actuators

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<fss_actuators::ValveDriverNode>()); }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("valve_driver_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
