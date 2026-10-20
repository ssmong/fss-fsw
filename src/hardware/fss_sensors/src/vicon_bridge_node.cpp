#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <fss_interfaces/msg/spacecraft_state.hpp>
#include <fss_hal/vicon_hal.hpp>

namespace fss_sensors {

class ViconBridgeNode final : public rclcpp::Node {
 public:
  ViconBridgeNode() : Node("vicon_bridge_node") {
    using rclcpp::ParameterType;
    declare_parameter("bind_ip", ParameterType::PARAMETER_STRING);
    declare_parameter("port", ParameterType::PARAMETER_INTEGER);
    declare_parameter("source_ip", ParameterType::PARAMETER_STRING);
    declare_parameter("object_name", ParameterType::PARAMETER_STRING);
    declare_parameter("poll_rate_hz", ParameterType::PARAMETER_DOUBLE);
    declare_parameter("vicon_frame_id", ParameterType::PARAMETER_STRING);
    declare_parameter("frame_offset_x", ParameterType::PARAMETER_DOUBLE);
    declare_parameter("frame_offset_y", ParameterType::PARAMETER_DOUBLE);
    declare_parameter("frame_rotation_deg", ParameterType::PARAMETER_DOUBLE);

    const auto port = get_parameter("port").as_int();
    if (port < 1 || port > 65535) throw std::invalid_argument("Vicon port must be in 1..65535");
    const double poll_rate = get_parameter("poll_rate_hz").as_double();
    if (!std::isfinite(poll_rate) || poll_rate <= 0.0) {
      throw std::invalid_argument("poll_rate_hz must be positive and finite");
    }
    const double period_sec = 1.0 / poll_rate;
    if (!std::isfinite(period_sec) || period_sec < 1e-9 ||
        period_sec >= std::chrono::duration<double>(std::chrono::nanoseconds::max()).count()) {
      throw std::invalid_argument("poll_rate_hz is outside the timer range");
    }
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_sec));
    frame_id_ = get_parameter("vicon_frame_id").as_string();
    transform_ = std::make_unique<fss_hal::PlanarPoseTransform>(
        get_parameter("frame_offset_x").as_double(), get_parameter("frame_offset_y").as_double(),
        get_parameter("frame_rotation_deg").as_double() * (fss_hal::kPi / 180.0));
    hal_ = std::make_unique<fss_hal::ViconHAL>(
        fss_hal::ViconConfig{get_parameter("bind_ip").as_string(), static_cast<int>(port),
                             get_parameter("source_ip").as_string(),
                             get_parameter("object_name").as_string()},
        [this](const std::string& message) { RCLCPP_WARN(get_logger(), "%s", message.c_str()); });
    hal_->connect();
    publisher_ = create_publisher<fss_interfaces::msg::SpacecraftState>("/sensors/vicon_state", 10);
    timer_ = create_wall_timer(period, [this] { poll(); });
    RCLCPP_INFO(get_logger(), "FSS_VICON UDP listening on %s:%ld, object=%s",
                get_parameter("bind_ip").as_string().c_str(), static_cast<long>(port),
                get_parameter("object_name").as_string().c_str());
  }

 private:
  void poll() {
    const auto sample = hal_->read();
    if (!sample) return;
    try {
      const auto state = transform_->convert(*sample);
      fss_interfaces::msg::SpacecraftState message;
      // Sender time supplies velocity only; Jetson ROS receive time is uncorrected for latency.
      message.header.stamp = now();
      message.header.frame_id = frame_id_;
      message.x = state.x;
      message.y = state.y;
      message.theta = state.theta;
      message.vx = state.vx;
      message.vy = state.vy;
      message.omega = state.omega;
      publisher_->publish(message);
    } catch (const std::invalid_argument& error) {
      RCLCPP_WARN(get_logger(), "%s; Vicon measurement discarded", error.what());
    }
  }

  std::string frame_id_;
  std::unique_ptr<fss_hal::ViconHAL> hal_;
  std::unique_ptr<fss_hal::PlanarPoseTransform> transform_;
  rclcpp::Publisher<fss_interfaces::msg::SpacecraftState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace fss_sensors

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<fss_sensors::ViconBridgeNode>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("vicon_bridge_node"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
