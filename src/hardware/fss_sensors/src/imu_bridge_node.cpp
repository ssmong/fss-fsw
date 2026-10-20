#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <fss_hal/imu_hal.hpp>

namespace fss_sensors {

class ImuBridgeNode final : public rclcpp::Node {
 public:
  ImuBridgeNode() : Node("imu_bridge_node") {
    using rclcpp::ParameterType;
    declare_parameter("input_topic", ParameterType::PARAMETER_STRING);
    declare_parameter("output_topic", ParameterType::PARAMETER_STRING);
    declare_parameter("frame_id", ParameterType::PARAMETER_STRING);
    declare_parameter("mounting_quaternion_xyzw", ParameterType::PARAMETER_DOUBLE_ARRAY);
    const auto mounting = get_parameter("mounting_quaternion_xyzw").as_double_array();
    if (mounting.size() != 4) throw std::invalid_argument("mounting_quaternion_xyzw needs four values");
    hal_ = std::make_unique<fss_hal::XsensImuHAL>(
        fss_hal::ImuQuaternion{mounting[0], mounting[1], mounting[2], mounting[3]},
        get_parameter("frame_id").as_string(),
        [this](const std::string& message) { RCLCPP_WARN(get_logger(), "%s", message.c_str()); });
    publisher_ = create_publisher<sensor_msgs::msg::Imu>(get_parameter("output_topic").as_string(), 10);
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        get_parameter("input_topic").as_string(), 10,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr message) { on_imu(*message); });
    timer_ = create_wall_timer(std::chrono::seconds(5), [this] {
      if (message_count_ == 0) RCLCPP_WARN(get_logger(), "No IMU data received in the last 5 seconds");
      else RCLCPP_DEBUG(get_logger(), "Received %zu IMU messages in the last 5 seconds", message_count_);
      message_count_ = 0;
    });
    RCLCPP_INFO(get_logger(), "Xsens IMU bridge: %s -> %s (frame=%s)",
                get_parameter("input_topic").as_string().c_str(),
                get_parameter("output_topic").as_string().c_str(),
                get_parameter("frame_id").as_string().c_str());
  }

 private:
  void on_imu(const sensor_msgs::msg::Imu& message) {
    fss_hal::ImuSample sample;
    sample.stamp_ns = static_cast<std::int64_t>(message.header.stamp.sec) * 1000000000LL +
                      message.header.stamp.nanosec;
    sample.frame_id = message.header.frame_id;
    sample.angular_velocity = {message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z};
    sample.linear_acceleration = {message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z};
    sample.orientation_xyzw = {message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w};
    sample.angular_velocity_covariance = message.angular_velocity_covariance;
    sample.linear_acceleration_covariance = message.linear_acceleration_covariance;
    sample.orientation_covariance = message.orientation_covariance;
    try {
      const auto out = hal_->convert(sample);
      sensor_msgs::msg::Imu converted;
      converted.header = message.header;
      converted.header.frame_id = out.frame_id;
      converted.angular_velocity.x = out.angular_velocity[0];
      converted.angular_velocity.y = out.angular_velocity[1];
      converted.angular_velocity.z = out.angular_velocity[2];
      converted.linear_acceleration.x = out.linear_acceleration[0];
      converted.linear_acceleration.y = out.linear_acceleration[1];
      converted.linear_acceleration.z = out.linear_acceleration[2];
      converted.orientation.x = out.orientation_xyzw[0];
      converted.orientation.y = out.orientation_xyzw[1];
      converted.orientation.z = out.orientation_xyzw[2];
      converted.orientation.w = out.orientation_xyzw[3];
      converted.angular_velocity_covariance = out.angular_velocity_covariance;
      converted.linear_acceleration_covariance = out.linear_acceleration_covariance;
      converted.orientation_covariance = out.orientation_covariance;
      publisher_->publish(converted);
      ++message_count_;
    } catch (const std::invalid_argument& error) {
      RCLCPP_WARN(get_logger(), "%s; IMU measurement discarded", error.what());
    }
  }

  std::unique_ptr<fss_hal::XsensImuHAL> hal_;
  std::size_t message_count_{};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace fss_sensors

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<fss_sensors::ImuBridgeNode>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("imu_bridge_node"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
