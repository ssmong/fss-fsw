#include "fss_navigation/navigation.hpp"

#include <fss_interfaces/msg/spacecraft_state.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace fss_navigation {

class EKFNode : public rclcpp::Node {
public:
    EKFNode() : Node("ekf_node") {
        const auto required_double = [this](const char* name) {
            declare_parameter(name, rclcpp::ParameterType::PARAMETER_DOUBLE);
            return get_parameter(name).as_double();
        };
        const auto required_integer = [this](const char* name) {
            declare_parameter(name, rclcpp::ParameterType::PARAMETER_INTEGER);
            return get_parameter(name).as_int();
        };
        EKFParameters parameters;
        parameters.gyro_noise_density = required_double("gyro_noise_density");
        parameters.accel_noise_density = required_double("accel_noise_density");
        parameters.gyro_bias_random_walk = required_double("gyro_bias_random_walk");
        parameters.accel_bias_random_walk = required_double("accel_bias_random_walk");
        parameters.table_accel_bias_random_walk = required_double("table_accel_bias_random_walk");
        parameters.vicon_position_noise = required_double("vicon_position_noise");
        parameters.vicon_heading_noise = required_double("vicon_heading_noise");
        parameters.gate_threshold = required_double("gate_threshold");
        const auto window = required_integer("nis_window");
        if (window < 0) {
            throw std::invalid_argument("nis_window must be nonnegative");
        }
        parameters.nis_window = static_cast<std::size_t>(window);
        const double rate = required_double("publish_rate_hz");
        const double timeout_ms = required_double("vicon_timeout_ms");
        const auto max_cycles = required_integer("max_prediction_cycles");
        if (!std::isfinite(rate) || rate <= 0.0 || rate > 1e9) {
            throw std::invalid_argument("publish_rate_hz must be finite and in (0, 1e9]");
        }
        navigation_ = std::make_unique<Navigation>(parameters, timeout_ms, max_cycles);
        vicon_sub_ = create_subscription<fss_interfaces::msg::SpacecraftState>(
            "/sensors/vicon_state", 10,
            [this](const fss_interfaces::msg::SpacecraftState& msg) { on_vicon(msg); });
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/sensors/imu", 10, [this](const sensor_msgs::msg::Imu& msg) { on_imu(msg); });
        state_pub_ = create_publisher<fss_interfaces::msg::SpacecraftState>("/navigation/state", 10);
        timer_ = rclcpp::create_timer(this, get_clock(),
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate)),
            [this] { publish_state(); });
        nis_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::seconds(10),
                                         [this] { check_consistency(); });
        RCLCPP_INFO(get_logger(), "11-state C++ strapdown EKF: %.1f Hz, Vicon timeout %.1f ms",
                    rate, timeout_ms);
    }

private:
    static std::int64_t stamp_ns(const builtin_interfaces::msg::Time& stamp) {
        return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    }

    void warn_input(const char* sensor, InputResult result) {
        if (result == InputResult::Invalid || result == InputResult::Stale ||
            result == InputResult::Reordered || result == InputResult::Rejected) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "%s sample discarded (invalid, unavailable, stale, reordered or gated): %d",
                sensor, static_cast<int>(result));
        }
    }

    void on_imu(const sensor_msgs::msg::Imu& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (msg.header.stamp.nanosec >= 1000000000U) {
            warn_input("IMU", InputResult::Invalid);
            return;
        }
        const auto result = navigation_->on_imu(now().nanoseconds(), stamp_ns(msg.header.stamp),
            msg.angular_velocity.z, msg.linear_acceleration.x, msg.linear_acceleration.y,
            !(msg.angular_velocity_covariance[0] < 0.0),
            !(msg.linear_acceleration_covariance[0] < 0.0));
        warn_input("IMU", result);
    }

    void on_vicon(const fss_interfaces::msg::SpacecraftState& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (msg.header.stamp.nanosec >= 1000000000U) {
            warn_input("Vicon", InputResult::Invalid);
            return;
        }
        const bool degraded = navigation_->navigation_degraded();
        const auto result = navigation_->on_vicon(now().nanoseconds(), stamp_ns(msg.header.stamp),
                                                  msg.x, msg.y, msg.theta);
        warn_input("Vicon", result);
        if (degraded && !navigation_->navigation_degraded()) {
            RCLCPP_INFO(get_logger(), "Vicon restored: normal navigation");
        }
    }

    void publish_state() {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool degraded = navigation_->navigation_degraded();
        const auto state = navigation_->state(now().nanoseconds());
        if (!state) {
            if (navigation_->needs_vicon_initialization()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "IMU prediction stopped: publication suspended pending Vicon resynchronization");
            }
            return;
        }
        if (!degraded && navigation_->navigation_degraded()) {
            RCLCPP_WARN(get_logger(), "Vicon timeout: IMU dead reckoning (navigation_degraded)");
        }
        if (!std::isfinite(state->std_x) || !std::isfinite(state->std_y) ||
            !std::isfinite(state->std_theta)) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                "EKF covariance diverged: publishing infinite uncertainty");
        }
        fss_interfaces::msg::SpacecraftState msg;
        msg.header.stamp = rclcpp::Time(state->stamp_ns);
        msg.header.frame_id = "table";
        msg.x = state->x[PlanarEKF::IPX];
        msg.y = state->x[PlanarEKF::IPY];
        msg.theta = state->x[PlanarEKF::ITHETA];
        msg.vx = state->x[PlanarEKF::IVX];
        msg.vy = state->x[PlanarEKF::IVY];
        msg.omega = state->x[PlanarEKF::IOMEGA];
        msg.std_x = state->std_x;
        msg.std_y = state->std_y;
        msg.std_theta = state->std_theta;
        state_pub_->publish(msg);
    }

    void check_consistency() {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto& filter = navigation_->filter();
        const double nis = filter.nis_mean();
        if (nis > 0.0 && !filter.filter_consistent()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
                "EKF consistency warning: mean NIS %.2f (expected 3.0); check Q/R", nis);
        }
        if (nis > 0.0) {
            RCLCPP_DEBUG(get_logger(),
                "NIS %.2f, gyro bias %.6f rad/s, body accel bias (%.5f, %.5f), table bias (%.5f, %.5f)",
                nis, filter.x[PlanarEKF::IBG], filter.x[PlanarEKF::IBAX], filter.x[PlanarEKF::IBAY],
                filter.x[PlanarEKF::IBTX], filter.x[PlanarEKF::IBTY]);
        }
    }

    std::mutex mutex_;
    std::unique_ptr<Navigation> navigation_;
    rclcpp::Subscription<fss_interfaces::msg::SpacecraftState>::SharedPtr vicon_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<fss_interfaces::msg::SpacecraftState>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr nis_timer_;
};

}  // namespace fss_navigation

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        rclcpp::spin(std::make_shared<fss_navigation::EKFNode>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("ekf_node"), "%s", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
