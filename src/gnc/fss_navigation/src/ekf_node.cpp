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
#include <optional>
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
        use_imu_ = declare_parameter("use_imu", true);
        parameters.gyro_noise_density = required_double("gyro_noise_density");
        parameters.accel_noise_density = required_double("accel_noise_density");
        parameters.accel_model_noise_density = declare_parameter("accel_model_noise_density", 0.0);
        parameters.gyro_bias_random_walk = required_double("gyro_bias_random_walk");
        parameters.accel_bias_random_walk = required_double("accel_bias_random_walk");
        parameters.table_accel_bias_random_walk = required_double("table_accel_bias_random_walk");
        parameters.vicon_position_noise = required_double("vicon_position_noise");
        parameters.vicon_heading_noise = required_double("vicon_heading_noise");
        parameters.vicon_timestamp_std_ms = declare_parameter("vicon_timestamp_std_ms", 0.0);
        vicon_position_noise_ = parameters.vicon_position_noise;
        vicon_heading_noise_ = parameters.vicon_heading_noise;
        velocity_lpf_hz_ = declare_parameter("vicon_only_velocity_lpf_hz", 5.0);
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
        for (const auto value : {timeout_ms, vicon_position_noise_, vicon_heading_noise_, velocity_lpf_hz_}) {
            if (!std::isfinite(value) || value <= 0.0) {
                throw std::invalid_argument("Vicon timeout, noise and velocity LPF must be finite and positive");
            }
        }
        NavigationTiming timing;
        timing.delay_ms = declare_parameter("vicon_delay_ms", 0.0);
        timing.history_ms = declare_parameter("vicon_history_ms", 200.0);
        const auto recovery_samples = declare_parameter<std::int64_t>("vicon_recovery_samples", 25);
        if (recovery_samples < 3) {
            throw std::invalid_argument("vicon_recovery_samples must be at least 3");
        }
        timing.recovery_samples = static_cast<std::size_t>(recovery_samples);
        timing.recovery_max_position = declare_parameter("vicon_recovery_max_position_m", 0.25);
        timing.recovery_max_heading = declare_parameter("vicon_recovery_max_heading_rad", 0.35);
        timing.recovery_max_speed = declare_parameter("vicon_recovery_max_speed_m_s", 1.0);
        timing.recovery_max_yaw_rate = declare_parameter("vicon_recovery_max_yaw_rate_rad_s", 2.0);
        navigation_ = std::make_unique<Navigation>(parameters, timeout_ms, max_cycles, timing);
        timeout_ns_ = static_cast<std::int64_t>(timeout_ms * 1e6);
        vicon_sub_ = create_subscription<fss_interfaces::msg::SpacecraftState>(
            "/sensors/vicon_state", 10,
            [this](const fss_interfaces::msg::SpacecraftState& msg) { on_vicon(msg); });
        if (use_imu_) {
            imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
                "/sensors/imu", 10, [this](const sensor_msgs::msg::Imu& msg) { on_imu(msg); });
        }
        state_pub_ = create_publisher<fss_interfaces::msg::SpacecraftState>("/navigation/state", 10);
        timer_ = rclcpp::create_timer(this, get_clock(),
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate)),
            [this] { publish_state(); });
        if (use_imu_) {
            nis_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::seconds(10),
                                             [this] { check_consistency(); });
        }
        RCLCPP_INFO(get_logger(), "C++ navigation: %s, %.1f Hz, Vicon timeout %.1f ms, delay %.1f ms",
                    use_imu_ ? "11-state EKF" : "Vicon only", rate, timeout_ms, timing.delay_ms);
    }

private:
    static std::int64_t stamp_ns(const builtin_interfaces::msg::Time& stamp) {
        return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    }

    void warn_input(const char* sensor, InputResult result, std::int64_t age_ns = 0) {
        if (result == InputResult::Stale) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "%s sample time mismatch: age_ms=%.3f, limit_ms=%.1f",
                sensor, age_ns * 1e-6, timeout_ns_ * 1e-6);
        } else if (result == InputResult::Invalid || result == InputResult::Reordered) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "%s sample discarded (invalid, unavailable or reordered): %d",
                sensor, static_cast<int>(result));
        }
    }

    void on_imu(const sensor_msgs::msg::Imu& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (msg.header.stamp.nanosec >= 1000000000U) {
            navigation_->clear_recovery();
            warn_input("IMU", InputResult::Invalid);
            return;
        }
        const auto now_ns = now().nanoseconds();
        const auto result = navigation_->on_imu(now_ns, stamp_ns(msg.header.stamp),
            msg.angular_velocity.z, msg.linear_acceleration.x, msg.linear_acceleration.y,
            !(msg.angular_velocity_covariance[0] < 0.0),
            !(msg.linear_acceleration_covariance[0] < 0.0));
        warn_input("IMU", result, now_ns - stamp_ns(msg.header.stamp));
        report_vicon_update();
    }

    void on_vicon(const fss_interfaces::msg::SpacecraftState& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (msg.header.stamp.nanosec >= 1000000000U) {
            navigation_->clear_recovery();
            warn_input("Vicon", InputResult::Invalid);
            return;
        }
        if (!use_imu_) {
            on_vicon_only(msg);
            return;
        }
        const auto now_ns = now().nanoseconds();
        const bool degraded = navigation_->navigation_degraded();
        const auto result = navigation_->on_vicon(now_ns, stamp_ns(msg.header.stamp),
                                                  msg.x, msg.y, msg.theta);
        warn_input("Vicon", result, now_ns - stamp_ns(msg.header.stamp));
        report_vicon_update();
        if (degraded && !navigation_->navigation_degraded()) {
            RCLCPP_INFO(get_logger(), "Vicon restored: normal navigation");
        }
    }

    void publish_state() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!use_imu_) {
            publish_vicon_only(now().nanoseconds());
            return;
        }
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

    void report_vicon_update() {
        if (reported_update_ == navigation_->update_count()) {
            return;
        }
        reported_update_ = navigation_->update_count();
        const auto& update = navigation_->last_update();
        if (update.outside_history) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Vicon sample outside IMU history: discarded");
        } else if (!update.accepted) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Vicon outlier rejected: residual dx=%.2f mm, dy=%.2f mm, yaw=%.3f deg; NIS=%.2f",
                update.innovation.x() * 1000, update.innovation.y() * 1000,
                update.innovation.z() * 180 / std::acos(-1.0), update.nis);
        } else if (update.recovered) {
            RCLCPP_WARN(get_logger(),
                "Vicon reacquired from consistent samples: previous residual %.3f m, %.2f deg",
                update.innovation.head<2>().norm(), update.innovation.z() * 180 / std::acos(-1.0));
        }
    }

    void on_vicon_only(const fss_interfaces::msg::SpacecraftState& msg) {
        const auto now_ns = now().nanoseconds();
        const auto sample_ns = stamp_ns(msg.header.stamp);
        const auto age = now_ns - sample_ns;
        if (age < 0 || age >= timeout_ns_) {
            warn_input("Vicon only", InputResult::Stale, age);
            return;
        }
        if (last_vicon_only_stamp_ && sample_ns <= *last_vicon_only_stamp_) {
            return;
        }
        if (!(std::isfinite(msg.x) && std::isfinite(msg.y) && std::isfinite(msg.theta) &&
              std::isfinite(msg.vx) && std::isfinite(msg.vy) && std::isfinite(msg.omega))) {
            vicon_only_state_.reset();
            last_vicon_only_stamp_ = sample_ns;
            vicon_only_degraded_ = true;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Vicon-only pose/velocity is nonfinite: publication suspended");
            return;
        }
        auto state = msg;
        const double pi = std::acos(-1.0);
        state.theta = std::remainder(state.theta, 2.0 * pi);
        if (vicon_only_state_) {
            const auto gap = sample_ns - stamp_ns(vicon_only_state_->header.stamp);
            if (gap > 0 && gap < timeout_ns_) {
                const double alpha = -std::expm1(-2.0 * pi * velocity_lpf_hz_ * gap * 1e-9);
                state.vx = (1.0 - alpha) * vicon_only_state_->vx + alpha * msg.vx;
                state.vy = (1.0 - alpha) * vicon_only_state_->vy + alpha * msg.vy;
                state.omega = (1.0 - alpha) * vicon_only_state_->omega + alpha * msg.omega;
            }
        }
        state.std_x = state.std_y = vicon_position_noise_;
        state.std_theta = vicon_heading_noise_;
        vicon_only_state_ = state;
        last_vicon_only_stamp_ = sample_ns;
        if (vicon_only_degraded_) {
            vicon_only_degraded_ = false;
            RCLCPP_INFO(get_logger(), "Vicon restored: Vicon-only publication resumed");
        }
    }

    void publish_vicon_only(std::int64_t now_ns) {
        if (!vicon_only_state_) {
            return;
        }
        const auto age = now_ns - stamp_ns(vicon_only_state_->header.stamp);
        if (age < 0 || age >= timeout_ns_) {
            vicon_only_state_.reset();
            vicon_only_degraded_ = true;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Vicon-only input expired: publication suspended");
            return;
        }
        state_pub_->publish(*vicon_only_state_);
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
    bool use_imu_ = true;
    bool vicon_only_degraded_ = false;
    double velocity_lpf_hz_ = 5.0;
    double vicon_position_noise_ = 0.0;
    double vicon_heading_noise_ = 0.0;
    std::int64_t timeout_ns_ = 0;
    std::uint64_t reported_update_ = 0;
    std::optional<std::int64_t> last_vicon_only_stamp_;
    std::optional<fss_interfaces::msg::SpacecraftState> vicon_only_state_;
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
