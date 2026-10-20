#include "fss_gnc/gnc_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fss_interfaces/msg/spacecraft_state.hpp>
#include <fss_interfaces/msg/thrust_commands.hpp>
#include <fss_interfaces/msg/trajectory_reference.hpp>
#include <fss_interfaces/srv/set_controller.hpp>
#include <fss_interfaces/srv/set_guidance_mode.hpp>
#include <rclcpp/create_timer.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "fss_gnc/control_pipeline.hpp"

namespace fss::gnc {
namespace {

using SpacecraftState = fss_interfaces::msg::SpacecraftState;
using ThrustCommands = fss_interfaces::msg::ThrustCommands;
using TrajectoryReference = fss_interfaces::msg::TrajectoryReference;
using SetController = fss_interfaces::srv::SetController;
using SetGuidanceMode = fss_interfaces::srv::SetGuidanceMode;

std::filesystem::path config_directory() {
    const char* override_path = std::getenv("FSS_CONFIG_DIR");
    if (override_path && *override_path)
        return override_path;
    return std::filesystem::path(ament_index_cpp::get_package_share_directory("fss_bringup")) / "config";
}

std::string controller_type(std::string type) {
    const auto first = type.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
        return {};
    type = type.substr(first, type.find_last_not_of(" \t\r\n") - first + 1);
    std::transform(type.begin(), type.end(), type.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return type;
}

class GNCNode : public rclcpp::Node {
public:
    explicit GNCNode(const rclcpp::NodeOptions& options) : Node("gnc_node", options) {
        const double rate = required_double("control_rate_hz");
        const double age_ms = required_double("state_age_limit_ms");
        if (!std::isfinite(rate) || rate <= 0 || !std::isfinite(1.0 / rate) ||
            1.0 / rate < 1e-9 || 1.0 / rate >= static_cast<double>(std::numeric_limits<int64_t>::max()) / 1e9)
            throw std::invalid_argument("control_rate_hz must produce a finite positive timer period");
        if (!std::isfinite(age_ms) || age_ms * 1e6 < 1 ||
            age_ms >= static_cast<double>(std::numeric_limits<int64_t>::max()) / 1e6)
            throw std::invalid_argument("state_age_limit_ms must be positive and finite");
        dt_ = 1.0 / rate;
        state_age_limit_ns_ = static_cast<int64_t>(age_ms * 1e6);

        auto translation = read_controller("translational", "max_force");
        auto rotation = read_controller("rotational", "max_torque");
        const double weight = required_double("allocator.thrust_weight");
        const double minimum = required_double("allocator.min_thrust");
        const double fallback_thrust = required_double("allocator.max_thrust_per_nozzle");
        const auto map_file = required_string("allocator.thrust_map_file");
        const auto mode = required_string("guidance.mode");
        const Reference target{required_double("guidance.target_x"), required_double("guidance.target_y"),
                               required_double("guidance.target_theta"), 0, 0, 0};
        std::vector<TrajectoryPoint> points;
        if (mode == "trajectory") {
            const auto times = required_array("guidance.traj_t");
            const auto x = required_array("guidance.traj_x");
            const auto y = required_array("guidance.traj_y");
            const auto theta = required_array("guidance.traj_theta");
            if (times.size() != x.size() || times.size() != y.size() || times.size() != theta.size())
                throw std::invalid_argument("guidance.traj_* arrays must have equal lengths");
            for (std::size_t i = 0; i < times.size(); ++i)
                points.push_back({times[i], x[i], y[i], theta[i]});
        }
        const auto directory = config_directory();
        const auto limits = load_thrust_limits(directory, map_file, fallback_thrust);
        pipeline_ = std::make_unique<ControlPipeline>(translation, rotation,
            GuidanceManager(mode, target, std::move(points)), directory, limits,
            load_allocation_matrix(directory, limits), weight, minimum);

        command_pub_ = create_publisher<ThrustCommands>("/actuators/thrust_commands", 10);
        reference_pub_ = create_publisher<TrajectoryReference>("/gnc/trajectory_ref", 10);
        state_sub_ = create_subscription<SpacecraftState>("/navigation/state", 10,
            [this](SpacecraftState::ConstSharedPtr state) {
                latest_state_ = std::move(state);
                latest_received_ns_ = get_clock()->now().nanoseconds();
            });
        guidance_service_ = create_service<SetGuidanceMode>("/gnc/set_guidance_mode",
            [this](const std::shared_ptr<SetGuidanceMode::Request> request,
                   std::shared_ptr<SetGuidanceMode::Response> response) {
                try {
                    if (request->mode == SetGuidanceMode::Request::STATION_KEEP)
                        pipeline_->guidance().set_mode("target");
                    else if (request->mode == SetGuidanceMode::Request::TRAJECTORY)
                        pipeline_->guidance().set_mode("trajectory");
                    else
                        throw std::invalid_argument("unsupported guidance mode (WAYPOINT/PROXIMITY/MANUAL)");
                    response->success = true;
                    response->message = "Guidance mode: " + pipeline_->guidance().mode();
                } catch (const std::exception& error) {
                    response->success = false;
                    response->message = error.what();
                }
            });
        controller_service_ = create_service<SetController>("/gnc/set_controller",
            [this](const std::shared_ptr<SetController::Request> request,
                   std::shared_ptr<SetController::Response> response) {
                try {
                    const auto type = controller_type(request->controller_type);
                    if (type.empty())
                        pipeline_->reset();
                    else
                        pipeline_->set_controller(type);
                    response->success = true;
                    response->message = type.empty() ? "Controllers reset" : "Controller type: " + type;
                } catch (const std::exception& error) {
                    response->success = false;
                    response->message = error.what();
                }
            });
        timer_ = rclcpp::create_timer(this, get_clock(),
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(dt_)),
            [this] { control_loop(); });
        RCLCPP_INFO(get_logger(), "C++ GNC started (%.1f Hz, trans=%s, rot=%s)", rate,
                    translation.type.c_str(), rotation.type.c_str());
    }

private:
    double required_double(const std::string& name) {
        declare_parameter(name, rclcpp::ParameterType::PARAMETER_DOUBLE);
        return get_parameter(name).as_double();
    }

    std::string required_string(const std::string& name) {
        declare_parameter(name, rclcpp::ParameterType::PARAMETER_STRING);
        return get_parameter(name).as_string();
    }

    std::vector<double> required_array(const std::string& name) {
        declare_parameter(name, rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
        return get_parameter(name).as_double_array();
    }

    ControllerConfig read_controller(const std::string& prefix, const std::string& limit) {
        return {required_string(prefix + ".type"), required_double(prefix + ".kp"),
                required_double(prefix + ".ki"), required_double(prefix + ".kd"),
                required_double(prefix + "." + limit)};
    }

    void publish_zero() {
        ThrustCommands command;
        command.header.stamp = now();
        command.thrust_n.fill(0.0);
        command.enabled.fill(true);
        command_pub_->publish(command);
    }

    void control_loop() {
        const auto now_ns = get_clock()->now().nanoseconds();
        if (!latest_state_) {
            publish_zero();
            return;
        }
        const auto age_ns = now_ns - latest_received_ns_;
        if (age_ns < 0 || age_ns > state_age_limit_ns_) {
            pipeline_->reset();
            publish_zero();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Navigation state is stale; thrust is zero");
            return;
        }
        try {
            const auto& state = *latest_state_;
            const double stamp = state.header.stamp.sec + state.header.stamp.nanosec * 1e-9;
            if (!t0_)
                t0_ = stamp;
            const auto output = pipeline_->compute({state.x, state.y, state.vx, state.vy, state.theta},
                                                   state.omega, stamp - *t0_, dt_);
            TrajectoryReference reference;
            reference.header.stamp = now();
            reference.x_ref = output.reference.x;
            reference.y_ref = output.reference.y;
            reference.theta_ref = output.reference.theta;
            reference.vx_ref = output.reference.vx;
            reference.vy_ref = output.reference.vy;
            reference.omega_ref = output.reference.omega;
            reference_pub_->publish(reference);
            ThrustCommands command;
            command.header.stamp = now();
            command.thrust_n = output.thrust_n;
            command.enabled.fill(true);
            command_pub_->publish(command);
        } catch (const std::exception& error) {
            pipeline_->reset();
            publish_zero();
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "GNC command rejected: %s", error.what());
        }
    }

    double dt_{};
    int64_t state_age_limit_ns_{}, latest_received_ns_{};
    std::optional<double> t0_;
    std::unique_ptr<ControlPipeline> pipeline_;
    SpacecraftState::ConstSharedPtr latest_state_;
    rclcpp::Subscription<SpacecraftState>::SharedPtr state_sub_;
    rclcpp::Publisher<ThrustCommands>::SharedPtr command_pub_;
    rclcpp::Publisher<TrajectoryReference>::SharedPtr reference_pub_;
    rclcpp::Service<SetGuidanceMode>::SharedPtr guidance_service_;
    rclcpp::Service<SetController>::SharedPtr controller_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

rclcpp::Node::SharedPtr make_gnc_node(const rclcpp::NodeOptions& options) {
    return std::make_shared<GNCNode>(options);
}

}  // namespace fss::gnc
