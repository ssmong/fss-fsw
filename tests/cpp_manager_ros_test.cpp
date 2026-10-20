#include <rcl/time.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "fss_manager/manager_node.hpp"

using namespace std::chrono_literals;
using fss_interfaces::msg::SpacecraftState;
using fss_interfaces::msg::SystemHealth;
using fss_interfaces::msg::SystemMode;
using fss_interfaces::msg::ValveStatus;
using fss_interfaces::srv::SetSystemMode;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition))                                                      \
            throw std::runtime_error(std::string(__func__) + ": " #condition); \
    } while (false)

namespace {
rclcpp::NodeOptions options() {
    rclcpp::NodeOptions result;
    result.parameter_overrides({
        {"use_sim_time", true},
        {"health_check_rate_hz", 20.0},
        {"nav_age_limit_ms", 1000.0},
        {"control_rate_min_hz", 15.0},
        {"auto_safe_hold", true},
        {"nav_pos_std_limit", 0.005},
        {"nav_heading_std_limit", 0.02},
    });
    return result;
}

void startup_requires_valid_parameters() {
    bool rejected = false;
    try {
        auto node = std::make_shared<fss_manager::ManagerNode>();
    } catch (const std::exception&) {
        rejected = true;
    }
    CHECK(rejected);
    auto invalid = options();
    auto parameters = invalid.parameter_overrides();
    for (auto& parameter : parameters) {
        if (parameter.get_name() == "health_check_rate_hz") {
            parameter = rclcpp::Parameter("health_check_rate_hz", 0.0);
        }
    }
    invalid.parameter_overrides(parameters);
    rejected = false;
    try {
        auto node = std::make_shared<fss_manager::ManagerNode>(invalid);
    } catch (const std::exception&) {
        rejected = true;
    }
    CHECK(rejected);
}

void adapter_topics_services_and_clock() {
    auto manager = std::make_shared<fss_manager::ManagerNode>(options());
    auto helper = std::make_shared<rclcpp::Node>("cpp_manager_adapter_test");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(manager);
    executor.add_node(helper);

    std::vector<SystemMode> modes;
    std::vector<SystemHealth> health;
    auto mode_sub = helper->create_subscription<SystemMode>(
        "/system/mode", 10, [&modes](SystemMode::ConstSharedPtr msg) { modes.push_back(*msg); });
    auto health_sub = helper->create_subscription<SystemHealth>(
        "/system/health", 10,
        [&health](SystemHealth::ConstSharedPtr msg) { health.push_back(*msg); });
    auto nav_pub = helper->create_publisher<SpacecraftState>("/navigation/state", 10);
    auto vicon_pub = helper->create_publisher<SpacecraftState>("/sensors/vicon_state", 10);
    auto valve_pub = helper->create_publisher<ValveStatus>("/actuators/valve_status", 10);
    auto client = helper->create_client<SetSystemMode>("/system/set_mode");
    CHECK(client->wait_for_service(2s));

    const auto spin_for = [&executor](std::chrono::milliseconds duration) {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline)
            executor.spin_once(2ms);
    };
    const auto discovery_deadline = std::chrono::steady_clock::now() + 2s;
    while (nav_pub->get_subscription_count() == 0 || vicon_pub->get_subscription_count() == 0 ||
           valve_pub->get_subscription_count() == 0 || mode_sub->get_publisher_count() == 0 ||
           health_sub->get_publisher_count() == 0) {
        CHECK(std::chrono::steady_clock::now() < discovery_deadline);
        spin_for(10ms);
    }

    // Keep ROS time paused between explicit advances: service publication cannot hide
    // behind the periodic health timer.
    auto* clock = manager->get_clock()->get_clock_handle();
    CHECK(rcl_enable_ros_time_override(clock) == RCL_RET_OK);
    const auto time_ms = [&](std::int64_t ms) {
        CHECK(rcl_set_ros_time_override(clock, ms * 1000000) == RCL_RET_OK);
        spin_for(30ms);
    };
    const auto call = [&](std::uint8_t requested_mode) {
        auto request = std::make_shared<SetSystemMode::Request>();
        request->requested_mode = requested_mode;
        auto future = client->async_send_request(request);
        CHECK(executor.spin_until_future_complete(future, 1s) == rclcpp::FutureReturnCode::SUCCESS);
        spin_for(10ms);
        return future.get();
    };
    SpacecraftState state;
    state.std_x = 0.001;
    state.std_y = 0.001;
    state.std_theta = 0.002;
    ValveStatus valve;
    valve.modbus_rtt_ms = 3.5;
    const auto publish_sensors = [&]() {
        nav_pub->publish(state);
        vicon_pub->publish(state);
        valve_pub->publish(valve);
        spin_for(20ms);
    };
    const auto healthy_tick = [&](std::int64_t ms) {
        time_ms(ms - 1);
        publish_sensors();
        time_ms(ms);
    };

    time_ms(1000);
    CHECK(!health.empty());
    CHECK(!modes.empty());
    CHECK(modes.back().mode == SystemMode::IDLE);
    CHECK(!health.back().navigation_ok);
    CHECK(!health.back().actuators_ok);
    CHECK(health.back().navigation_degraded);
    CHECK(health.back().navigation_age_sec == 9.999);
    CHECK(!call(SystemMode::PRESSURIZING)->success);
    const auto mode_count = modes.size();
    for (const auto invalid : {6, 255}) {
        const auto response = call(static_cast<std::uint8_t>(invalid));
        CHECK(!response->success);
        CHECK(response->actual_mode == SystemMode::IDLE);
    }
    CHECK(modes.size() == mode_count);

    publish_sensors();
    CHECK(!call(SystemMode::PRESSURIZING)->success);
    healthy_tick(1050);
    CHECK(health.back().navigation_ok);
    CHECK(health.back().actuators_ok);
    CHECK(!health.back().navigation_degraded);
    CHECK(health.back().modbus_rtt_ms == 3.5);
    const auto health_count = health.size();
    CHECK(call(SystemMode::PRESSURIZING)->success);
    CHECK(modes.back().mode == SystemMode::PRESSURIZING);
    CHECK(!call(SystemMode::READY)->success);
    valve.main_valve_open = true;
    publish_sensors();
    CHECK(call(SystemMode::READY)->success);
    CHECK(call(SystemMode::ACTIVE)->success);
    CHECK(modes.back().mode == SystemMode::ACTIVE);
    CHECK(modes.back().mode_name == "ACTIVE");
    CHECK(health.size() == health_count);

    state.std_x = 0.1;
    healthy_tick(1100);
    CHECK(modes.back().mode == SystemMode::SAFE_HOLD);
    CHECK(health.back().navigation_ok);
    CHECK(health.back().actuators_ok);
    CHECK(health.back().navigation_degraded);
    CHECK(health.back().errors.size() == 1);
    state.std_x = 0.001;
    healthy_tick(1150);
    CHECK(modes.back().mode == SystemMode::SAFE_HOLD);
    CHECK(!call(SystemMode::ACTIVE)->success);
    healthy_tick(2000);
    healthy_tick(2050);
    CHECK(health.back().control_loop_rate_hz > 0.0);
    CHECK(call(SystemMode::READY)->success);
    CHECK(call(SystemMode::ACTIVE)->success);
    time_ms(2150);
    CHECK(modes.back().mode == SystemMode::ACTIVE);
    CHECK(health.back().navigation_ok);
    healthy_tick(2200);
    CHECK(modes.back().mode == SystemMode::ACTIVE);
    time_ms(3199);
    CHECK(modes.back().mode == SystemMode::ACTIVE);
    CHECK(health.back().navigation_ok);
    time_ms(3200);
    CHECK(modes.back().mode == SystemMode::SAFE_HOLD);
    CHECK(!health.back().navigation_ok);
    CHECK(!health.back().actuators_ok);
    healthy_tick(3250);
    CHECK(health.back().navigation_ok);
    CHECK(modes.back().mode == SystemMode::SAFE_HOLD);
    CHECK(!call(SystemMode::ACTIVE)->success);
    CHECK(call(SystemMode::READY)->success);
    CHECK(call(SystemMode::ACTIVE)->success);

    const auto before_estop_health = health.size();
    CHECK(call(SystemMode::EMERGENCY_STOP)->success);
    CHECK(modes.back().mode == SystemMode::EMERGENCY_STOP);
    CHECK(health.size() == before_estop_health);
    time_ms(3800);
    CHECK(modes.back().mode == SystemMode::EMERGENCY_STOP);
    CHECK(!call(SystemMode::ACTIVE)->success);
    CHECK(!call(SystemMode::EMERGENCY_STOP)->success);
    CHECK(call(SystemMode::IDLE)->success);
    CHECK(modes.back().mode == SystemMode::IDLE);
    CHECK(!call(SystemMode::PRESSURIZING)->success);
    CHECK(modes.back().header.stamp.sec == 3);
    CHECK(modes.back().header.stamp.nanosec == 800000000);
}
}  // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        startup_requires_valid_parameters();
        adapter_topics_services_and_clock();
        std::cout
            << "manager ROS startup, topics, service, fake ROS clock and safety tests passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
