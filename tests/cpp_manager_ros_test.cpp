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
    CHECK(modes.back().mode == SystemMode::SAFE_HOLD);
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

void automatic_start_keeps_arm_gate_and_external_cancellation() {
    for (const auto scenario : {0, 1, 2, 3}) {
        const bool dry = scenario == 1;
        const bool cancel = scenario == 2;
        const bool fail = scenario == 3;
        auto opts = options();
        auto params = opts.parameter_overrides();
        for (auto& parameter : params) {
            if (parameter.get_name() == "use_sim_time") parameter = rclcpp::Parameter("use_sim_time", false);
            if (parameter.get_name() == "control_rate_min_hz") parameter = rclcpp::Parameter("control_rate_min_hz", 1.0);
        }
        params.emplace_back("auto_activate", true);
        params.emplace_back("dry_bench_solenoid", dry);
        params.emplace_back("require_arm_status", true);
        opts.parameter_overrides(params);
        auto manager = std::make_shared<fss_manager::ManagerNode>(opts);
        auto helper = std::make_shared<rclcpp::Node>("cpp_manager_auto_test");
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(manager);
        executor.add_node(helper);
        int valve_calls = 0;
        bool valve_open = false;
        std::uint8_t last_mode = SystemMode::IDLE;
        auto valve_service = helper->create_service<fss_interfaces::srv::MainValveControl>(
            "/actuators/main_valve", [&](const std::shared_ptr<fss_interfaces::srv::MainValveControl::Request> request,
                                         std::shared_ptr<fss_interfaces::srv::MainValveControl::Response> response) {
                ++valve_calls;
                CHECK(request->open == !dry);
                valve_open = !fail && request->open;
                response->success = !fail;
            });
        auto mode_sub = helper->create_subscription<SystemMode>("/system/mode", 10,
            [&](const SystemMode& msg) { last_mode = msg.mode; });
        auto nav = helper->create_publisher<SpacecraftState>("/navigation/state", 10);
        auto vicon = helper->create_publisher<SpacecraftState>("/sensors/vicon_state", 10);
        auto valve = helper->create_publisher<ValveStatus>(dry ? "/actuators/dry_bench/valve_status" : "/actuators/valve_status", 10);
        auto arm = helper->create_publisher<std_msgs::msg::String>("/arm/operation_status", rclcpp::SensorDataQoS());
        auto mode_client = helper->create_client<SetSystemMode>("/system/set_mode");
        const auto started = std::chrono::steady_clock::now();
        bool cancellation_sent = false;
        std::optional<rclcpp::Client<SetSystemMode>::SharedFuture> cancellation;
        while (std::chrono::steady_clock::now() - started < 5s) {
            const auto elapsed = std::chrono::steady_clock::now() - started;
            SpacecraftState state;
            state.header.stamp = helper->now();
            state.std_x = state.std_y = 0.001;
            state.std_theta = 0.002;
            nav->publish(state);
            vicon->publish(state);
            ValveStatus feedback;
            feedback.main_valve_open = valve_open;
            valve->publish(feedback);
            std_msgs::msg::String status;
            status.data = elapsed < 1200ms ? "{invalid" :
                "{\"api_version\":1,\"status_stamp\":" + std::to_string(helper->now().seconds()) + "}";
            arm->publish(status);
            if (elapsed < 1200ms) CHECK(last_mode != SystemMode::ACTIVE);
            if (cancel && !cancellation_sent && last_mode == SystemMode::PRESSURIZING) {
                auto request = std::make_shared<SetSystemMode::Request>();
                request->requested_mode = SystemMode::IDLE;
                cancellation = mode_client->async_send_request(request).future.share();
                cancellation_sent = true;
            }
            executor.spin_once(10ms);
            if (!cancel && !fail && last_mode == SystemMode::ACTIVE) break;
            if ((cancel || fail) && elapsed > 2200ms) break;
        }
        CHECK(valve_calls == 1);
        if (cancel) {
            CHECK(cancellation_sent);
            CHECK(cancellation->wait_for(0s) == std::future_status::ready);
            CHECK(cancellation->get()->success);
            CHECK(last_mode == SystemMode::IDLE);
        } else if (fail) {
            CHECK(last_mode == SystemMode::PRESSURIZING);
        } else {
            CHECK(last_mode == SystemMode::ACTIVE);
        }
    }
}
}  // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        startup_requires_valid_parameters();
        adapter_topics_services_and_clock();
        automatic_start_keeps_arm_gate_and_external_cancellation();
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
