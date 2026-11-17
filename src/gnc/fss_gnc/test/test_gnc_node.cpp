#include <gtest/gtest.h>

#include <fss_interfaces/msg/spacecraft_state.hpp>
#include <fss_interfaces/msg/system_mode.hpp>
#include <fss_interfaces/msg/thrust_commands.hpp>
#include <fss_interfaces/msg/trajectory_reference.hpp>
#include <fss_interfaces/srv/set_controller.hpp>
#include <fss_interfaces/srv/set_guidance_mode.hpp>
#include <fss_interfaces/srv/set_target.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <future>
#include <limits>
#include <optional>
#include <thread>
#include <vector>

#include "fss_gnc/gnc_node.hpp"

namespace fss::gnc {
namespace {

using namespace std::chrono_literals;
using State = fss_interfaces::msg::SpacecraftState;
using Command = fss_interfaces::msg::ThrustCommands;
using ReferenceMessage = fss_interfaces::msg::TrajectoryReference;
using ControllerService = fss_interfaces::srv::SetController;
using GuidanceService = fss_interfaces::srv::SetGuidanceMode;

std::vector<rclcpp::Parameter> parameters() {
    return {{"control_rate_hz", 100.0}, {"state_age_limit_ms", 100.0},
            {"translational.type", "pid"}, {"translational.kp", 1.0},
            {"translational.ki", 0.0}, {"translational.kd", 0.0}, {"translational.max_force", 2.0},
            {"rotational.type", "pid"}, {"rotational.kp", 1.0},
            {"rotational.ki", 0.0}, {"rotational.kd", 0.0}, {"rotational.max_torque", 1.0},
            {"allocator.thrust_weight", 0.01}, {"allocator.min_thrust", 0.0},
            {"allocator.max_thrust_per_nozzle", 2.0},
            {"allocator.thrust_map_file", "calibration/valve_thrust_measured.yaml"},
            {"guidance.mode", "target"}, {"guidance.target_x", 1.0},
            {"guidance.target_y", 0.0}, {"guidance.target_theta", 0.0}};
}

class GncNodeTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override {
        const char* previous = std::getenv("FSS_CONFIG_DIR");
        if (previous)
            previous_config_ = previous;
        setenv("FSS_CONFIG_DIR", FSS_TEST_CONFIG_DIR, 1);
        observer_ = std::make_shared<rclcpp::Node>("gnc_test_observer");
        state_pub_ = observer_->create_publisher<State>("/navigation/state", 10);
        command_sub_ = observer_->create_subscription<Command>("/actuators/thrust_commands", 10,
            [this](Command::ConstSharedPtr command) {
                command_ = *command;
                ++commands_received_;
            });
        reference_sub_ = observer_->create_subscription<ReferenceMessage>("/gnc/trajectory_ref", 10,
            [this](ReferenceMessage::ConstSharedPtr reference) { reference_ = *reference; });
        controller_client_ = observer_->create_client<ControllerService>("/gnc/set_controller");
        guidance_client_ = observer_->create_client<GuidanceService>("/gnc/set_guidance_mode");
        executor_.add_node(observer_);
    }

    void TearDown() override {
        if (gnc_)
            executor_.remove_node(gnc_);
        executor_.remove_node(observer_);
        gnc_.reset();
        observer_.reset();
        if (previous_config_)
            setenv("FSS_CONFIG_DIR", previous_config_->c_str(), 1);
        else
            unsetenv("FSS_CONFIG_DIR");
    }

    void start(std::vector<rclcpp::Parameter> overrides = parameters()) {
        gnc_ = make_gnc_node(rclcpp::NodeOptions().parameter_overrides(overrides));
        executor_.add_node(gnc_);
        ASSERT_TRUE(spin_until([this] { return state_pub_->get_subscription_count() > 0; }));
    }

    template <typename Predicate>
    bool spin_until(Predicate predicate, std::chrono::milliseconds timeout = 2000ms) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            executor_.spin_some();
            if (predicate())
                return true;
            std::this_thread::sleep_for(2ms);
        }
        return predicate();
    }

    void publish(double theta = 0, double x = 0) {
        State state;
        state.header.stamp = observer_->now();
        state.theta = theta;
        state.x = x;
        state_pub_->publish(state);
    }

    bool zero_command() const {
        return command_ && std::all_of(command_->thrust_n.begin(), command_->thrust_n.end(),
                                       [](double value) { return value == 0; });
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    rclcpp::Node::SharedPtr gnc_, observer_;
    rclcpp::Publisher<State>::SharedPtr state_pub_;
    rclcpp::Subscription<Command>::SharedPtr command_sub_;
    rclcpp::Subscription<ReferenceMessage>::SharedPtr reference_sub_;
    rclcpp::Client<ControllerService>::SharedPtr controller_client_;
    rclcpp::Client<GuidanceService>::SharedPtr guidance_client_;
    std::optional<Command> command_;
    std::optional<ReferenceMessage> reference_;
    std::optional<std::string> previous_config_;
    std::size_t commands_received_{};
};

TEST_F(GncNodeTest, MissingRequiredParameterAndInvalidRatesFailStartup) {
    EXPECT_THROW(make_gnc_node(), rclcpp::exceptions::ParameterUninitializedException);
    auto overrides = parameters();
    overrides[0] = rclcpp::Parameter("control_rate_hz", 0.0);
    EXPECT_THROW(make_gnc_node(rclcpp::NodeOptions().parameter_overrides(overrides)), std::invalid_argument);
}

TEST_F(GncNodeTest, EmitsReferenceAndMappedThrustThenZerosWhenStateIsStale) {
    start();
    publish();
    ASSERT_TRUE(spin_until([this] { return command_ && command_->thrust_n[0] > 0.1 && reference_; }));
    EXPECT_DOUBLE_EQ(reference_->x_ref, 1);
    EXPECT_DOUBLE_EQ(reference_->vx_ref, 0);
    for (std::size_t i = 0; i < 8; ++i) {
        EXPECT_TRUE(command_->enabled[i]);
        EXPECT_GE(command_->thrust_n[i], 0);
        EXPECT_LE(command_->thrust_n[i], 0.5);
    }
    const auto count = commands_received_;
    ASSERT_TRUE(spin_until([this, count] { return commands_received_ > count && zero_command(); }));
}

TEST_F(GncNodeTest, NonfiniteNavigationStateProducesZeroAndRecovers) {
    start();
    publish();
    ASSERT_TRUE(spin_until([this] { return command_ && command_->thrust_n[0] > 0.1; }));
    const auto count = commands_received_;
    publish(std::numeric_limits<double>::quiet_NaN());
    ASSERT_TRUE(spin_until([this, count] { return commands_received_ > count && zero_command(); }, 80ms));
    publish();
    ASSERT_TRUE(spin_until([this] { return command_ && command_->thrust_n[0] > 0.1; }));
}

TEST_F(GncNodeTest, OldFutureAndMalformedNavigationStampsDoNotProduceThrust) {
    start();
    State state;
    for (int age_ms : {1000, -1000}) {
        state.header.stamp = observer_->now() - rclcpp::Duration::from_seconds(age_ms / 1000.0);
        const auto count = commands_received_;
        state_pub_->publish(state);
        ASSERT_TRUE(spin_until([&] { return commands_received_ >= count + 3; }));
        EXPECT_TRUE(zero_command());
        EXPECT_FALSE(reference_);
    }
    state.header.stamp = observer_->now();
    state.header.stamp.nanosec = 1000000000U;
    const auto count = commands_received_;
    state_pub_->publish(state);
    ASSERT_TRUE(spin_until([&] { return commands_received_ >= count + 3; }));
    EXPECT_TRUE(zero_command());
    EXPECT_FALSE(reference_);
    publish();
    EXPECT_TRUE(spin_until([&] { return command_ && command_->thrust_n[0] > 0.1; }));
}

TEST_F(GncNodeTest, DuplicateAndReorderedNavigationCannotReplaceOrRefreshTheLastSample) {
    start();
    State state;
    state.header.stamp = observer_->now();
    state_pub_->publish(state);
    ASSERT_TRUE(spin_until([&] { return command_ && command_->thrust_n[0] > 0.1; }));
    const auto accepted_stamp = rclcpp::Time(state.header.stamp);
    // A changed position with the same or earlier timestamp must not replace the
    // accepted state, nor keep it alive by repeated receipt after its expiry.
    state.x = 1.0;
    for (int64_t offset_ns : {0, -1}) {
        state.header.stamp = accepted_stamp + rclcpp::Duration::from_nanoseconds(offset_ns);
        const auto count = commands_received_;
        state_pub_->publish(state);
        ASSERT_TRUE(spin_until([&] { return commands_received_ >= count + 2; }, 40ms));
        EXPECT_GT(command_->thrust_n[0], 0.1);
    }
    state.header.stamp = accepted_stamp;
    state.x = 0.0;
    auto duplicate_timer = observer_->create_wall_timer(5ms, [&] { state_pub_->publish(state); });
    ASSERT_TRUE(spin_until([&] { return zero_command(); }, 200ms));
    duplicate_timer->cancel();
    publish();
    EXPECT_TRUE(spin_until([&] { return command_ && command_->thrust_n[0] > 0.1; }));
}

TEST_F(GncNodeTest, ServicesRejectUnsupportedRequestsAndSwitchResetControllers) {
    start();
    ASSERT_TRUE(controller_client_->wait_for_service(1s));
    ASSERT_TRUE(guidance_client_->wait_for_service(1s));
    for (const auto& type : {"typo", " LQR ", ""}) {
        auto request = std::make_shared<ControllerService::Request>();
        request->controller_type = type;
        auto future = controller_client_->async_send_request(request);
        ASSERT_TRUE(spin_until([&future] { return future.wait_for(0ms) == std::future_status::ready; }));
        EXPECT_EQ(future.get()->success, std::string(type) != "typo");
    }
    for (const auto mode : {GuidanceService::Request::WAYPOINT, GuidanceService::Request::TRAJECTORY,
                           GuidanceService::Request::STATION_KEEP}) {
        auto request = std::make_shared<GuidanceService::Request>();
        request->mode = mode;
        auto future = guidance_client_->async_send_request(request);
        ASSERT_TRUE(spin_until([&future] { return future.wait_for(0ms) == std::future_status::ready; }));
        EXPECT_EQ(future.get()->success, mode == GuidanceService::Request::STATION_KEEP);
    }
    publish();
    EXPECT_TRUE(spin_until([this] { return command_ && command_->thrust_n[0] > 0.1; }));
}

TEST_F(GncNodeTest, LoadedTrajectoryCanSwitchToTargetAndBack) {
    auto overrides = parameters();
    for (auto& parameter : overrides)
        if (parameter.get_name() == "guidance.mode")
            parameter = rclcpp::Parameter("guidance.mode", "trajectory");
    overrides.emplace_back("guidance.traj_t", std::vector<double>{0, 10});
    overrides.emplace_back("guidance.traj_x", std::vector<double>{0, 2});
    overrides.emplace_back("guidance.traj_y", std::vector<double>{0, 0});
    overrides.emplace_back("guidance.traj_theta", std::vector<double>{0, 0});
    start(overrides);
    ASSERT_TRUE(guidance_client_->wait_for_service(1s));
    for (const auto mode : {GuidanceService::Request::STATION_KEEP, GuidanceService::Request::TRAJECTORY}) {
        auto request = std::make_shared<GuidanceService::Request>();
        request->mode = mode;
        auto future = guidance_client_->async_send_request(request);
        ASSERT_TRUE(spin_until([&future] { return future.wait_for(0ms) == std::future_status::ready; }));
        EXPECT_TRUE(future.get()->success);
    }
    publish();
    ASSERT_TRUE(spin_until([this] { return reference_.has_value(); }));
    EXPECT_DOUBLE_EQ(reference_->x_ref, 0);
}

TEST_F(GncNodeTest, TargetServiceAppliesPidAndLqrAndRetainsTargetOnRejectedRequests) {
    using Target = fss_interfaces::srv::SetTarget;
    start();
    const auto client = observer_->create_client<Target>("/gnc/set_target");
    ASSERT_TRUE(client->wait_for_service(1s));
    const auto send = [&](const std::string& type, double x, bool accepted) {
        auto request = std::make_shared<Target::Request>();
        request->controller_type = type;
        request->x = x;
        request->y = 2.5;
        request->yaw = -0.4;
        auto future = client->async_send_request(request);
        ASSERT_TRUE(spin_until([&] { return future.wait_for(0ms) == std::future_status::ready; }));
        EXPECT_EQ(future.get()->success, accepted);
    };
    send("pid", 1.5, true);
    send("lqr", 2.0, true);
    send("unknown", 0.0, false);
    send("pid", std::numeric_limits<double>::quiet_NaN(), false);
    publish();
    ASSERT_TRUE(spin_until([this] { return reference_.has_value(); }));
    EXPECT_DOUBLE_EQ(reference_->x_ref, 2.0);
    EXPECT_DOUBLE_EQ(reference_->y_ref, 2.5);
    EXPECT_DOUBLE_EQ(reference_->theta_ref, -0.4);
    EXPECT_DOUBLE_EQ(reference_->vx_ref, 0.0);
    EXPECT_DOUBLE_EQ(reference_->vy_ref, 0.0);
    EXPECT_DOUBLE_EQ(reference_->omega_ref, 0.0);
}

TEST_F(GncNodeTest, ExplicitTargetAndFreshActiveModeGateThrustAndResetIntegral) {
    using Mode = fss_interfaces::msg::SystemMode;
    using Target = fss_interfaces::srv::SetTarget;
    auto overrides = parameters();
    overrides.emplace_back("require_explicit_target", true);
    for (auto& parameter : overrides) {
        if (parameter.get_name() == "translational.kp")
            parameter = rclcpp::Parameter("translational.kp", 0.0);
        if (parameter.get_name() == "translational.ki")
            parameter = rclcpp::Parameter("translational.ki", 2.0);
    }
    start(overrides);
    auto nav_timer = observer_->create_wall_timer(10ms, [this] { publish(); });
    auto mode_pub = observer_->create_publisher<Mode>("/system/mode", 1);
    ASSERT_TRUE(spin_until([&] { return mode_pub->get_subscription_count() > 0; }));
    auto mode = [&](uint8_t value, int age_ms = 0) {
        Mode msg;
        msg.mode = value;
        msg.header.stamp = observer_->now() - rclcpp::Duration::from_seconds(age_ms / 1000.0);
        mode_pub->publish(msg);
    };
    auto remains_idle = [&] {
        auto before = commands_received_;
        reference_.reset();
        EXPECT_TRUE(spin_until([&] { return commands_received_ >= before + 4; }));
        EXPECT_TRUE(zero_command());
        EXPECT_FALSE(reference_);
    };
    auto client = observer_->create_client<Target>("/gnc/set_target");
    ASSERT_TRUE(client->wait_for_service(1s));
    auto target = [&](std::string controller, bool accepted) {
        auto req = std::make_shared<Target::Request>();
        req->controller_type = controller;
        req->x = 1.0;
        auto future = client->async_send_request(req);
        ASSERT_TRUE(spin_until([&] { return future.wait_for(0ms) == std::future_status::ready; }));
        EXPECT_EQ(future.get()->success, accepted);
    };
    mode(Mode::ACTIVE);
    remains_idle();
    target("unknown", false);
    remains_idle();
    mode(Mode::IDLE);
    target("pid", true);
    remains_idle();
    mode(Mode::ACTIVE, 2000);
    remains_idle();
    mode(Mode::ACTIVE);
    ASSERT_TRUE(spin_until([&] { return reference_ && command_ && command_->thrust_n[0] > 0.09; }));
    const double accumulated = command_->thrust_n[0];
    mode(Mode::SAFE_HOLD);
    ASSERT_TRUE(spin_until([&] { return zero_command(); }));
    remains_idle();
    mode(Mode::ACTIVE);
    ASSERT_TRUE(spin_until([&] { return command_ && command_->thrust_n[0] > 0; }));
    EXPECT_LT(command_->thrust_n[0], accumulated * 0.6);
    // Navigation continues after the last ACTIVE heartbeat expires.
    ASSERT_TRUE(spin_until([&] { return zero_command(); }, 2s));
    remains_idle();
}

TEST_F(GncNodeTest, ControlTimerUsesRosClockAndPausesWithSimulation) {
    auto overrides = parameters();
    overrides.emplace_back("use_sim_time", true);
    start(overrides);
    EXPECT_FALSE(spin_until([this] { return command_.has_value(); }, 80ms));
    const auto clock_pub = observer_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    ASSERT_TRUE(spin_until([&clock_pub] { return clock_pub->get_subscription_count() > 0; }));
    rosgraph_msgs::msg::Clock clock;
    clock.clock.sec = 1;
    clock_pub->publish(clock);
    ASSERT_TRUE(spin_until([this] { return command_.has_value(); }));
    // Drain callbacks queued by the initial clock jump before measuring a paused clock.
    spin_until([] { return false; }, 30ms);
    EXPECT_DOUBLE_EQ(gnc_->now().seconds(), 1.0);
    const auto count = commands_received_;
    EXPECT_FALSE(spin_until([this, count] { return commands_received_ > count; }, 80ms));
    clock.clock.nanosec = 20000000;
    clock_pub->publish(clock);
    EXPECT_TRUE(spin_until([this, count] { return commands_received_ > count; }));
}

TEST_F(GncNodeTest, ClockRewindRequiresNewNavigationAndRestartsTrajectoryTime) {
    auto overrides = parameters();
    overrides.emplace_back("use_sim_time", true);
    for (auto& parameter : overrides)
        if (parameter.get_name() == "guidance.mode")
            parameter = rclcpp::Parameter("guidance.mode", "trajectory");
    overrides.emplace_back("guidance.traj_t", std::vector<double>{0, 10});
    overrides.emplace_back("guidance.traj_x", std::vector<double>{1, 2});
    overrides.emplace_back("guidance.traj_y", std::vector<double>{0, 0});
    overrides.emplace_back("guidance.traj_theta", std::vector<double>{0, 0});
    start(overrides);
    const auto clock_pub = observer_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    ASSERT_TRUE(spin_until([&] { return clock_pub->get_subscription_count() > 0; }));
    const auto clock_at = [&](int64_t milliseconds) {
        rosgraph_msgs::msg::Clock clock;
        clock.clock = rclcpp::Time(milliseconds * 1000000);
        clock_pub->publish(clock);
        ASSERT_TRUE(spin_until([&] { return gnc_->now().nanoseconds() == milliseconds * 1000000; }));
    };
    clock_at(10000);
    State state;
    state.header.stamp = gnc_->now();
    state_pub_->publish(state);
    spin_until([] { return false; }, 20ms);
    clock_at(10020);
    ASSERT_TRUE(spin_until([&] { return reference_ && command_ && command_->thrust_n[0] > 0.1; }));
    spin_until([] { return false; }, 20ms);
    reference_.reset();
    const auto count = commands_received_;
    clock_at(1000);
    clock_at(1020);
    ASSERT_TRUE(spin_until([&] { return commands_received_ > count && zero_command(); }));
    EXPECT_FALSE(reference_);
    state.header.stamp = gnc_->now();
    state_pub_->publish(state);
    spin_until([] { return false; }, 20ms);
    clock_at(1040);
    ASSERT_TRUE(spin_until([&] { return reference_ && command_ && command_->thrust_n[0] > 0.1; }));
    EXPECT_DOUBLE_EQ(reference_->x_ref, 1.0);
}

}  // namespace
}  // namespace fss::gnc
