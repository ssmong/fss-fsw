#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>

#include "fss_gnc/control_pipeline.hpp"

namespace fss::gnc {
namespace {

const std::filesystem::path kConfig{FSS_TEST_CONFIG_DIR};

ControlPipeline pipeline(Reference target = {1, 0, 0, 0, 0, 0}, double force_limit = 10,
                         std::filesystem::path directory = kConfig, double ki = 0) {
    const auto limits = load_thrust_limits(kConfig, "calibration/valve_thrust_measured.yaml", 2);
    return ControlPipeline({"pid", 1, ki, 0, force_limit}, {"pid", 1, 0, 0, 1},
                           GuidanceManager("target", target), directory, limits,
                           load_allocation_matrix(kConfig, limits), 0.01, 0);
}

TEST(Guidance, TargetAndTrajectoryEndpointsMatchPythonContract) {
    GuidanceManager target("target", {1, -0.5, 0.3, 0, 0, 0});
    EXPECT_DOUBLE_EQ(target.compute(7).x, 1);
    EXPECT_DOUBLE_EQ(target.compute(7).vy, 0);
    GuidanceManager guidance("trajectory", {}, {{0, 0, 0, 0}, {10, 1, 2, 0.5}});
    const auto middle = guidance.compute(5);
    EXPECT_DOUBLE_EQ(middle.x, 0.5);
    EXPECT_DOUBLE_EQ(middle.y, 1);
    EXPECT_DOUBLE_EQ(middle.theta, 0.25);
    EXPECT_DOUBLE_EQ(middle.vx, 0.1);
    EXPECT_DOUBLE_EQ(middle.vy, 0.2);
    EXPECT_DOUBLE_EQ(middle.omega, 0.05);
    EXPECT_DOUBLE_EQ(guidance.compute(-1).x, 0);
    EXPECT_DOUBLE_EQ(guidance.compute(0).vx, 0);
    EXPECT_DOUBLE_EQ(guidance.compute(10).vx, 0);
    EXPECT_DOUBLE_EQ(guidance.compute(100).x, 1);
}

TEST(Guidance, WrapsShortestAngleAndUsesLeftVelocityAtWaypoint) {
    GuidanceManager guidance("trajectory", {}, {{0, 0, 0, 3}, {2, 2, 0, -3}, {3, 6, 0, 0}});
    const auto middle = guidance.compute(1);
    EXPECT_NEAR(middle.theta, std::acos(-1.0), 1e-12);
    EXPECT_NEAR(middle.omega, (2 * std::acos(-1.0) - 6) / 2, 1e-12);
    EXPECT_DOUBLE_EQ(guidance.compute(2).vx, 1);
    EXPECT_DOUBLE_EQ(guidance.compute(2.1).vx, 4);
}

TEST(Guidance, RejectsInvalidModesTimesAndMissingTrajectory) {
    EXPECT_THROW(GuidanceManager("typo", {}), std::invalid_argument);
    EXPECT_THROW(GuidanceManager("trajectory", {}), std::invalid_argument);
    EXPECT_THROW(GuidanceManager("trajectory", {}, {{0, 0, 0, 0}, {0, 1, 0, 0}}), std::invalid_argument);
    EXPECT_THROW(GuidanceManager("target", {std::numeric_limits<double>::infinity(), 0, 0, 0, 0, 0}),
                 std::invalid_argument);
    GuidanceManager target("target", {});
    EXPECT_THROW(target.set_mode("trajectory"), std::invalid_argument);
    EXPECT_EQ(target.mode(), "target");
}

TEST(RuntimeConfig, MeasuredMapScalesGeometryAndOverridesFallback) {
    const auto limits = load_thrust_limits(kConfig, "calibration/valve_thrust_measured.yaml", 2);
    for (const auto maximum : limits)
        EXPECT_DOUBLE_EQ(maximum, 0.5);
    const auto matrix = load_allocation_matrix(kConfig, limits);
    EXPECT_DOUBLE_EQ(matrix[0], 0.5);
    EXPECT_DOUBLE_EQ(matrix[2], -0.5);
    EXPECT_DOUBLE_EQ(matrix[16], -0.025);
    EXPECT_DOUBLE_EQ(matrix[17], 0.025);
    EXPECT_DOUBLE_EQ(load_thrust_limits(kConfig, "", 2)[0], 2);
    EXPECT_THROW(load_thrust_limits(kConfig, "", 0), std::invalid_argument);
    EXPECT_THROW(load_lqr_gains<2>(kConfig, "K_trans"), std::invalid_argument);
    EXPECT_THROW(load_lqr_gains<8>(kConfig, "missing"), YAML::Exception);
}

TEST(RuntimeConfig, RejectsMalformedMapAndPreservesPerNozzleLimits) {
    const auto temporary = std::filesystem::temp_directory_path() /
        ("fss_gnc_map_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
    struct RemoveFile {
        std::filesystem::path path;
        ~RemoveFile() { std::error_code ignored; std::filesystem::remove(path, ignored); }
    } cleanup{temporary};
    const auto write = [&](const std::string& rows) {
        std::ofstream stream(temporary);
        stream << "duty: [0, 0.5, 1]\nthrust_n:\n" << rows;
    };
    std::string valid;
    for (int i = 0; i < 8; ++i)
        valid += "  - [0, 0.1, " + std::to_string(0.4 + i * 0.01) + "]\n";
    write(valid);
    const auto limits = load_thrust_limits(temporary.parent_path(), temporary.filename().string(), 2);
    EXPECT_DOUBLE_EQ(limits[0], 0.4);
    EXPECT_DOUBLE_EQ(limits[7], 0.47);
    write("  - [0, 0.2, 0.1]\n");
    EXPECT_THROW(load_thrust_limits(temporary.parent_path(), temporary.filename().string(), 2), std::invalid_argument);
    valid.replace(valid.find("[0, 0.1, 0.400000]"), 18, "[0, 0.5, 0.4]");
    write(valid);
    EXPECT_THROW(load_thrust_limits(temporary.parent_path(), temporary.filename().string(), 2), std::invalid_argument);
}

TEST(ControlPipeline, RotatesTableForceOnceAndPublishesPhysicalThrust) {
    const double half_pi = std::acos(-1.0) / 2;
    auto control = pipeline({1, 0, half_pi, 0, 0, 0});
    const auto result = control.compute({0, 0, 0, 0, half_pi}, 0, 0, 0.05);
    EXPECT_NEAR(result.thrust_n[0] + result.thrust_n[1], 0, 1e-8);
    EXPECT_GT(result.thrust_n[6] + result.thrust_n[7], 0.9);
    for (double thrust : result.thrust_n) {
        EXPECT_GE(thrust, 0);
        EXPECT_LE(thrust, 0.5);
    }
}

TEST(ControlPipeline, ClampsBodyForceAfterRotation) {
    const double angle = std::acos(-1.0) / 4;
    auto control = pipeline({10, 10, angle, 0, 0, 0}, 0.25);
    const auto result = control.compute({0, 0, 0, 0, angle}, 0, 0, 0.05);
    const auto limits = load_thrust_limits(kConfig, "calibration/valve_thrust_measured.yaml", 2);
    QPAllocator allocator;
    const auto matrix = load_allocation_matrix(kConfig, limits);
    allocator.set_B_matrix(matrix.data());
    const auto expected = allocator.allocate(0.25, 0, 0);
    for (std::size_t i = 0; i < 8; ++i)
        EXPECT_NEAR(result.thrust_n[i], expected.duty[i] * limits[i], 1e-10);
}

TEST(ControlPipeline, NonfiniteStateResetsIntegralAndCannotReachAllocator) {
    auto control = pipeline({0, 0, 0, 0, 0, 0}, 10, kConfig, 1);
    for (int i = 0; i < 20; ++i)
        control.compute({-1, 0, 0, 0, 0}, 0, i * 0.05, 0.05);
    EXPECT_THROW(control.compute({0, 0, 0, 0, std::numeric_limits<double>::quiet_NaN()}, 0, 0, 0.05),
                 std::invalid_argument);
    const auto recovered = control.compute({0, 0, 0, 0, 0}, 0, 0, 0.05);
    for (double thrust : recovered.thrust_n)
        EXPECT_DOUBLE_EQ(thrust, 0);
}

TEST(ControlPipeline, FailedControllerSwitchKeepsActivePairAndLqrSwitchWorks) {
    auto control = pipeline({}, 10, kConfig / "does-not-exist");
    EXPECT_THROW(control.set_controller("lqr"), YAML::Exception);
    EXPECT_THROW(control.set_controller("typo"), std::invalid_argument);
    const auto still_pid = control.compute({-0.1, 0, 0, 0, 0}, 0, 0, 0.05);
    EXPECT_GT(still_pid.thrust_n[0], 0);
    auto lqr = pipeline();
    EXPECT_NO_THROW(lqr.set_controller("lqr"));
    const auto result = lqr.compute({0, 0, 0, 0, 0}, 0, 0, 0.05);
    EXPECT_GT(result.thrust_n[0], still_pid.thrust_n[0]);
}

TEST(ControlPipeline, FailureBuildingSecondControllerDoesNotReplaceFirstController) {
    const auto temporary = std::filesystem::temp_directory_path() /
        ("fss_gnc_gains_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(temporary / "gnc/controller");
    struct RemoveGains {
        std::filesystem::path directory;
        ~RemoveGains() {
            std::error_code ignored;
            std::filesystem::remove(directory / "gnc/controller/lqr_gains.yaml", ignored);
            std::filesystem::remove(directory / "gnc/controller", ignored);
            std::filesystem::remove(directory / "gnc", ignored);
            std::filesystem::remove(directory, ignored);
        }
    } cleanup{temporary};
    std::ofstream(temporary / "gnc/controller/lqr_gains.yaml")
        << "lqr:\n  K_trans: [100, 0, 0, 0, 0, 100, 0, 0]\n  K_rot: [1]\n";
    auto control = pipeline({1, 0, 0, 0, 0, 0}, 10, temporary);
    const auto before = control.compute({0.9, 0, 0, 0, 0}, 0, 0, 0.05);
    EXPECT_THROW(control.set_controller("lqr"), std::invalid_argument);
    const auto after = control.compute({0.9, 0, 0, 0, 0}, 0, 0, 0.05);
    EXPECT_EQ(before.thrust_n, after.thrust_n);
}

}  // namespace
}  // namespace fss::gnc
