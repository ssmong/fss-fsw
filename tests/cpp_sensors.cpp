#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <fss_hal/imu_hal.hpp>
#include <fss_hal/pose_sensor.hpp>
#include <fss_hal/vicon_hal.hpp>

namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void near(double actual, double expected, double tolerance = 1e-10) {
  require(std::abs(actual - expected) <= tolerance, "Numeric comparison failed");
}

template <std::size_t N>
void near(const std::array<double, N>& actual, const std::array<double, N>& expected) {
  for (std::size_t i = 0; i < N; ++i) near(actual[i], expected[i]);
}

template <typename Function>
void rejects(Function function) {
  bool rejected = false;
  try { function(); } catch (const std::exception&) { rejected = true; }
  require(rejected, "Invalid input was accepted");
}

std::string packet(int frame = 1, double time = 100.0,
                   const std::string& object = R"({"valid":true,"x":1234.5,"y":-678.9,"yaw":90})") {
  return R"({"frame":)" + std::to_string(frame) + R"(,"t":)" + std::to_string(time) +
         R"(,"objects":{"robotA":)" + object + "}}";
}

void test_vicon_packets() {
  fss_hal::ViconPacketDecoder decoder("robotA");
  decoder.consume(packet());
  require(decoder.latest().has_value(), "Valid Vicon pose missing");
  near(decoder.latest()->x_m, 1.2345);
  near(decoder.latest()->y_m, -0.6789);
  near(decoder.latest()->yaw_rad, fss_hal::kPi / 2);
  require(decoder.latest()->frame_number == 1, "Vicon frame changed");
  near(decoder.latest()->source_time_sec, 100.0);
  decoder.consume(packet(3, 100.02));
  decoder.consume(packet(2, 100.01));
  require(decoder.latest()->frame_number == 3, "Reordered Vicon pose replaced newer pose");
  decoder.begin_poll();
  decoder.consume(packet(3, 100.03));
  decoder.consume(packet(4, 100.01));
  require(!decoder.latest(), "Duplicate/reordered Vicon pose emitted");
  decoder.consume(packet(0, 101));
  require(decoder.latest()->frame_number == 0, "Vicon frame counter restart was rejected");
  decoder.consume(packet(1, 102, R"({"valid":false})"));
  require(!decoder.latest(), "Lost newest target left an older valid pose");
  decoder.consume(packet(0, 101));
  require(!decoder.latest(), "Reordered pose revived lost target");
  decoder.consume(packet(2, 103));
  decoder.consume(R"({"frame":3,"t":104,"objects":{"other":{"valid":true}}})");
  require(!decoder.latest(), "Missing newest target left an older valid pose");
  decoder.reset();
  decoder.consume(packet());
  require(decoder.latest()->frame_number == 1, "Disconnect reset failed");
  decoder.begin_poll();
  require(!decoder.latest(), "Empty poll re-emitted an earlier measurement");

  const std::vector<std::string> malformed{
      "broken", std::string(1, static_cast<char>(0xff)), "null", "[]", "{}",
      R"({"frame":true,"t":100,"objects":{}})",
      R"({"frame":-1,"t":100,"objects":{}})",
      R"({"frame":1.5,"t":100,"objects":{}})",
      R"({"frame":1,"t":-1,"objects":{}})",
      R"({"frame":1,"t":1e1000,"objects":{}})",
      R"({"frame":1,"t":"100","objects":{}})",
      R"({"frame":1,"t":100,"objects":[]})",
      packet(2, 101, "[]"), packet(2, 101, R"({"valid":"false"})"),
      packet(2, 101, R"({"valid":true})"),
      packet(2, 101, R"({"valid":true,"x":true,"y":1,"yaw":0})"),
      packet(2, 101, R"({"valid":true,"x":1,"y":1,"yaw":NaN})"),
      packet(2, 101).substr(0, packet(2, 101).size() - 1),
      std::string(100, '[') + std::string(100, ']')};
  for (const auto& bad : malformed) {
    decoder.reset();
    decoder.consume(packet(0, 99));
    rejects([&] { decoder.consume(bad); });
    require(!decoder.latest(), "Malformed newest packet left an earlier valid pose");
    decoder.consume(packet(4, 105));
    require(decoder.latest()->frame_number == 4, "Malformed packet poisoned later valid pose");
  }
  decoder.reset();
  decoder.consume(packet(1, 100, R"({"valid":true,"x":10,"y":20,"yaw":-180})"));
  near(decoder.latest()->yaw_rad, -fss_hal::kPi);
  fss_hal::ViconPacketDecoder selected("other");
  selected.consume(R"({"frame":1,"t":100,"objects":{"robotA":{"valid":false},"other":{"valid":true,"x":10,"y":20,"yaw":90}}})");
  require(selected.latest().has_value(), "Configured Vicon object was not selected");
  near(selected.latest()->x_m, 0.01);
  rejects([] { fss_hal::ViconHAL invalid({"0.0.0.0", 0, "", "robotA"}); });
  rejects([] { fss_hal::ViconHAL invalid({"0.0.0.0", 65536, "", "robotA"}); });
  rejects([] { fss_hal::ViconHAL invalid({"not-an-ip", 9870, "", "robotA"}); });
  rejects([] { fss_hal::ViconHAL invalid({"0.0.0.0", 9870, "not-an-ip", "robotA"}); });
  rejects([] { fss_hal::ViconPacketDecoder invalid("  "); });
  fss_hal::ViconHAL disconnected({"127.0.0.1", 9870, "", "robotA"});
  rejects([&] { disconnected.read(); });
  disconnected.disconnect();
  disconnected.disconnect();
}

void test_vicon_frame_and_derivatives() {
  fss_hal::PlanarPoseTransform transform(3, 4, fss_hal::kPi / 2);
  auto first = transform.convert({1, 2, 0, 1, 100});
  near(first.x, 1); near(first.y, 5); near(first.theta, fss_hal::kPi / 2);
  near(first.vx, 0); near(first.vy, 0); near(first.omega, 0);
  auto next = transform.convert({1.1, 2, 0.1, 2, 100.1});
  near(next.vx, 0); near(next.vy, 1); near(next.omega, 1);
  for (double dt : {0.0, -1.0, 2.0, 1e-7, 1.0}) {
    fss_hal::PlanarPoseTransform gated(0, 0, 0);
    gated.convert({0, 0, 0, 1, 100});
    const auto state = gated.convert({100, 100, 1, 2, 100 + dt});
    near(state.vx, 0); near(state.vy, 0); near(state.omega, 0);
  }
  fss_hal::PlanarPoseTransform wrapped(0, 0, 0);
  wrapped.convert({0, 0, fss_hal::kPi - 0.01, 1, 100});
  near(wrapped.convert({0, 0, -fss_hal::kPi + 0.01, 2, 100.1}).omega, 0.2);
  rejects([] { fss_hal::PlanarPoseTransform invalid(0, 0, std::numeric_limits<double>::infinity()); });
}

fss_hal::ImuSample imu_sample() {
  fss_hal::ImuSample sample;
  sample.stamp_ns = 123000000007LL;
  sample.frame_id = "imu_link";
  sample.angular_velocity = {1, 0, 0};
  sample.linear_acceleration = {0, 1, 9.81};
  sample.orientation_xyzw = {0, 0, std::sqrt(0.5), std::sqrt(0.5)};
  sample.angular_velocity_covariance = {1, 0, 0, 0, 2, 0, 0, 0, 3};
  sample.orientation_covariance = sample.angular_velocity_covariance;
  return sample;
}

void test_imu_conversion() {
  const auto sample = imu_sample();
  fss_hal::XsensImuHAL hal({0, 0, std::sqrt(0.5), std::sqrt(0.5)}, "body");
  const auto out = hal.convert(sample);
  near(out.angular_velocity, fss_hal::ImuVector3{0, 1, 0});
  near(out.linear_acceleration, fss_hal::ImuVector3{-1, 0, 9.81});
  near(out.orientation_xyzw, fss_hal::ImuQuaternion{0, 0, 0, 1});
  const fss_hal::ImuCovariance covariance{2, 0, 0, 0, 1, 0, 0, 0, 3};
  near(out.angular_velocity_covariance, covariance);
  near(out.orientation_covariance, covariance);
  near(out.linear_acceleration_covariance, fss_hal::ImuCovariance{});
  require(out.stamp_ns == sample.stamp_ns && out.frame_id == "body", "IMU stamp/frame changed incorrectly");
  require(sample.frame_id == "imu_link", "Input IMU sample was mutated");

  auto missing = sample;
  missing.orientation_xyzw.fill(std::numeric_limits<double>::quiet_NaN());
  missing.orientation_covariance.fill(std::numeric_limits<double>::quiet_NaN());
  missing.orientation_covariance[0] = -1;
  missing.angular_velocity_covariance = {-1, 0, 0, 0, 0, 0, 0, 0, 0};
  const auto unavailable = hal.convert(missing);
  near(unavailable.orientation_xyzw, fss_hal::ImuQuaternion{});
  near(unavailable.orientation_covariance, fss_hal::ImuCovariance{-1, 0, 0, 0, 0, 0, 0, 0, 0});
  require(unavailable.angular_velocity_covariance == missing.angular_velocity_covariance,
          "Unavailable gyro covariance marker changed");
  missing = sample;
  missing.orientation_xyzw = {};
  require(hal.convert(missing).orientation_covariance[0] == -1, "Zero orientation quaternion not marked unavailable");

  // A non-diagonal covariance catches transpose and row-major indexing errors.
  auto correlated = sample;
  correlated.angular_velocity_covariance = {1, 0.2, 0.3, 0.2, 2, 0.4, 0.3, 0.4, 3};
  near(hal.convert(correlated).angular_velocity_covariance,
       fss_hal::ImuCovariance{2, -0.2, -0.4, -0.2, 1, 0.3, -0.4, 0.3, 3});

  for (double bad : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    for (int field = 0; field < 6; ++field) {
      auto invalid = sample;
      if (field == 0) invalid.angular_velocity[2] = bad;
      if (field == 1) invalid.linear_acceleration[2] = bad;
      if (field == 2) invalid.orientation_xyzw[3] = bad;
      if (field == 3) invalid.angular_velocity_covariance[8] = bad;
      if (field == 4) invalid.linear_acceleration_covariance[8] = bad;
      if (field == 5) invalid.orientation_covariance[8] = bad;
      rejects([&] { hal.convert(invalid); });
    }
  }
  rejects([] { fss_hal::XsensImuHAL invalid({0, 0, 0, 0}, "body"); });
  rejects([] { fss_hal::XsensImuHAL invalid({0, 0, std::numeric_limits<double>::quiet_NaN(), 1}, "body"); });
  int warnings = 0;
  fss_hal::XsensImuHAL warning_hal({0, 0, 0, 1}, "body", [&](const std::string&) { ++warnings; });
  auto saturated = sample;
  saturated.angular_velocity = {35, 0, 0};
  saturated.linear_acceleration = {191, 0, 0};
  const auto unchanged = warning_hal.convert(saturated);
  require(warnings == 2 && unchanged.angular_velocity == saturated.angular_velocity,
          "Saturated IMU data did not warn and pass through");
}
}  // namespace

int main() {
  try {
    test_vicon_packets();
    test_vicon_frame_and_derivatives();
    test_imu_conversion();
    std::cout << "C++ sensor checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
