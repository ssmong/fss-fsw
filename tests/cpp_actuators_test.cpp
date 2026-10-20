#include "fss_actuators/reaction_wheel_control.hpp"
#include "fss_actuators/valve_control.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

namespace {
TEST(ValveThrustMap, PreservesMeasuredInterpolationAndSmallestPlateauInput) {
  fss_actuators::ValveThrustMap map({0, 0.2, 0.4, 0.6, 0.8, 1}, {{0, 0, 0.5, 0.5, 1, 1}}, 1);
  for (const auto& point : std::vector<std::pair<double, double>>{{0, 0}, {0.25, 0.3},
       {0.5, 0.4}, {0.75, 0.7}, {2, 0.8}}) {
    EXPECT_NEAR(map.duty_from_thrust({point.first})[0], point.second, 1e-12);
  }
  for (int i = 0; i <= 100; ++i) {
    const double target = i / 100.0;
    EXPECT_NEAR(map.thrust_from_duty(map.duty_from_thrust({target}))[0], target, 1e-12);
  }
  EXPECT_THROW(map.duty_from_thrust({NAN}), std::invalid_argument);
  EXPECT_THROW(map.thrust_from_duty({}), std::invalid_argument);
  EXPECT_THROW(fss_actuators::ValveThrustMap({0, 0, 1}, {{0, 0.2, 1}}, 1), std::invalid_argument);
  EXPECT_THROW(fss_actuators::ValveThrustMap({0, 1}, {{0, NAN}}, 1), std::invalid_argument);
  EXPECT_THROW(fss_actuators::ValveThrustMap({0, 0.5, 1}, {{0, 1, 0.5}}, 1), std::invalid_argument);
}

class FailingValves final : public fss_hal::ValveHal {
 public:
  void connect() override { connected = true; }
  void disconnect() override { connected = false; }
  void set_valves(const std::vector<double>& values) override {
    if (fail) throw std::runtime_error("Valve write failed");
    sent = values;
  }
  void set_main_valve(bool open) override {
    if (fail) throw std::runtime_error("Main valve write failed");
    main_writes.push_back(open);
  }
  void emergency_stop() override {
    if (fail) throw std::runtime_error("Stop write failed");
    sent.assign(8, 0.0); main_writes.push_back(false);
  }
  bool is_connected() const override { return connected; }
  bool connected{true}, fail{false};
  std::vector<double> sent;
  std::vector<bool> main_writes;
};

TEST(ValveControl, GatingStopsImmediatelyAndNeverReplaysCommandsOnReactivation) {
  fss_hal::SimHal hal;
  hal.connect();
  fss_actuators::ValveControl control(hal, 4000, 0, 2.0, true);
  std::array<double, 8> target{}; target[0] = 0.8;
  std::array<bool, 8> enabled{}; enabled.fill(true);
  control.command(target, enabled);
  EXPECT_EQ(hal.values()[0], 0);
  control.mode(3);
  EXPECT_EQ(hal.values()[0], 0);
  control.command(target, enabled);
  EXPECT_EQ(hal.values()[0], 1600);
  for (std::uint8_t mode : {0, 1, 2, 4, 5}) {
    control.mode(mode);
    EXPECT_EQ(hal.values(), std::vector<std::uint16_t>(8, 0));
    control.mode(3);
    EXPECT_EQ(hal.values()[0], 0);
    control.command(target, enabled);
  }
}

TEST(ValveControl, PublishedAppliedDutyUsesQuantizedRegistersAndMeasuredThrust) {
  fss_hal::SimHal hal;
  hal.connect();
  auto map = std::make_unique<fss_actuators::ValveThrustMap>(
      std::vector<double>{0, 1}, std::vector<std::vector<double>>(8, {0, 0.5}));
  fss_actuators::ValveControl control(hal, 4000, 0, 123.0, true, std::move(map));
  control.mode(3);
  std::array<double, 8> target{}; target[0] = 0.150095;
  std::array<bool, 8> enabled{}; enabled.fill(true);
  control.command(target, enabled);
  EXPECT_NEAR(control.commanded()[0], 0.30019, 1e-12);
  EXPECT_EQ(control.registers()[0], 1200);
  EXPECT_DOUBLE_EQ(control.applied_duty()[0], 0.3);
  EXPECT_DOUBLE_EQ(control.estimated_thrust()[0], 0.15);
  EXPECT_EQ(hal.values()[0], control.registers()[0]);
  enabled[0] = false;
  control.command(target, enabled);
  EXPECT_DOUBLE_EQ(control.applied_duty()[0], 0.0);
}

TEST(ValveControl, InvalidCommandsDeadbandAndDisabledThrustersFailClosed) {
  fss_hal::SimHal hal;
  hal.connect();
  std::array<double, 8> target{{NAN, INFINITY, -INFINITY, -1.0, 0.3, 0.4, 1.5, 0.0}};
  std::array<bool, 8> enabled{}; enabled.fill(true);
  fss_actuators::ValveControl control(hal, 4000, 1600, 1.0, true);
  control.mode(3); control.command(target, enabled);
  EXPECT_EQ(hal.values(), (std::vector<std::uint16_t>{0, 0, 0, 0, 0, 1600, 4000, 0}));
  fss_actuators::ValveControl disabled(hal, 4000, 0, 1.0, false);
  disabled.mode(3); disabled.command(target, enabled);
  EXPECT_EQ(hal.values(), std::vector<std::uint16_t>(8, 0));
}

TEST(ValveControl, FailedWritesRetainLastConfirmedOutputAndEmergencyCannotReopenMain) {
  FailingValves hal;
  fss_actuators::ValveControl control(hal, 4000, 0, 1.0, true);
  std::array<double, 8> target{}; target.fill(0.8);
  std::array<bool, 8> enabled{}; enabled.fill(true);
  control.mode(3); control.command(target, enabled); control.main_valve(true);
  hal.fail = true;
  EXPECT_THROW(control.mode(5), std::runtime_error);
  EXPECT_EQ(control.registers()[0], 3200);
  EXPECT_TRUE(control.main_valve_open());
  EXPECT_NO_THROW(control.heartbeat());
  EXPECT_EQ(hal.main_writes.size(), 1u);
  EXPECT_THROW(control.main_valve(true), std::runtime_error);
  hal.fail = false;
  control.mode(5);
  EXPECT_EQ(control.registers()[0], 0);
  EXPECT_FALSE(control.main_valve_open());
  control.mode(4);
  EXPECT_THROW(control.main_valve(true), std::runtime_error);
}

TEST(ValveControl, FailedCommandAndSafeHoldLeaveLastConfirmedStateUntilStopIsRetried) {
  FailingValves hal;
  fss_actuators::ValveControl control(hal, 4000, 0, 1.0, true);
  std::array<double, 8> target{}; target.fill(0.2);
  std::array<bool, 8> enabled{}; enabled.fill(true);
  control.mode(3); control.command(target, enabled); control.main_valve(true);
  hal.fail = true;
  target.fill(0.8);
  EXPECT_THROW(control.command(target, enabled), std::runtime_error);
  EXPECT_DOUBLE_EQ(control.commanded()[0], 0.2);
  EXPECT_EQ(control.registers()[0], 800);
  EXPECT_THROW(control.mode(4), std::runtime_error);
  EXPECT_EQ(control.registers()[0], 800);
  hal.fail = false;
  control.mode(4);
  EXPECT_EQ(control.registers()[0], 0);
  EXPECT_TRUE(control.main_valve_open());
  control.mode(3);
  EXPECT_EQ(control.registers()[0], 0);
}

TEST(ValveControl, MappedAndLinearCommandsHaveTheSameInvalidValueAndEnableBoundaries) {
  for (const bool mapped : {false, true}) {
    fss_hal::SimHal hal;
    hal.connect();
    auto map = mapped ? std::make_unique<fss_actuators::ValveThrustMap>(
        std::vector<double>{0, 1}, std::vector<std::vector<double>>(8, {0, 0.5})) : nullptr;
    fss_actuators::ValveControl control(hal, 4000, 1600, 0.5, true, std::move(map));
    control.mode(3);
    std::array<double, 8> target{{NAN, INFINITY, -INFINITY, -1.0, 0.15, 0.2, 100.0, 0.5}};
    std::array<bool, 8> enabled{}; enabled.fill(true); enabled[7] = false;
    control.command(target, enabled);
    EXPECT_EQ(hal.values(), (std::vector<std::uint16_t>{0, 0, 0, 0, 0, 1600, 4000, 0}));
  }
}

class FakeWheelSerial final : public fss_hal::WheelSerial {
 public:
  void open() override {
    if (open_error) throw std::runtime_error("USB port missing");
    connected = true;
  }
  void close() noexcept override { connected = false; }
  bool is_open() const override { return connected; }
  std::size_t available() override { return rx.size(); }
  std::string read(std::size_t count) override {
    if (read_error) throw std::runtime_error("USB read failed");
    const auto result = rx.substr(0, count); rx.erase(0, count); return result;
  }
  void write(const std::string& data) override {
    writes.push_back(data);
    if (write_error) throw std::runtime_error("USB partial write");
  }
  void reset_input() override { rx.clear(); }
  bool connected{false}, open_error{false}, read_error{false}, write_error{false};
  std::string rx;
  std::vector<std::string> writes;
};

struct SerialWheelFixture : testing::Test {
  void SetUp() override {
    auto serial = std::make_unique<FakeWheelSerial>();
    fake = serial.get();
    hal = std::make_unique<fss_hal::SerialReactionWheelHal>(config, std::move(serial), [this] { return now; });
  }
  void ready() {
    if (hal->is_connected()) hal->stop(); else hal->connect();
    fake->rx = "S,0,0,0,0\n";
    hal->read_status();
  }
  fss_hal::ReactionWheelConfig config;
  double now{10.0};
  FakeWheelSerial* fake;
  std::unique_ptr<fss_hal::SerialReactionWheelHal> hal;
};

TEST_F(SerialWheelFixture, FreshStopMeasurementRequiredAndAckDoesNotCount) {
  fake->rx = "S,0,0,0,0\n";
  hal->connect();
  EXPECT_FALSE(hal->read_status());
  EXPECT_EQ(fake->writes, std::vector<std::string>{"Z"});
  EXPECT_THROW(hal->set_torque(0.001), std::runtime_error);
  fake->rx = "ACK,STOP,Z\n"; hal->read_status();
  EXPECT_FALSE(hal->stop_confirmed());
  fake->rx = "S,0,0,0,0\n";
  EXPECT_EQ(hal->read_status()->received_at, now);
  EXPECT_TRUE(hal->stop_confirmed());
  hal->set_torque(-0.01);
  EXPECT_EQ(fake->writes.back(), "T,-0.01\n");
}

TEST_F(SerialWheelFixture, ParsesLatestTelemetryAndPartialLinesWithoutRefreshingAcks) {
  ready();
  fake->rx = "S,-0.1,";
  EXPECT_FALSE(hal->read_status());
  now += 0.02;
  fake->rx = "-0.2,-0.001405,1\nACK,T,-0.001405\nS,0";
  EXPECT_DOUBLE_EQ(hal->read_status()->current_a, -0.1);
  now += 0.02;
  fake->rx = ",0,0,0\n";
  EXPECT_EQ(hal->read_status()->state, 0);
  EXPECT_EQ(hal->last_ack(), "ACK,T,-0.001405");
  EXPECT_FALSE(hal->read_status());
}

TEST_F(SerialWheelFixture, FaultNeedsExplicitStopAndFreshStopTelemetry) {
  ready();
  fake->rx = "FAULT,OVERCURRENT\nS,0,0,0,2\nS,0,0,0,0\n";
  EXPECT_EQ(hal->read_status()->fault_reason, "OVERCURRENT");
  EXPECT_TRUE(hal->fault_latched());
  EXPECT_THROW(hal->set_torque(0.001), std::runtime_error);
  fake->rx = "S,0,0,0,0\n"; hal->stop();
  EXPECT_FALSE(hal->read_status());
  fake->rx = "ACK,STOP,Z\nS,0,0,0,1\n"; hal->read_status();
  EXPECT_TRUE(hal->fault_latched());
  fake->rx = "S,0,0,0,0\n"; hal->read_status();
  EXPECT_FALSE(hal->fault_latched());
  EXPECT_TRUE(hal->stop_confirmed());
}

TEST_F(SerialWheelFixture, PartialTimeoutStopsEvenIfRemainderIsNowAvailable) {
  ready(); fake->rx = "S,0"; hal->read_status();
  now += 0.051; fake->rx = ",0,0,0\n";
  EXPECT_THROW(hal->read_status(), std::runtime_error);
  EXPECT_TRUE(hal->fault_latched());
  EXPECT_EQ(fake->writes.back(), "Z");
}

TEST_F(SerialWheelFixture, MalformedAndNonfiniteFramesTripStop) {
  for (const std::string& frame : std::vector<std::string>{"garbage\n", "S,nan,0,0,0\n",
       "S,0,inf,0,1\n", "S,0,0,-inf,1\n", "S,0,0,0,7\n", "S,1,2,3\n",
       "ACK,T,nan\n", "ACK,T,1\n", "FAULT,\n", "S,0,0,0,1.0\n", "S,0,0,0,1x\n",
       std::string(1, static_cast<char>(0xff)) + "\n"}) {
    ready(); fake->rx = frame;
    EXPECT_THROW(hal->read_status(), std::exception) << frame;
    EXPECT_TRUE(hal->fault_latched());
    EXPECT_EQ(fake->writes.back(), "Z");
  }
}

TEST_F(SerialWheelFixture, RxAndIoFailuresAreBoundedAndLatched) {
  for (const auto& frame : std::vector<std::string>{std::string(257, 'x'),
       std::string(257, 'x') + "\n", std::string(4097, 'x')}) {
    ready(); fake->rx = frame;
    EXPECT_THROW(hal->read_status(), std::runtime_error);
    EXPECT_TRUE(hal->fault_latched());
  }
  ready(); fake->write_error = true;
  EXPECT_THROW(hal->set_torque(0.001), std::runtime_error);
  EXPECT_TRUE(hal->fault_latched());
  EXPECT_THROW(hal->disconnect(), std::runtime_error);
  EXPECT_FALSE(fake->connected);
}

TEST_F(SerialWheelFixture, ReadErrorAndOpenFailureDoNotLeaveTorqueArmed) {
  fake->open_error = true;
  EXPECT_THROW(hal->connect(), std::runtime_error);
  EXPECT_FALSE(hal->is_connected());
  fake->open_error = false;
  ready(); fake->rx = "S"; fake->read_error = true;
  EXPECT_THROW(hal->read_status(), std::runtime_error);
  EXPECT_TRUE(hal->fault_latched());
  EXPECT_EQ(fake->writes.back(), "Z");
  EXPECT_THROW(hal->set_torque(0.001), std::runtime_error);
}

TEST_F(SerialWheelFixture, FaultMessagesAreNotTelemetryAndInvalidCommandsNeverSendTorque) {
  ready(); fake->rx = "FAULT,OVERCURRENT\n";
  EXPECT_FALSE(hal->read_status());
  EXPECT_EQ(hal->fault_reason(), "OVERCURRENT");
  for (const double bad : std::vector<double>{NAN, INFINITY, -INFINITY, 0.013489, -0.013489}) {
    ready(); fake->writes.clear();
    EXPECT_THROW(hal->set_torque(bad), std::invalid_argument);
    EXPECT_EQ(fake->writes, std::vector<std::string>{"Z"});
    EXPECT_TRUE(hal->fault_latched());
  }
}

TEST(ReactionWheelConfig, RejectsInvalidContractAndLimitsBeforeOpeningHardware) {
  for (int field = 0; field < 10; ++field) {
    fss_hal::ReactionWheelConfig config;
    if (field == 0) config.firmware_variant = "other";
    if (field == 1) config.interface = "tcp";
    if (field == 2) config.torque_semantics = "body";
    if (field == 3) config.state_codes = {{1, 0, 2}};
    if (field == 4) config.partial_line_timeout_sec = 0.0;
    if (field == 5) config.write_timeout_sec = NAN;
    if (field == 6) config.max_torque_nm = -1.0;
    if (field == 7) config.stop_epsilon_nm = config.max_torque_nm;
    if (field == 8) config.command_timeout_sec = 0.01;
    if (field == 9) config.command_timeout_sec = config.rx_timeout_sec;
    EXPECT_THROW(fss_hal::SerialReactionWheelHal hal(config), std::invalid_argument);
    EXPECT_THROW(fss_hal::SimReactionWheelHal hal(config), std::invalid_argument);
  }
}

TEST(ReactionWheelSimulation, PreservesTorqueCurrentUnitsAndStopFaultHandshake) {
  fss_hal::ReactionWheelConfig config;
  fss_hal::SimReactionWheelHal hal(config);
  hal.connect();
  EXPECT_FALSE(hal.stop_confirmed());
  EXPECT_THROW(hal.set_torque(0.001), std::runtime_error);
  EXPECT_EQ(hal.read_status()->state, 0);
  hal.set_torque(0.007025); hal.set_torque(0.007025);
  const auto sample = hal.read_status();
  EXPECT_DOUBLE_EQ(sample->generated_torque_nm, 0.007025);
  EXPECT_NEAR(sample->current_a, 0.5, 1e-12);
  hal.set_torque(-0.5e-6);
  EXPECT_FALSE(hal.stop_confirmed());
  EXPECT_EQ(hal.read_status()->state, 0);
  EXPECT_THROW(hal.set_torque(NAN), std::invalid_argument);
  EXPECT_EQ(hal.read_status()->state, 2);
  EXPECT_THROW(hal.set_torque(0.001), std::runtime_error);
  hal.stop();
  EXPECT_TRUE(hal.fault_latched());
  EXPECT_EQ(hal.read_status()->state, 0);
  EXPECT_FALSE(hal.fault_latched());
  hal.disconnect();
  EXPECT_FALSE(hal.is_connected());
}

TEST_F(SerialWheelFixture, TorqueLimitsAndZeroStopAndConnectCleanup) {
  ready();
  hal->set_torque(config.max_torque_nm); hal->set_torque(-config.max_torque_nm);
  hal->set_torque(0.5e-6);
  EXPECT_EQ(fake->writes.back(), "Z");
  EXPECT_FALSE(hal->stop_confirmed());
  EXPECT_THROW(hal->set_torque(NAN), std::invalid_argument);
  EXPECT_TRUE(hal->fault_latched());
  hal->disconnect(); fake->write_error = true;
  EXPECT_THROW(hal->connect(), std::runtime_error);
  EXPECT_FALSE(hal->is_connected());
}

TEST(ReactionWheelSerial, PosixTransportSendsStopAndTorqueOverPseudoTerminal) {
  const int master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  ASSERT_GE(master, 0);
  ASSERT_EQ(grantpt(master), 0);
  ASSERT_EQ(unlockpt(master), 0);
  fss_hal::ReactionWheelConfig config;
  config.port = ptsname(master);
  fss_hal::SerialReactionWheelHal hal(config);
  hal.connect();
  pollfd descriptor{master, POLLIN, 0};
  ASSERT_GT(poll(&descriptor, 1, 100), 0);
  char bytes[128];
  ASSERT_EQ(::read(master, bytes, sizeof(bytes)), 1);
  EXPECT_EQ(bytes[0], 'Z');
  const std::string stop_sample = "S,0,0,0,0\n";
  ASSERT_EQ(::write(master, stop_sample.data(), stop_sample.size()), static_cast<ssize_t>(stop_sample.size()));
  for (int i = 0; i < 100 && !hal.stop_confirmed(); ++i) {
    hal.read_status();
    if (!hal.stop_confirmed()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(hal.stop_confirmed());
  hal.set_torque(-0.001);
  ASSERT_GT(poll(&descriptor, 1, 100), 0);
  const auto received = ::read(master, bytes, sizeof(bytes));
  ASSERT_GT(received, 0);
  EXPECT_EQ(std::string(bytes, static_cast<std::size_t>(received)), "T,-0.001\n");
  hal.disconnect();
  ::close(master);
}

TEST(ReactionWheelControl, DisabledCommandsNeverReplayAndTimeoutRequiresNewEnable) {
  fss_hal::ReactionWheelConfig config;
  double now = 1.0;
  fss_hal::SimReactionWheelHal hal(config, [&] { return now; });
  fss_actuators::ReactionWheelControl control(hal, config, [&] { return now; });
  hal.connect();
  EXPECT_THROW(control.enable(), std::runtime_error);
  control.command(0.002); control.step(); control.enable(); control.step();
  EXPECT_EQ(hal.read_status()->state, 0);
  control.command(0.003); control.step();
  EXPECT_DOUBLE_EQ(hal.read_status()->generated_torque_nm, 0.003);
  for (int i = 0; i < 5; ++i) { now += 0.06; control.step(); }
  EXPECT_FALSE(control.enabled());
  EXPECT_NE(control.last_error().find("command timeout"), std::string::npos);
  control.command(0.002); control.step();
  EXPECT_EQ(hal.read_status()->state, 0);
  control.enable(); control.command(0.002); control.step();
  EXPECT_EQ(hal.read_status()->state, 1);
}

TEST(ReactionWheelControl, HostStallCannotBeHiddenByFreshCommandOrQueuedTelemetry) {
  fss_hal::ReactionWheelConfig config;
  double now = 1.0;
  fss_hal::SimReactionWheelHal hal(config, [&] { return now; });
  fss_actuators::ReactionWheelControl control(hal, config, [&] { return now; });
  hal.connect(); control.step(); control.enable(); control.command(0.002); control.step();
  now += 0.6; control.command(0.002); control.step();
  EXPECT_FALSE(control.enabled());
  EXPECT_NE(control.last_error().find("telemetry timeout"), std::string::npos);
  EXPECT_EQ(hal.read_status()->state, 0);
}

TEST(ReactionWheelControl, InvalidTorqueDisablesOutputAndFaultNeedsFreshRearm) {
  fss_hal::ReactionWheelConfig config;
  double now = 1.0;
  fss_hal::SimReactionWheelHal hal(config, [&] { return now; });
  fss_actuators::ReactionWheelControl control(hal, config, [&] { return now; });
  hal.connect(); control.step(); control.enable(); control.command(0.002); control.step();
  EXPECT_THROW(control.command(INFINITY), std::invalid_argument);
  EXPECT_FALSE(control.enabled());
  EXPECT_EQ(hal.read_status()->state, 0);
  control.step(); control.enable();
  EXPECT_THROW(hal.set_torque(NAN), std::invalid_argument);
  control.step();
  EXPECT_FALSE(control.enabled());
  control.step();
  EXPECT_FALSE(control.enabled());
  EXPECT_NO_THROW(control.enable());
}

class ControlledWheel final : public fss_hal::ReactionWheelHal {
 public:
  ControlledWheel(const fss_hal::ReactionWheelConfig& config, fss_hal::WheelClock clock)
      : sim(config, std::move(clock)) {}
  void connect() override { sim.connect(); }
  void disconnect() override { sim.disconnect(); }
  void set_torque(double torque) override { writes.push_back(torque); sim.set_torque(torque); }
  void stop() override {
    ++stops;
    if (fail_stop) throw std::runtime_error("STOP write failed");
    sim.stop();
  }
  std::optional<fss_hal::WheelStatus> read_status() override {
    if (fail_read) throw std::runtime_error("USB disconnected");
    return drop_samples ? std::nullopt : sim.read_status();
  }
  bool is_connected() const override { return sim.is_connected(); }
  bool fault_latched() const override { return sim.fault_latched(); }
  bool stop_confirmed() const override { return sim.stop_confirmed(); }
  const std::string& fault_reason() const override { return sim.fault_reason(); }
  fss_hal::SimReactionWheelHal sim;
  std::vector<double> writes;
  int stops{0};
  bool drop_samples{false}, fail_read{false}, fail_stop{false};
};

TEST(ReactionWheelControl, EnforcesCommandRateAndKeepsTheLatestCommand) {
  fss_hal::ReactionWheelConfig config;
  double now = 1.0;
  ControlledWheel hal(config, [&] { return now; });
  fss_actuators::ReactionWheelControl control(hal, config, [&] { return now; });
  hal.connect(); control.step(); control.enable(); control.command(0.002); control.step();
  now += 0.01; control.command(0.003); control.step();
  EXPECT_EQ(hal.writes, std::vector<double>{0.002});
  now += 0.05; control.step();
  EXPECT_EQ(hal.writes, (std::vector<double>{0.002, 0.003}));
}

TEST(ReactionWheelControl, TelemetrySilenceCannotBeHiddenByAFreshCommand) {
  fss_hal::ReactionWheelConfig config;
  double now = 1.0;
  ControlledWheel hal(config, [&] { return now; });
  fss_actuators::ReactionWheelControl control(hal, config, [&] { return now; });
  hal.connect(); control.step(); control.enable(); control.command(0.002); control.step();
  hal.drop_samples = true;
  now += 0.11; control.command(0.003); control.step();
  EXPECT_FALSE(control.enabled());
  EXPECT_NE(control.last_error().find("telemetry timeout"), std::string::npos);
  EXPECT_THROW(control.enable(), std::runtime_error);
  EXPECT_EQ(hal.writes, std::vector<double>{0.002});
}

TEST(ReactionWheelControl, ReadErrorsDisableBeforePropagatingAndRetainStopFailure) {
  fss_hal::ReactionWheelConfig config;
  double now = 1.0;
  ControlledWheel hal(config, [&] { return now; });
  fss_actuators::ReactionWheelControl control(hal, config, [&] { return now; });
  hal.connect(); control.step(); control.enable();
  hal.fail_read = hal.fail_stop = true;
  EXPECT_THROW(control.step(), std::runtime_error);
  EXPECT_FALSE(control.enabled());
  EXPECT_NE(control.last_error().find("USB disconnected"), std::string::npos);
  EXPECT_NE(control.last_error().find("STOP write failed"), std::string::npos);
  EXPECT_EQ(hal.stops, 1);
}

TEST(ReactionWheelControl, EnablingWithoutACommandStillExpires) {
  fss_hal::ReactionWheelConfig config;
  double now = 1.0;
  ControlledWheel hal(config, [&] { return now; });
  fss_actuators::ReactionWheelControl control(hal, config, [&] { return now; });
  hal.connect(); control.step(); control.enable();
  for (int i = 0; i < 5; ++i) { now += 0.06; control.step(); }
  EXPECT_FALSE(control.enabled());
  EXPECT_NE(control.last_error().find("command timeout"), std::string::npos);
  EXPECT_TRUE(hal.writes.empty());
  EXPECT_EQ(hal.sim.read_status()->state, 0);
}
}  // namespace
