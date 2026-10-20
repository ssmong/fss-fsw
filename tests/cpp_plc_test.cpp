#include "fss_hal/plc_hal.hpp"

#include <gtest/gtest.h>
#include <modbus.h>

#include <atomic>
#include <cerrno>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {
class FakeModbus final : public fss_hal::ModbusTransport {
 public:
  void connect() override { ++connections; open = true; }
  void close() noexcept override { open = false; }
  void write_registers(int address, const std::vector<std::uint16_t>& values) override {
    valve_address = address;
    valve_writes.push_back(values);
    if (fail_valves) throw std::runtime_error("FC16 protocol exception");
  }
  void write_register(int address, std::uint16_t value) override {
    main_address = address;
    main_writes.push_back(value);
    if (fail_main) throw std::runtime_error("FC06 transport timeout");
  }
  std::vector<bool> read_discrete_inputs(int address, int count) override {
    EXPECT_EQ(address, 0); EXPECT_EQ(count, 8);
    if (fail_inputs) throw std::runtime_error("FC02 protocol exception");
    return {true, false, true, false, true, false, true, false};
  }
  int connections{0}, valve_address{-1}, main_address{-1};
  bool open{false}, fail_valves{false}, fail_main{false}, fail_inputs{false};
  std::vector<std::vector<std::uint16_t>> valve_writes;
  std::vector<std::uint16_t> main_writes;
};

TEST(PlcHal, ClearsBothOutputsOnConnectAndDisconnect) {
  auto transport = std::make_unique<FakeModbus>();
  auto* fake = transport.get();
  fss_hal::PlcHal hal({}, std::move(transport));
  hal.connect();
  ASSERT_TRUE(hal.is_connected());
  ASSERT_EQ(fake->valve_writes.size(), 1u);
  EXPECT_EQ(fake->valve_writes.back(), std::vector<std::uint16_t>(8, 0));
  EXPECT_EQ(fake->main_writes, std::vector<std::uint16_t>({0}));
  hal.set_valves(std::vector<double>(8, 0.5));
  hal.set_main_valve(true);
  hal.disconnect();
  EXPECT_FALSE(fake->open);
  EXPECT_FALSE(hal.is_connected());
  EXPECT_EQ(fake->valve_writes.back(), std::vector<std::uint16_t>(8, 0));
  EXPECT_EQ(fake->main_writes.back(), 0);
}

TEST(PlcHal, EmergencyStopAttemptsBothOutputsForEveryFailureCombination) {
  for (bool valves_fail : {false, true}) for (bool main_fails : {false, true}) {
    auto transport = std::make_unique<FakeModbus>();
    auto* fake = transport.get();
    fss_hal::PlcHal hal({}, std::move(transport));
    hal.connect();
    fake->valve_writes.clear(); fake->main_writes.clear();
    fake->fail_valves = valves_fail; fake->fail_main = main_fails;
    if (valves_fail || main_fails) EXPECT_THROW(hal.emergency_stop(), std::runtime_error);
    else EXPECT_NO_THROW(hal.emergency_stop());
    EXPECT_EQ(fake->valve_writes.size(), 1u);
    EXPECT_EQ(fake->main_writes.size(), 1u);
    EXPECT_EQ(fake->valve_writes.front(), std::vector<std::uint16_t>(8, 0));
    EXPECT_EQ(fake->main_writes.front(), 0);
    EXPECT_EQ(hal.fault_latched(), valves_fail || main_fails);
  }
}

TEST(PlcHal, ClampsQuantizesAndBatchesAllRegisters) {
  auto transport = std::make_unique<FakeModbus>();
  auto* fake = transport.get();
  fss_hal::PlcHal hal({}, std::move(transport));
  hal.connect();
  hal.set_valves({0.1234, 1.5, -0.5, std::nan(""), INFINITY, -INFINITY, 1.0, 0.0});
  EXPECT_EQ(fake->valve_address, 100);
  EXPECT_EQ(fake->valve_writes.back(), (std::vector<std::uint16_t>{493, 4000, 0, 0, 0, 0, 4000, 0}));
  EXPECT_THROW(hal.set_valves({0.5}), std::invalid_argument);
  EXPECT_EQ(fake->valve_writes.size(), 2u);
}

TEST(PlcHal, FailedWriteCannotReconnectOrReplayTorque) {
  auto transport = std::make_unique<FakeModbus>();
  auto* fake = transport.get();
  fss_hal::PlcHal hal({}, std::move(transport));
  hal.connect();
  fake->fail_valves = true;
  EXPECT_THROW(hal.set_valves(std::vector<double>(8, 0.5)), std::runtime_error);
  fake->fail_valves = false;
  const auto sent = fake->valve_writes.size();
  EXPECT_THROW(hal.set_valves(std::vector<double>(8, 0.5)), std::runtime_error);
  EXPECT_THROW(hal.set_main_valve(true), std::runtime_error);
  EXPECT_NO_THROW(hal.emergency_stop());
  EXPECT_EQ(fake->valve_writes.size(), sent + 1);
  EXPECT_EQ(fake->connections, 1);
  EXPECT_TRUE(hal.fault_latched());
}

TEST(PlcHal, ConnectStopFailureClosesSocketAndInputFailureReturnsNoSample) {
  auto transport = std::make_unique<FakeModbus>();
  auto* fake = transport.get();
  fake->fail_valves = true;
  fss_hal::PlcHal hal({}, std::move(transport));
  EXPECT_THROW(hal.connect(), std::runtime_error);
  EXPECT_FALSE(fake->open);
  EXPECT_FALSE(hal.is_connected());
  EXPECT_EQ(fake->main_writes.size(), 1u);
  fake->fail_valves = false;
  hal.connect();
  EXPECT_EQ(*hal.read_digital_inputs(), (std::vector<bool>{true, false, true, false, true, false, true, false}));
  fake->fail_inputs = true;
  EXPECT_FALSE(hal.read_digital_inputs());
}

TEST(PlcHal, RejectsInvalidConfigurationBeforeConnecting) {
  for (int field = 0; field < 7; ++field) {
    fss_hal::PlcConfig config;
    if (field == 0) config.timeout_sec = std::nan("");
    if (field == 1) config.valve_count = 0;
    if (field == 2) config.valve_count = 124;
    if (field == 3) config.valve_register_start = 65535;
    if (field == 4) config.main_valve_register = 100;
    if (field == 5) config.max_value = 65536;
    if (field == 6) config.port = -1;
    EXPECT_THROW(fss_hal::PlcHal hal(config), std::invalid_argument);
  }
}

// This local libmodbus server checks the real TCP transport's protocol-error path.
class LoopbackPlc {
 public:
  LoopbackPlc() {
    context_ = modbus_new_tcp("127.0.0.1", 0);
    mapping_ = modbus_mapping_new(0, 8, 108, 0);
    listener_ = modbus_tcp_listen(context_, 1);
    if (!context_ || !mapping_ || listener_ < 0) throw std::runtime_error("Loopback PLC setup failed");
    sockaddr_in address{};
    socklen_t size = sizeof(address);
    if (getsockname(listener_, reinterpret_cast<sockaddr*>(&address), &size) < 0)
      throw std::runtime_error("Loopback PLC port lookup failed");
    port = ntohs(address.sin_port);
    worker_ = std::thread([this] {
      if (modbus_tcp_accept(context_, &listener_) < 0) return;
      client_ = modbus_get_socket(context_);
      std::uint8_t request[MODBUS_TCP_MAX_ADU_LENGTH];
      for (;;) {
        const int count = modbus_receive(context_, request);
        if (count <= 0) break;
        const int function = request[modbus_get_header_length(context_)];
        last_unit = request[6];
        if (function == 16) ++fc16_count;
        if (function == 6) ++fc06_count;
        if (function == 16 && reject_valves)
          modbus_reply_exception(context_, request, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        else modbus_reply(context_, request, count, mapping_);
      }
    });
  }
  ~LoopbackPlc() {
    const int client = client_.load();
    if (client >= 0) shutdown(client, SHUT_RDWR);
    shutdown(listener_, SHUT_RDWR);
    ::close(listener_);
    if (worker_.joinable()) worker_.join();
    modbus_close(context_);
    modbus_free(context_);
    modbus_mapping_free(mapping_);
  }
  int port{0};
  std::atomic<bool> reject_valves{false};
  std::atomic<int> fc16_count{0}, fc06_count{0};
  std::atomic<int> last_unit{-1};
 private:
  modbus_t* context_{nullptr};
  modbus_mapping_t* mapping_{nullptr};
  int listener_{-1};
  std::atomic<int> client_{-1};
  std::thread worker_;
};

TEST(PlcTransport, RealTcpProtocolExceptionStillAttemptsMainValveClose) {
  LoopbackPlc server;
  fss_hal::PlcConfig config;
  config.ip = "127.0.0.1"; config.port = server.port; config.timeout_sec = 0.1;
  fss_hal::PlcHal hal(config);
  hal.connect();
  hal.set_valves(std::vector<double>(8, 0.1234));
  EXPECT_EQ(server.fc16_count, 2);
  EXPECT_EQ(server.last_unit, 1);
  server.reject_valves = true;
  EXPECT_THROW(hal.set_valves(std::vector<double>(8, 0.5)), std::runtime_error);
  const int main_writes = server.fc06_count;
  EXPECT_THROW(hal.emergency_stop(), std::runtime_error);
  EXPECT_EQ(server.fc06_count, main_writes + 1);
  EXPECT_TRUE(hal.fault_latched());
}

TEST(PlcTransport, RepeatedDisconnectAndDestructionCannotCloseAReusedDescriptor) {
  LoopbackPlc server;
  fss_hal::PlcConfig config;
  config.ip = "127.0.0.1"; config.port = server.port; config.timeout_sec = 0.1;
  auto hal = std::make_unique<fss_hal::PlcHal>(config);
  hal->connect(); hal->disconnect(); hal->disconnect();
  const int reused = ::open("/dev/null", O_RDONLY | O_CLOEXEC);
  ASSERT_GE(reused, 0);
  hal.reset();
  EXPECT_NE(fcntl(reused, F_GETFD), -1);
  ::close(reused);
}
}  // namespace
