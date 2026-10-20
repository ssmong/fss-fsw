#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <deque>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <utility>

#include <fss_hal/vicon_hal.hpp>

namespace {
std::deque<std::pair<std::string, std::string>> packets;
bool inject_packets = false;
int receive_error = EAGAIN;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

std::string packet(int frame, double time, const std::string& object = R"({"valid":true,"x":1000,"y":2000,"yaw":90})") {
  return R"({"frame":)" + std::to_string(frame) + R"(,"t":)" + std::to_string(time) +
         R"(,"objects":{"robotA":)" + object + "}}";
}

void feed(const std::string& data, const std::string& source = "127.0.0.1") {
  packets.emplace_back(data, source);
}

std::size_t open_descriptors() {
  return static_cast<std::size_t>(std::distance(std::filesystem::directory_iterator("/proc/self/fd"),
                                               std::filesystem::directory_iterator{}));
}

void test_udp_lifecycle_and_read_budget() {
  const int reserve = ::socket(AF_INET, SOCK_DGRAM, 0);
  require(reserve >= 0, "Could not reserve localhost UDP port");
  sockaddr_in address{};
  address.sin_family = AF_INET;
  inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
  if (::bind(reserve, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
    ::close(reserve);
    throw std::runtime_error("Could not bind localhost UDP port");
  }
  socklen_t size = sizeof(address);
  require(::getsockname(reserve, reinterpret_cast<sockaddr*>(&address), &size) == 0,
          "Could not discover localhost UDP port");
  fss_hal::ViconHAL hal({"127.0.0.1", ntohs(address.sin_port), "127.0.0.1", "robotA"});
  const auto before_failure = open_descriptors();
  bool failed = false;
  try { hal.connect(); } catch (const std::exception&) { failed = true; }
  ::close(reserve);
  require(failed, "Vicon accepted a port already in use");
  require(open_descriptors() + 1 == before_failure, "Failed Vicon connect leaked a socket");
  hal.connect();
  const auto connected = open_descriptors();
  hal.connect();
  require(open_descriptors() == connected, "Repeated Vicon connect opened an extra socket");
  inject_packets = true;
  feed(packet(1, 100)); feed(packet(3, 100.02)); feed(packet(2, 100.01));
  require(hal.read()->frame_number == 3, "UDP queue did not return newest pose");
  require(!hal.read(), "Empty UDP queue re-emitted an old pose");
  feed(packet(4, 100.03)); feed(packet(5, 100.04, R"({"valid":false})"));
  require(!hal.read(), "Newest lost target retained an earlier valid UDP pose");
  feed(packet(6, 100.05)); feed("partial JSON");
  require(!hal.read(), "Malformed newest UDP data retained an earlier valid pose");
  feed(packet(7, 100.06)); feed(packet(8, 100.07, R"({"valid":false})"), "127.0.0.2");
  require(hal.read()->frame_number == 7, "Filtered UDP source changed the latest pose");
  receive_error = EIO;
  feed(packet(8, 100.07));
  require(!hal.read(), "UDP receive failure returned an earlier queued pose");
  receive_error = EAGAIN;
  for (int frame = 9; frame < 266; ++frame) feed(packet(frame, 100.0 + frame));
  require(!hal.read() && packets.size() == 1, "UDP read budget published known backlog");
  require(hal.read()->frame_number == 265, "Next UDP poll failed to drain remaining backlog");
  hal.disconnect();
  hal.disconnect();
  require(open_descriptors() + 1 == connected, "Vicon disconnect leaked a socket");
  hal.connect();
  feed(packet(1, 100));
  require(hal.read()->frame_number == 1, "Reconnect retained old source ordering state");
  hal.disconnect();
  inject_packets = false;
}
}  // namespace

extern "C" ssize_t __real_recvfrom(int, void*, std::size_t, int, sockaddr*, socklen_t*);

// Linker injection exercises the real HAL's bounded queue loop without UDP queue-size races.
extern "C" ssize_t __wrap_recvfrom(int fd, void* data, std::size_t size, int flags,
                                   sockaddr* sender, socklen_t* sender_size) {
  if (!inject_packets) return __real_recvfrom(fd, data, size, flags, sender, sender_size);
  if (packets.empty()) { errno = receive_error; return -1; }
  const auto item = std::move(packets.front());
  packets.pop_front();
  if (item.first.size() > size || *sender_size < sizeof(sockaddr_in)) { errno = EMSGSIZE; return -1; }
  std::memcpy(data, item.first.data(), item.first.size());
  auto* address = reinterpret_cast<sockaddr_in*>(sender);
  *address = {};
  address->sin_family = AF_INET;
  inet_pton(AF_INET, item.second.c_str(), &address->sin_addr);
  *sender_size = sizeof(sockaddr_in);
  return static_cast<ssize_t>(item.first.size());
}

int main() {
  try {
    test_udp_lifecycle_and_read_budget();
    std::cout << "C++ Vicon UDP lifecycle and queue checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
