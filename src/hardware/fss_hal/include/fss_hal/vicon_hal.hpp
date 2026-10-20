#pragma once

#include <chrono>
#include <functional>
#include <optional>
#include <string>
#include <string_view>

#include "fss_hal/pose_sensor.hpp"

namespace fss_hal {

struct ViconConfig {
  std::string bind_ip;
  int port{};
  std::string source_ip;
  std::string object_name;
};

// A poll's newest invalid/malformed packet clears its earlier valid pose.
class ViconPacketDecoder {
 public:
  explicit ViconPacketDecoder(std::string object_name);
  void begin_poll() noexcept { latest_.reset(); }
  void consume(std::string_view packet);
  const std::optional<PlanarPoseSample>& latest() const noexcept { return latest_; }
  void reset() noexcept;

 private:
  std::string object_name_;
  std::optional<std::uint64_t> last_frame_;
  std::optional<double> last_time_;
  std::optional<PlanarPoseSample> latest_;
};

class ViconHAL final : public PoseSensorHAL {
 public:
  explicit ViconHAL(ViconConfig config,
                    std::function<void(const std::string&)> warn = {});
  ~ViconHAL() override;
  ViconHAL(const ViconHAL&) = delete;
  ViconHAL& operator=(const ViconHAL&) = delete;
  void connect() override;
  void disconnect() noexcept override;
  std::optional<PlanarPoseSample> read() override;

 private:
  void warn(const std::string& message);
  ViconConfig config_;
  ViconPacketDecoder decoder_;
  std::function<void(const std::string&)> warning_;
  int socket_{-1};
  std::optional<std::chrono::steady_clock::time_point> last_warning_;
};

}  // namespace fss_hal
