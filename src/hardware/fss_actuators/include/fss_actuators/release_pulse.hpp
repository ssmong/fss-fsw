#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace fss_actuators {

class ReleasePulse {
 public:
  using Event = std::function<void(std::int64_t, const char*, const std::string&)>;
  ReleasePulse(std::function<void(bool)> write, Event event)
      : write_(std::move(write)), event_(std::move(event)) {}

  void request(std::int64_t stamp, std::int64_t now_ns, double steady_now,
               bool active, double mode_received, bool valve_busy) {
    if (stamp <= last_stamp_) return;
    if (stamp > now_ns || now_ns - stamp > 100000000) {
      event_(stamp, "REJECTED", "release stamp outside 100 ms window");
      return;
    }
    if (!active || steady_now - mode_received > 1.0 || end_ || valve_busy) {
      event_(stamp, "REJECTED", "fresh ACTIVE and idle valve required");
      return;
    }
    last_stamp_ = stamp;
    end_ = steady_now + 0.010;
    // OFF remains pending even when the ON response is lost.
    try { write_(true); }
    catch (const std::exception& error) {
      end_ = 0.0;
      event_(stamp, "ON_UNCONFIRMED", error.what());
      close();
      return;
    }
    event_(stamp, "ON_ACK", "PLC write confirmed");
  }

  void close() {
    if (!end_) return;
    try { write_(false); }
    catch (const std::exception& error) {
      end_ = 0.0;
      event_(last_stamp_, "OFF_UNCONFIRMED", error.what());
      return;
    }
    end_.reset();
    event_(last_stamp_, "OFF_ACK", "PLC write confirmed");
  }

  bool due(double now) const { return end_ && now >= *end_; }
  bool pending() const { return end_.has_value(); }

 private:
  std::function<void(bool)> write_;
  Event event_;
  std::int64_t last_stamp_{0};
  std::optional<double> end_;
};
}  // namespace fss_actuators
