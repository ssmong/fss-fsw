#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fss::gnc {

struct Reference {
    double x{}, y{}, theta{}, vx{}, vy{}, omega{};
};

struct TrajectoryPoint {
    double t{}, x{}, y{}, theta{};
};

class GuidanceManager {
public:
    GuidanceManager(std::string mode, Reference target, std::vector<TrajectoryPoint> points = {})
        : target_(target), points_(std::move(points)) {
        if (!finite(target_))
            throw std::invalid_argument("guidance target must be finite");
        for (std::size_t i = 0; i < points_.size(); ++i) {
            const auto& p = points_[i];
            if (!std::isfinite(p.t) || !std::isfinite(p.x) || !std::isfinite(p.y) ||
                !std::isfinite(p.theta) || (i && p.t <= points_[i - 1].t))
                throw std::invalid_argument("trajectory must be finite with strictly increasing time");
        }
        set_mode(mode);
    }

    bool has_trajectory() const { return points_.size() >= 2; }
    const std::string& mode() const { return mode_; }

    void set_mode(const std::string& mode) {
        if (mode != "target" && mode != "trajectory")
            throw std::invalid_argument("guidance.mode must be target or trajectory");
        if (mode == "trajectory" && !has_trajectory())
            throw std::invalid_argument("trajectory mode requires at least two loaded points");
        mode_ = mode;
    }

    Reference compute(double t) const {
        if (!std::isfinite(t))
            throw std::invalid_argument("trajectory time must be finite");
        if (mode_ == "target")
            return target_;
        if (t <= points_.front().t)
            return held(points_.front());
        if (t >= points_.back().t)
            return held(points_.back());

        // Match numpy.searchsorted's left segment at an exact interior waypoint.
        const auto upper = std::lower_bound(points_.begin(), points_.end(), t,
            [](const TrajectoryPoint& point, double time) { return point.t < time; });
        const auto& a = *(upper - 1);
        const auto& b = *upper;
        const double duration = b.t - a.t;
        const double alpha = (t - a.t) / duration;
        const double dtheta = std::remainder(b.theta - a.theta, 2.0 * std::acos(-1.0));
        return {a.x + alpha * (b.x - a.x), a.y + alpha * (b.y - a.y),
                a.theta + alpha * dtheta, (b.x - a.x) / duration,
                (b.y - a.y) / duration, dtheta / duration};
    }

    static bool finite(const Reference& ref) {
        return std::isfinite(ref.x) && std::isfinite(ref.y) && std::isfinite(ref.theta) &&
               std::isfinite(ref.vx) && std::isfinite(ref.vy) && std::isfinite(ref.omega);
    }

private:
    static Reference held(const TrajectoryPoint& p) { return {p.x, p.y, p.theta, 0, 0, 0}; }
    std::string mode_;
    Reference target_;
    std::vector<TrajectoryPoint> points_;
};

}  // namespace fss::gnc
