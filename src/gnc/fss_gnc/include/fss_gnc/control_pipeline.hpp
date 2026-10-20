#pragma once

#include "fss_gnc/guidance.hpp"
#include "fss_gnc/lqr_controller.hpp"
#include "fss_gnc/pid_controller.hpp"
#include "fss_gnc/qp_allocator.hpp"
#include "fss_gnc/runtime_config.hpp"

#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace fss::gnc {

struct ControllerConfig {
    std::string type;
    double kp{}, ki{}, kd{}, limit{};

    void validate() const {
        if ((type != "pid" && type != "lqr") || !std::isfinite(kp) || !std::isfinite(ki) ||
            !std::isfinite(kd) || !std::isfinite(limit) || limit <= 0)
            throw std::invalid_argument("controller requires pid/lqr, finite gains and positive limit");
    }
};

struct ControlResult {
    Reference reference;
    std::array<double, 8> thrust_n{};
};

class ControlPipeline {
public:
    ControlPipeline(ControllerConfig translation, ControllerConfig rotation, GuidanceManager guidance,
                    std::filesystem::path config_directory, const std::array<double, 8>& limits,
                    const std::array<double, 24>& matrix, double weight, double minimum)
        : translation_(std::move(translation)), rotation_(std::move(rotation)),
          guidance_(std::move(guidance)), config_directory_(std::move(config_directory)), limits_(limits) {
        translation_.validate();
        rotation_.validate();
        if (!std::isfinite(weight) || weight < 0 || !std::isfinite(minimum) || minimum < 0 || minimum > 1)
            throw std::invalid_argument("allocator requires nonnegative weight and minimum in [0,1]");
        for (double limit : limits_)
            if (!std::isfinite(limit) || limit <= 0)
                throw std::invalid_argument("nozzle limits must be positive and finite");
        for (double value : matrix)
            if (!std::isfinite(value))
                throw std::invalid_argument("allocation matrix must be finite");
        trans_controller_ = make_translation(translation_.type);
        rot_controller_ = make_rotation(rotation_.type);
        allocator_.thrust_weight = weight;
        allocator_.min_thrust = minimum;
        allocator_.set_B_matrix(matrix.data());
    }

    GuidanceManager& guidance() { return guidance_; }

    void reset() {
        trans_controller_->reset();
        rot_controller_->reset();
    }

    void set_controller(const std::string& type) {
        // Build both first: a missing gain file must leave the active pair intact.
        auto translation = make_translation(type);
        auto rotation = make_rotation(type);
        trans_controller_ = std::move(translation);
        rot_controller_ = std::move(rotation);
        translation_.type = type;
        rotation_.type = type;
    }

    ControlResult compute(const TranslationalState& state, double omega, double time, double dt) {
        if (!std::isfinite(state.x) || !std::isfinite(state.y) || !std::isfinite(state.vx) ||
            !std::isfinite(state.vy) || !std::isfinite(state.theta) || !std::isfinite(omega) ||
            !std::isfinite(time) || !std::isfinite(dt) || dt <= 0) {
            reset();
            throw std::invalid_argument("control state and positive timestep must be finite");
        }
        ControlResult result;
        result.reference = guidance_.compute(time);
        const auto& ref = result.reference;
        if (!GuidanceManager::finite(ref)) {
            reset();
            throw std::runtime_error("guidance produced a nonfinite reference");
        }
        const auto force = trans_controller_->compute(state, {ref.x, ref.y, ref.vx, ref.vy}, dt);
        const double torque = rot_controller_->compute({state.theta, omega}, {ref.theta, ref.omega}, dt);
        if (!std::isfinite(force.fx) || !std::isfinite(force.fy) || !std::isfinite(torque)) {
            reset();
            throw std::runtime_error("controller produced a nonfinite wrench");
        }
        // Controllers output table-frame force; body-fixed geometry needs one inverse rotation.
        const double c = std::cos(state.theta), s = std::sin(state.theta);
        const double fx = std::clamp(c * force.fx + s * force.fy, -translation_.limit, translation_.limit);
        const double fy = std::clamp(-s * force.fx + c * force.fy, -translation_.limit, translation_.limit);
        const auto allocated = allocator_.allocate(fx, fy, torque);
        for (std::size_t i = 0; i < result.thrust_n.size(); ++i) {
            if (!std::isfinite(allocated.duty[i])) {
                reset();
                throw std::runtime_error("allocator produced a nonfinite command");
            }
            result.thrust_n[i] = std::clamp(allocated.duty[i], 0.0, 1.0) * limits_[i];
        }
        return result;
    }

private:
    std::unique_ptr<TranslationalController> make_translation(const std::string& type) const {
        if (type == "pid") {
            auto controller = std::make_unique<PIDTranslational>();
            controller->set_gains(translation_.kp, translation_.ki, translation_.kd, translation_.limit);
            return controller;
        }
        if (type == "lqr") {
            auto controller = std::make_unique<LQRTranslational>();
            controller->set_gain(load_lqr_gains<8>(config_directory_, "K_trans"), translation_.limit);
            return controller;
        }
        throw std::invalid_argument("controller type must be pid or lqr");
    }

    std::unique_ptr<RotationalController> make_rotation(const std::string& type) const {
        if (type == "pid") {
            auto controller = std::make_unique<PIDRotational>();
            controller->set_gains(rotation_.kp, rotation_.ki, rotation_.kd, rotation_.limit);
            return controller;
        }
        if (type == "lqr") {
            auto controller = std::make_unique<LQRRotational>();
            controller->set_gain(load_lqr_gains<2>(config_directory_, "K_rot"), rotation_.limit);
            return controller;
        }
        throw std::invalid_argument("controller type must be pid or lqr");
    }

    ControllerConfig translation_, rotation_;
    GuidanceManager guidance_;
    std::filesystem::path config_directory_;
    std::array<double, 8> limits_;
    std::unique_ptr<TranslationalController> trans_controller_;
    std::unique_ptr<RotationalController> rot_controller_;
    QPAllocator allocator_;
};

}  // namespace fss::gnc
