#pragma once
// PID 제어기 — 병진(2채널) + 회전(1채널)
// ki=0 설정 시 PD 제어기로 동작

#include "fss_gnc/controller_interface.hpp"
#include <algorithm>
#include <cmath>

namespace fss::gnc {

// 단일 채널 PID
class PIDChannel {
public:
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double integral_limit = 10.0;
    double output_limit = 5.0;

    // 적분기 상태
    double compute(double error, double error_dot, double dt) {
        // 적분 갱신 + anti-windup (clamping)
        integral_ += error * dt;
        integral_ = std::clamp(integral_, -integral_limit, integral_limit);

        double output = kp * error + ki * integral_ + kd * error_dot;
        return std::clamp(output, -output_limit, output_limit);
    }

    void reset() { integral_ = 0.0; }

private:
    double integral_ = 0.0;
};

// 병진 PID (X, Y 독립 2채널)
class PIDTranslational : public TranslationalController {
public:
    PIDChannel ch_x;
    PIDChannel ch_y;

    void set_gains(double kp, double ki, double kd, double max_force) {
        ch_x.kp = kp; ch_x.ki = ki; ch_x.kd = kd; ch_x.output_limit = max_force;
        ch_y.kp = kp; ch_y.ki = ki; ch_y.kd = kd; ch_y.output_limit = max_force;
    }

    TranslationalOutput compute(
        const TranslationalState& state,
        const TranslationalRef& ref,
        double dt) override
    {
        // 관성 프레임 정식화: 위치/속도 모두 테이블 프레임에서 직접 제어한다.
        // 출력 힘도 테이블 프레임이며, 본체 고정 추력기를 위한 회전은
        // gnc_node가 할당 직전에 1회 수행한다 (state.theta는 사용 안 함).
        double ex = ref.x - state.x;     // 테이블 프레임 위치 오차
        double ey = ref.y - state.y;
        double evx = ref.vx - state.vx;  // 테이블 프레임 속도 오차
        double evy = ref.vy - state.vy;

        return {ch_x.compute(ex, evx, dt), ch_y.compute(ey, evy, dt)};
    }

    void reset() override { ch_x.reset(); ch_y.reset(); }
};

// 회전 PID (θ 단일 채널)
class PIDRotational : public RotationalController {
public:
    PIDChannel ch_theta;

    void set_gains(double kp, double ki, double kd, double max_torque) {
        ch_theta.kp = kp; ch_theta.ki = ki; ch_theta.kd = kd;
        ch_theta.output_limit = max_torque;
    }

    double compute(
        const RotationalState& state,
        const RotationalRef& ref,
        double dt) override
    {
        // 각도 오차 (wrapping)
        double eth = ref.theta - state.theta;
        eth = std::remainder(eth, 2.0 * M_PI);  // [-π, π] 정규화
        double eom = ref.omega - state.omega;

        return ch_theta.compute(eth, eom, dt);
    }

    void reset() override { ch_theta.reset(); }
};

}  // namespace fss::gnc
