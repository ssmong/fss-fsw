#pragma once
// LQR 제어기 — 사전계산 게인 로드 + 온라인 행렬-벡터 곱
// K 매트릭스는 scripts/compute_lqr_gains.py (scipy CARE)로 계산

#include "fss_gnc/controller_interface.hpp"
#include <array>
#include <cmath>
#include <algorithm>

namespace fss::gnc {

// 병진 LQR: K_trans (2×4) → [Fx, Fy]_table = -K @ [ex, ey, evx, evy]_table
class LQRTranslational : public TranslationalController {
public:
    // K 매트릭스 (2행 4열, row-major)
    std::array<double, 8> K = {};
    double max_force = 5.0;

    void set_gain(const std::array<double, 8>& gain, double max_f) {
        K = gain;
        max_force = max_f;
    }

    TranslationalOutput compute(
        const TranslationalState& state,
        const TranslationalRef& ref,
        double /*dt*/) override
    {
        // 관성 프레임 정식화: 위치/속도 오차 모두 테이블 프레임에서 직접 계산한다.
        // 더블 적분기 모델로 계산된 K도 테이블 프레임 기준이므로 회전이 불필요하다.
        // 출력 [Fx, Fy]는 테이블 프레임이며, 본체 회전은 gnc_node가 할당 직전 수행한다.
        // e = state - ref (표준 LQR u = -K·e; ref-state로 두면 부호가 뒤집혀 발산)
        double e[4] = {
            state.x - ref.x,     // 테이블 X 위치 오차
            state.y - ref.y,     // 테이블 Y 위치 오차
            state.vx - ref.vx,   // 테이블 X 속도 오차
            state.vy - ref.vy,   // 테이블 Y 속도 오차
        };

        // u = -K @ e
        double fx = 0.0, fy = 0.0;
        for (int j = 0; j < 4; ++j) {
            fx -= K[0 * 4 + j] * e[j];
            fy -= K[1 * 4 + j] * e[j];
        }

        return {
            std::clamp(fx, -max_force, max_force),
            std::clamp(fy, -max_force, max_force),
        };
    }

    void reset() override {}
};

// 회전 LQR: K_rot (1×2) → Mz = -K @ [eθ, eω]
class LQRRotational : public RotationalController {
public:
    std::array<double, 2> K = {};
    double max_torque = 2.0;

    void set_gain(const std::array<double, 2>& gain, double max_t) {
        K = gain;
        max_torque = max_t;
    }

    double compute(
        const RotationalState& state,
        const RotationalRef& ref,
        double /*dt*/) override
    {
        // e = state - ref (표준 LQR u = -K·e)
        double eth = std::remainder(state.theta - ref.theta, 2.0 * M_PI);
        double eom = state.omega - ref.omega;

        double mz = -(K[0] * eth + K[1] * eom);
        return std::clamp(mz, -max_torque, max_torque);
    }

    void reset() override {}
};

}  // namespace fss::gnc
