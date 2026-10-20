#pragma once
// 제어기 추상 인터페이스 — 병진/회전 독립 교체 가능

#include "fss_gnc/types.hpp"

namespace fss::gnc {

// 병진 제어기: (x,y,vx,vy) 오차 → (Fx, Fy)
class TranslationalController {
public:
    virtual ~TranslationalController() = default;
    virtual TranslationalOutput compute(
        const TranslationalState& state,
        const TranslationalRef& ref,
        double dt) = 0;
    virtual void reset() = 0;
};

// 회전 제어기: (θ, ω) 오차 → Mz
class RotationalController {
public:
    virtual ~RotationalController() = default;
    virtual double compute(
        const RotationalState& state,
        const RotationalRef& ref,
        double dt) = 0;
    virtual void reset() = 0;
};

}  // namespace fss::gnc
