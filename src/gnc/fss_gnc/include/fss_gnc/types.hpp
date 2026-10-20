#pragma once
// 3-DOF 평면 GNC 공용 타입 정의
// 인덱스 실수 방지를 위해 명시적 struct 사용

namespace fss::gnc {

// 병진 상태/기준 (테이블 프레임 위치 + 속도, 관성 프레임 정식화)
struct TranslationalState {
    double x;      // 위치 X [m] (테이블)
    double y;      // 위치 Y [m] (테이블)
    double vx;     // 속도 X [m/s] (테이블)
    double vy;     // 속도 Y [m/s] (테이블)
    double theta;  // 방향각 [rad] (관성 정식화에서는 미사용, 호환 위해 유지)
};

struct TranslationalRef {
    double x;
    double y;
    double vx;
    double vy;
};

// 회전 상태/기준
struct RotationalState {
    double theta;  // 방향각 [rad]
    double omega;  // 각속도 [rad/s]
};

struct RotationalRef {
    double theta;
    double omega;
};

// 제어기 출력
struct TranslationalOutput {
    double fx;  // 힘 X [N]
    double fy;  // 힘 Y [N]
};

// 할당기 출력
struct AllocatorOutput {
    double duty[8];  // 밸브 듀티 [0, 1]
};

}  // namespace fss::gnc
