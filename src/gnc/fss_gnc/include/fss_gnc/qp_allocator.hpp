#pragma once
// QP 기반 추력 할당기
// min ||Bu - w||² + λ||u||²  s.t. 0 ≤ u ≤ 1
//
// pseudo-inverse로 초기화 후 projected gradient descent로 box 제약을 강제한다.
// (필요 시 외부 QP 솔버로 교체 가능 — 현재는 외부 의존성 없는 내장 구현)

#include "fss_gnc/types.hpp"
#include <array>
#include <cmath>
#include <algorithm>
#include <cstring>

namespace fss::gnc {

class QPAllocator {
public:
    static constexpr int N_THRUSTERS = 8;
    static constexpr int N_WRENCH = 3;

    // B 행렬 (3×8, row-major)
    double B[N_WRENCH * N_THRUSTERS] = {};
    double thrust_weight = 0.01;
    double min_thrust = 0.0;  // 데드밴드 근사: 0.5로 설정 가능
    double max_thrust = 1.0;
    int max_iter = 5000;

    void set_B_matrix(const double b[N_WRENCH * N_THRUSTERS]) {
        std::memcpy(B, b, sizeof(B));
        _compute_pseudoinverse();
    }

    AllocatorOutput allocate(double fx, double fy, double mz) {
        double w[N_WRENCH] = {fx, fy, mz};
        AllocatorOutput out;

        _solve_pgd(w, out.duty);
        return out;
    }

private:
    // B의 pseudo-inverse (3×8 → 8×3, B^+ = Bᵀ(BBᵀ)⁻¹)
    double Bpinv_[N_THRUSTERS * N_WRENCH] = {};

    void _compute_pseudoinverse() {
        // BBᵀ (3×3)
        double BBt[9] = {};
        for (int i = 0; i < N_WRENCH; ++i)
            for (int j = 0; j < N_WRENCH; ++j)
                for (int k = 0; k < N_THRUSTERS; ++k)
                    BBt[i * 3 + j] += B[i * N_THRUSTERS + k] * B[j * N_THRUSTERS + k];

        // 정규화 (특이행렬 방지)
        for (int i = 0; i < 3; ++i)
            BBt[i * 3 + i] += 1e-8;

        // (BBᵀ)⁻¹ (3×3 직접 역행렬)
        double inv[9];
        _invert3x3(BBt, inv);

        // B^+ = Bᵀ · (BBᵀ)⁻¹
        for (int i = 0; i < N_THRUSTERS; ++i)
            for (int j = 0; j < N_WRENCH; ++j) {
                double sum = 0.0;
                for (int k = 0; k < N_WRENCH; ++k)
                    sum += B[k * N_THRUSTERS + i] * inv[k * 3 + j];
                Bpinv_[i * N_WRENCH + j] = sum;
            }
    }

    static void _invert3x3(const double m[9], double inv[9]) {
        // 3×3 역행렬 (Cramer)
        double det =
            m[0] * (m[4] * m[8] - m[5] * m[7]) -
            m[1] * (m[3] * m[8] - m[5] * m[6]) +
            m[2] * (m[3] * m[7] - m[4] * m[6]);
        double id = 1.0 / (det + 1e-15);

        inv[0] = (m[4] * m[8] - m[5] * m[7]) * id;
        inv[1] = (m[2] * m[7] - m[1] * m[8]) * id;
        inv[2] = (m[1] * m[5] - m[2] * m[4]) * id;
        inv[3] = (m[5] * m[6] - m[3] * m[8]) * id;
        inv[4] = (m[0] * m[8] - m[2] * m[6]) * id;
        inv[5] = (m[2] * m[3] - m[0] * m[5]) * id;
        inv[6] = (m[3] * m[7] - m[4] * m[6]) * id;
        inv[7] = (m[1] * m[6] - m[0] * m[7]) * id;
        inv[8] = (m[0] * m[4] - m[1] * m[3]) * id;
    }

    void _solve_pgd(const double w[N_WRENCH], double u[N_THRUSTERS]) {
        // Projected Gradient Descent
        // 목적함수: f(u) = 0.5||Bu-w||² + 0.5·λ||u||²
        // 그래디언트: g = Bᵀ(Bu-w) + λu
        // 투영: clamp(u, min, max)

        // 초기값: pseudo-inverse
        _solve_pinv(w, u);

        // ||BBᵀ||∞ + λ bounds the Hessian's largest eigenvalue.
        double lipschitz = 0.0;
        for (int i = 0; i < N_WRENCH; ++i) {
            double row_sum = 0.0;
            for (int j = 0; j < N_WRENCH; ++j) {
                double bbt = 0.0;
                for (int k = 0; k < N_THRUSTERS; ++k)
                    bbt += B[i * N_THRUSTERS + k] * B[j * N_THRUSTERS + k];
                row_sum += std::abs(bbt);
            }
            lipschitz = std::max(lipschitz, row_sum);
        }
        lipschitz += thrust_weight;
        if (lipschitz <= 0.0) return;
        const double step = 1.0 / lipschitz;
        for (int iter = 0; iter < max_iter; ++iter) {
            // Bu
            double Bu[N_WRENCH] = {};
            for (int i = 0; i < N_WRENCH; ++i)
                for (int j = 0; j < N_THRUSTERS; ++j)
                    Bu[i] += B[i * N_THRUSTERS + j] * u[j];

            // 잔차 = Bu - w
            double res[N_WRENCH];
            for (int i = 0; i < N_WRENCH; ++i)
                res[i] = Bu[i] - w[i];

            // 그래디언트: Bᵀ·res + λ·u
            double grad[N_THRUSTERS];
            for (int j = 0; j < N_THRUSTERS; ++j) {
                double g = thrust_weight * u[j];
                for (int i = 0; i < N_WRENCH; ++i)
                    g += B[i * N_THRUSTERS + j] * res[i];
                grad[j] = g;
            }

            // 갱신 + 투영
            double max_change = 0.0;
            for (int j = 0; j < N_THRUSTERS; ++j) {
                const double next = std::clamp(u[j] - step * grad[j], min_thrust, max_thrust);
                max_change = std::max(max_change, std::abs(next - u[j]));
                u[j] = next;
            }

            // At active bounds the raw gradient need not vanish.
            if (max_change / step < 1e-6) return;
        }
        // shortcut: finite iteration budget; increase max_iter for ill-conditioned geometry.
    }

    void _solve_pinv(const double w[N_WRENCH], double u[N_THRUSTERS]) {
        // u = B^+ · w, clamped
        for (int i = 0; i < N_THRUSTERS; ++i) {
            double sum = 0.0;
            for (int j = 0; j < N_WRENCH; ++j)
                sum += Bpinv_[i * N_WRENCH + j] * w[j];
            u[i] = std::clamp(sum, min_thrust, max_thrust);
        }
    }
};

}  // namespace fss::gnc
