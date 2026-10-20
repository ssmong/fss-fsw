#!/usr/bin/env python3
"""LQR 게인 자동 계산 스크립트.

3-DOF 평면 우주선 모델의 연속시간 선형화 후,
scipy CARE (Continuous Algebraic Riccati Equation)를 풀어
LQR 게인 K를 계산한다.

시스템 모델 (테이블/관성 프레임, 병진은 exact 이중적분기):
  상태: [x, y, θ, vx, vy, ω]  (6D, 위치·속도 모두 테이블 프레임)
  입력: [Fx, Fy, Mz]          (3D, 병진력은 테이블 프레임)
  관성 프레임 정식화에서 병진은 회전과 무관한 이중적분기이므로
  이 모델은 θ에 대한 근사가 아니라 정확(exact)하다.

  ẋ = A·x + B·u
  A = [[0 0 0 1 0 0],    B = [[0     0     0   ],
       [0 0 0 0 1 0],         [0     0     0   ],
       [0 0 0 0 0 1],         [0     0     0   ],
       [0 0 0 0 0 0],         [1/m   0     0   ],
       [0 0 0 0 0 0],         [0     1/m   0   ],
       [0 0 0 0 0 0]]         [0     0     1/Izz]]

사용법:
  python scripts/compute_lqr_gains.py
  python scripts/compute_lqr_gains.py --mass 12.0 --izz 0.6
  python scripts/compute_lqr_gains.py --q 20,20,10,2,2,1 --r 1,1,1

결과: config/gnc/controller/lqr_gains.yaml 갱신 + 콘솔 출력
(max_force/max_torque 한계는 Hydra 트리 vehicle/mass_properties.yaml에서 관리)
"""

import argparse
import os
import sys

import numpy as np
import yaml
from scipy.linalg import solve_continuous_are


def build_system(mass: float, izz: float) -> tuple[np.ndarray, np.ndarray]:
    """3-DOF 평면 선형화 모델의 A, B 행렬을 생성한다."""
    A = np.zeros((6, 6))
    A[0, 3] = 1.0  # dx/dt = vx
    A[1, 4] = 1.0  # dy/dt = vy
    A[2, 5] = 1.0  # dθ/dt = ω

    B = np.zeros((6, 3))
    B[3, 0] = 1.0 / mass   # dvx/dt = Fx/m
    B[4, 1] = 1.0 / mass   # dvy/dt = Fy/m
    B[5, 2] = 1.0 / izz    # dω/dt = Mz/Izz

    return A, B


def compute_lqr_gain(
    A: np.ndarray,
    B: np.ndarray,
    Q: np.ndarray,
    R: np.ndarray,
) -> np.ndarray:
    """연속시간 LQR 게인 K를 계산한다. u = -K·x"""
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    return K


def split_gains(K: np.ndarray) -> tuple[list[float], list[float]]:
    """6×3 풀 게인 → 병진 K_trans(2×4) + 회전 K_rot(1×2)로 분리한다.

    K 전체: [Fx]   [K00 K01 K02 K03 K04 K05] [x ]
            [Fy] = [K10 K11 K12 K13 K14 K15] [y ]
            [Mz]   [K20 K21 K22 K23 K24 K25] [θ ]
                                               [vx]
                                               [vy]
                                               [ω ]

    병진 (테이블/관성 프레임, exact):
      K_trans (2×4) = K[0:2, [0,1,3,4]]  → [ex, ey, evx, evy]
    회전:
      K_rot (1×2)  = K[2, [2,5]]         → [eθ, eω]
    """
    K_trans = K[0:2][:, [0, 1, 3, 4]].flatten().tolist()
    K_rot = K[2, [2, 5]].tolist()
    return K_trans, K_rot


def main() -> None:
    """메인 엔트리포인트."""
    parser = argparse.ArgumentParser(description="LQR 게인 계산기")
    parser.add_argument("--mass", type=float, default=10.0, help="질량 [kg]")
    parser.add_argument("--izz", type=float, default=0.5, help="Z축 관성 모멘트 [kg·m²]")
    parser.add_argument(
        "--q", type=str, default="10,10,5,1,1,0.5",
        help="Q 대각 요소 (x,y,θ,vx,vy,ω)",
    )
    parser.add_argument(
        "--r", type=str, default="1,1,1",
        help="R 대각 요소 (Fx,Fy,Mz)",
    )
    parser.add_argument(
        "--output", type=str, default=None,
        help="출력 YAML 경로 (기본: config/gnc/controller/lqr_gains.yaml)",
    )
    args = parser.parse_args()

    Q_diag = [float(x) for x in args.q.split(",")]
    R_diag = [float(x) for x in args.r.split(",")]

    if len(Q_diag) != 6 or len(R_diag) != 3:
        print("오류: Q는 6개, R은 3개 요소 필요")
        sys.exit(1)

    Q = np.diag(Q_diag)
    R = np.diag(R_diag)

    # 시스템 모델
    A, B = build_system(args.mass, args.izz)
    print(f"차량: mass={args.mass}kg, Izz={args.izz}kg·m²")
    print(f"Q_diag: {Q_diag}")
    print(f"R_diag: {R_diag}")

    # LQR 게인 계산
    K = compute_lqr_gain(A, B, Q, R)
    K_trans, K_rot = split_gains(K)

    print(f"\n== LQR 게인 ==")
    print(f"K 전체 (3×6):")
    for row in K:
        print(f"  [{', '.join(f'{v:8.4f}' for v in row)}]")
    print(f"\nK_trans (2×4, row-major): {[round(v, 6) for v in K_trans]}")
    print(f"K_rot   (1×2):            {[round(v, 6) for v in K_rot]}")

    # 폐루프 극점 확인
    A_cl = A - B @ K
    eigvals = np.linalg.eigvals(A_cl)
    print(f"\n폐루프 극점:")
    for ev in sorted(eigvals, key=lambda x: x.real):
        print(f"  {ev.real:+.4f} {ev.imag:+.4f}j  (|ω|={abs(ev):.4f})")
    if all(ev.real < 0 for ev in eigvals):
        print("✓ 모든 극점 좌반면 — 안정")
    else:
        print("✗ 불안정 극점 존재!")

    # 불안정 게인은 저장하지 않음 (silent 저장 방지)
    if not all(ev.real < 0 for ev in eigvals):
        print("오류: 불안정 게인 — 저장 중단", file=sys.stderr)
        sys.exit(1)

    # YAML 저장 (K 게인 데이터 파일 — 제어 한계는 Hydra vehicle 그룹에서 관리)
    output_path = args.output
    if output_path is None:
        output_path = os.path.join(
            os.path.dirname(__file__), "..", "config", "gnc", "controller", "lqr_gains.yaml",
        )
    output_path = os.path.normpath(output_path)

    lqr_config = {
        "lqr": {
            "K_trans": [round(v, 8) for v in K_trans],
            "K_rot": [round(v, 8) for v in K_rot],
            "Q_diag": Q_diag,
            "R_diag": R_diag,
        }
    }

    with open(output_path, "w", encoding="utf-8") as f:
        f.write("# LQR K 게인 데이터 — scripts/compute_lqr_gains.py가 생성/갱신 (Hydra 트리 외부)\n")
        f.write(f"# 차량: mass={args.mass}kg, Izz={args.izz}kg·m² | 상태: [x, y, θ, vx, vy, ω], 입력: [Fx, Fy, Mz]\n")
        f.write("# gnc_node가 직접 로드하며, 파일/키 누락 시 기동 실패(silent zero-gain 방지)\n")
        yaml.dump(lqr_config, f, default_flow_style=None, allow_unicode=True)

    print(f"\n저장: {output_path}")


if __name__ == "__main__":
    main()
