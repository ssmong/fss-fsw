"""2D 평면 우주선 시뮬레이터용 수학 유틸리티 함수.

회전 행렬, wrench 변환, 좌표계 변환 등을 제공한다.
"""

import numpy as np
from numpy.typing import NDArray


def rotation_matrix_2d(theta: float) -> NDArray[np.float64]:
    """2D 회전 행렬을 생성한다.

    Args:
        theta: 회전각 [rad]

    Returns:
        2x2 회전 행렬
    """
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def body_to_table(
    position_body: NDArray[np.float64], theta: float
) -> NDArray[np.float64]:
    """본체 프레임 좌표를 테이블 프레임으로 변환한다.

    Args:
        position_body: 본체 프레임에서의 2D 위치 [x, y]
        theta: 본체의 방향각 [rad]

    Returns:
        테이블 프레임에서의 2D 위치
    """
    R = rotation_matrix_2d(theta)
    return R @ position_body


def table_to_body(
    position_table: NDArray[np.float64], theta: float
) -> NDArray[np.float64]:
    """테이블 프레임 좌표를 본체 프레임으로 변환한다.

    Args:
        position_table: 테이블 프레임에서의 2D 위치 [x, y]
        theta: 본체의 방향각 [rad]

    Returns:
        본체 프레임에서의 2D 위치
    """
    R = rotation_matrix_2d(theta)
    return R.T @ position_table


def wrap_angle(angle: float) -> float:
    """각도를 [-pi, pi) 범위로 정규화한다.

    Args:
        angle: 입력 각도 [rad]

    Returns:
        정규화된 각도 [rad]
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


def build_thruster_matrix(
    thrusters: list[dict],
) -> NDArray[np.float64]:
    """추력기 배치로부터 3x8 추력기 구성 행렬 B를 생성한다.

    B * u = [Fx, Fy, Mz] (본체 프레임)
    여기서 u는 8x1 개별 추력 벡터.

    Args:
        thrusters: 추력기 정보 목록. 각 항목은 'position'과 'direction' 키를 가짐.
                   position: [x, y, z], direction: [dx, dy, dz]

    Returns:
        3x8 추력기 구성 행렬
    """
    n = len(thrusters)
    B = np.zeros((3, n))

    for i, t in enumerate(thrusters):
        pos = np.array(t["position"], dtype=float)
        dir_ = np.array(t["direction"], dtype=float)

        # direction은 단위벡터여야 함 (아니면 f_max 스케일이 조용히 틀어짐).
        # 개수(3×n)는 하류에서 강제되지만 방향 크기는 여기가 유일한 검증점이다.
        norm = float(np.linalg.norm(dir_))
        if not np.isclose(norm, 1.0, atol=1e-6):
            raise ValueError(
                f"[build_thruster_matrix] 추력기 {i} direction이 단위벡터가 아님 "
                f"(norm={norm:.6f}): {t['direction']}"
            )

        # 힘 기여 (Fx, Fy)
        B[0, i] = dir_[0]
        B[1, i] = dir_[1]

        # 토크 기여 (Mz = x*Fy - y*Fx)
        B[2, i] = pos[0] * dir_[1] - pos[1] * dir_[0]

    return B


def clamp(value: float, min_val: float, max_val: float) -> float:
    """값을 지정 범위 내로 제한한다.

    Args:
        value: 입력값
        min_val: 최소값
        max_val: 최대값

    Returns:
        범위 내로 제한된 값
    """
    return max(min_val, min(value, max_val))
