# 평면 우주선 시뮬레이터 전체 상태
# 테이블(관성) 프레임 기준 위치 + 속도 (관성 프레임 GNC 정식화)
std_msgs/Header header
float64 x          # X 위치 (테이블 프레임) [m]
float64 y          # Y 위치 (테이블 프레임) [m]
float64 theta      # 방향각 [rad]
float64 vx         # X 속도 (테이블 프레임) [m/s]
float64 vy         # Y 속도 (테이블 프레임) [m/s]
float64 omega      # 각속도 [rad/s]
# 추정 불확실도 (EKF 공분산 대각 1σ). 발행자가 추정기가 아니면 0
# Vicon 상실 시 추측항법으로 증가 → 매니저가 임계값으로 항법 저하 판정
float64 std_x      # X 위치 1σ [m]
float64 std_y      # Y 위치 1σ [m]
float64 std_theta  # 방향각 1σ [rad]
