# 제어기 기준 상태 (유도 출력)
std_msgs/Header header
# 목표 상태
float64 x_ref       # 목표 X 위치 [m]
float64 y_ref       # 목표 Y 위치 [m]
float64 theta_ref   # 목표 방향각 [rad]
float64 vx_ref      # 목표 X 속도 [m/s]
float64 vy_ref      # 목표 Y 속도 [m/s]
float64 omega_ref   # 목표 각속도 [rad/s]
# 피드포워드 항
float64 ax_ff       # 피드포워드 X 가속도 [m/s²]
float64 ay_ff       # 피드포워드 Y 가속도 [m/s²]
float64 alpha_ff    # 피드포워드 각가속도 [rad/s²]
