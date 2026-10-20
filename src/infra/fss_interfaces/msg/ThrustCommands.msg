# GNC → 밸브 드라이버: 노즐별 목표 추력 (thruster_geometry.yaml 순서)
std_msgs/Header header
float64[8] thrust_n       # 목표 추력 [N], 음수는 0, 가용 최대 추력 초과는 포화
bool[8] enabled          # false인 노즐은 출력 0
