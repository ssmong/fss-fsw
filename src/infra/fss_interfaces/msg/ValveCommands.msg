# 밸브 드라이버 → SIL/관측: HAL 전송에 성공한 8개 비례밸브 입력
# 추력 맵 역변환과 안전 게이팅을 거친 정규화 입력 (0.0~1.0).
std_msgs/Header header
float64[8] duty_cycles    # 각 밸브의 듀티 사이클 (0.0~1.0)
bool[8] enabled           # 밸브별 활성화 플래그
