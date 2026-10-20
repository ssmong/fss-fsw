# 밸브 실제 적용 상태 (설정된 캘리브레이션 보상 후)
std_msgs/Header header
float64[8] commanded_duty_cycles   # 목표 추력을 역변환한 입력 (안전 게이팅 전, 정규화)
float64[8] applied_values          # 실제 PLC에 전송된 값 (0~4000)
float64[8] estimated_thrust        # 적용 입력의 정적 맵 추정치 [N], 맵 미설정 시 0
bool main_valve_open               # 메인 밸브(에어베어링) 상태
float64 modbus_rtt_ms              # 최근 Modbus 왕복 시간 [ms] (SimHal은 0.0)
