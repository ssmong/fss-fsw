# 시스템 건강 상태 요약
std_msgs/Header header
bool navigation_ok             # 항법 시스템 정상 여부
bool actuators_ok              # 액추에이터 시스템 정상 여부
bool main_valve_open           # 메인 밸브(에어베어링) 상태
bool navigation_degraded       # 항법 성능 저하 (Vicon 장애 등)
float64 control_loop_rate_hz   # 실제 제어 루프 주파수 [Hz]
float64 navigation_age_sec     # 마지막 유효한 항법 업데이트 이후 경과 시간 [s]
float64 modbus_rtt_ms          # 최근 Modbus 왕복 시간 [ms]
string[] warnings              # 경고 메시지 목록
string[] errors                # 오류 메시지 목록
