# 시스템 운영 모드
std_msgs/Header header
uint8 mode
uint8 IDLE=0              # 대기: 모든 밸브 꺼짐
uint8 PRESSURIZING=1      # 가압 중: 에어베어링 활성화 대기
uint8 READY=2             # 준비: 에어베어링 활성, GNC 대기
uint8 ACTIVE=3            # 활성: GNC 루프 실행 중
uint8 SAFE_HOLD=4         # 안전 유지: 정지 유지 모드
uint8 EMERGENCY_STOP=5    # 비상 정지: 모든 출력 즉시 차단
string mode_name           # 모드명 문자열
