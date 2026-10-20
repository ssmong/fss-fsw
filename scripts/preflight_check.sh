#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# FSS 사전 점검 스크립트 (Preflight Check)
# Jetson 접속 후 첫 번째로 실행하여 모든 하드웨어/소프트웨어 상태를 확인한다.
#
# 사용법:
#   bash scripts/preflight_check.sh
#
# 체크 항목:
#   1. ROS2 환경
#   2. 시리얼 포트 권한 (dialout)
#   3. IMU (Xsens MTi-630-DK)
#   4. RealSense D435i
#   5. PLC 네트워크 (Modbus TCP)
#   6. Vicon 네트워크 (FSS_VICON UDP)
#   7. 잔여 프로세스 정리
#   8. 빌드 상태
# ═══════════════════════════════════════════════════════════════

# 색상
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

PASS=0
WARN=0
FAIL=0

pass() { echo -e "  ${GREEN}✅ $1${NC}"; ((PASS++)); }
warn() { echo -e "  ${YELLOW}⚠️  $1${NC}"; ((WARN++)); }
fail() { echo -e "  ${RED}❌ $1${NC}"; ((FAIL++)); }
info() { echo -e "  ${CYAN}ℹ  $1${NC}"; }
header() { echo -e "\n${BOLD}[$1] $2${NC}"; }

echo -e "${BOLD}═══════════════════════════════════════════${NC}"
echo -e "${BOLD}  FSS Preflight Check$(date +' — %Y-%m-%d %H:%M')${NC}"
echo -e "${BOLD}═══════════════════════════════════════════${NC}"

# ─────────────────────────────
header "1/8" "ROS2 환경"
# ─────────────────────────────
if command -v ros2 &>/dev/null; then
    pass "ros2 명령어 사용 가능"
else
    fail "ros2 명령어 없음 — source /opt/ros/humble/setup.bash 필요"
    echo -e "  ${YELLOW}→ 자동 소싱 시도...${NC}"
    source /opt/ros/humble/setup.bash 2>/dev/null
    export LD_LIBRARY_PATH="/opt/ros/humble/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"
    source ~/xsens_mti_ros2_driver/install/setup.bash 2>/dev/null
    source ~/fss-fsw/install/setup.bash 2>/dev/null
    if command -v ros2 &>/dev/null; then
        pass "ros2 자동 소싱 성공"
    else
        fail "ros2 소싱 실패 — .bashrc 확인 필요"
    fi
fi

# ROS_DOMAIN_ID
info "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0 (기본값)}"

# ─────────────────────────────
header "2/8" "시리얼 포트 권한"
# ─────────────────────────────
if id -nG | grep -qw dialout; then
    pass "dialout 그룹 소속"
else
    warn "dialout 그룹 미소속 — Xsens/반응휠 MCU 접근에 sg dialout 필요"
    info "영구 해결: sudo usermod -a -G dialout \$USER + 재로그인"
fi

# 반응휠 MCU는 아직 선택 장치이며 GNC에 연결되지 않는다.
if [ -e /dev/fss_reaction_wheel ]; then
    pass "반응휠 MCU 고정 포트 감지: /dev/fss_reaction_wheel"
elif compgen -G "/dev/ttyACM*" >/dev/null; then
    RW_PORT=$(compgen -G "/dev/ttyACM*" | head -1)
    info "반응휠 후보 USB-CDC 포트: $RW_PORT (아직 bench_only)"
else
    info "반응휠 MCU 미연결 (현재 선택 장치, GNC 미통합)"
fi

# ─────────────────────────────
header "3/8" "IMU (Xsens MTi-630-DK)"
# ─────────────────────────────
if ! command -v lsusb >/dev/null 2>&1; then
    warn "lsusb 미설치 — USB 장치 자동 감지 불가 (sudo apt install usbutils)"
elif lsusb | grep -q "2639:0300"; then
    pass "Xsens USB 디바이스 감지 (VID:2639)"
else
    fail "Xsens USB 미감지 — USB 케이블 확인 (Micro-B, 데이터 케이블)"
fi

if [ -e /dev/ttyUSB0 ]; then
    pass "/dev/ttyUSB0 존재"
    # 포트 잠김 확인
    FUSER_OUT=$(fuser /dev/ttyUSB0 2>/dev/null)
    if [ -n "$FUSER_OUT" ]; then
        warn "/dev/ttyUSB0 잠김 (PID: $FUSER_OUT)"
        info "해제: fuser -k /dev/ttyUSB0"
    else
        pass "/dev/ttyUSB0 사용 가능 (잠김 없음)"
    fi
else
    fail "/dev/ttyUSB0 없음"
fi

# Xsens 드라이버 설치 확인
if [ -d ~/xsens_mti_ros2_driver/install ]; then
    pass "Xsens ROS2 드라이버 빌드 완료"
else
    fail "Xsens 드라이버 미설치 — ~/xsens_mti_ros2_driver 없음"
fi

# ─────────────────────────────
header "4/8" "RealSense D435i"
# ─────────────────────────────
if ! command -v lsusb >/dev/null 2>&1; then
    warn "lsusb 미설치 — RealSense USB 감지 생략 (sudo apt install usbutils)"
else
    RS_INFO=$(lsusb | grep -i "8086.*RealSense")
    if [ -n "$RS_INFO" ]; then
        MODEL=$(echo "$RS_INFO" | sed 's/.*Intel(R) RealSense(TM) //' | sed 's/ *$//')
        pass "RealSense 감지: $MODEL"
    else
        warn "RealSense USB 미감지 — 연결 확인"
    fi
fi

if dpkg -l ros-humble-realsense2-camera &>/dev/null; then
    pass "RealSense ROS2 드라이버 설치됨"
else
    warn "realsense2_camera 패키지 미설치"
fi

# SN과 USB 링크 속도는 영상 스트리밍 전용 도구가 있을 때 추가 확인한다.
if command -v rs-enumerate-devices >/dev/null 2>&1; then
    RS_DEVICES=$(rs-enumerate-devices -s 2>/dev/null)
    if echo "$RS_DEVICES" | grep -q "247122073297"; then
        pass "설정된 D435i SN 감지 (247122073297)"
    else
        warn "설정된 D435i SN 미감지 — config/hardware/realsense.yaml과 장치 확인"
    fi

    if echo "$RS_DEVICES" | grep -Eqi "Usb Type Descriptor.*3\."; then
        pass "D435i USB 3.x 링크 확인"
    else
        warn "D435i USB 3.x 링크 미확인 — USB 2.x에서는 영상 프레임 드롭 가능"
    fi
else
    info "rs-enumerate-devices 없음 — D435i SN/USB 링크 상세 확인 생략"
fi

# ─────────────────────────────
header "5/8" "PLC 네트워크 (Modbus TCP)"
# ─────────────────────────────
# Ethernet 인터페이스 확인
ETH_IF=$(ip -4 addr show | grep "192.168.1" | head -1)
if [ -n "$ETH_IF" ]; then
    pass "Ethernet IP 192.168.1.x 설정됨"
else
    warn "192.168.1.x 네트워크 없음 — PLC 케이블 확인"
fi

# PLC ping
if ping -c 1 -W 1 192.168.1.2 &>/dev/null; then
    pass "PLC 응답 (192.168.1.2)"
else
    warn "PLC 미응답 — 케이블/전원 확인"
fi

# ─────────────────────────────
header "6/8" "Vicon 네트워크 (FSS_VICON UDP)"
# ─────────────────────────────
# WiFi 인터페이스 확인
WIFI_IF=$(ip -4 addr show | grep "192.168.0" | head -1)
if [ -n "$WIFI_IF" ]; then
    pass "WiFi IP 192.168.0.x 설정됨"
else
    warn "192.168.0.x 네트워크 없음 — WiFi 연결 확인"
fi

# Vicon PC ping
if ping -c 1 -W 1 192.168.0.2 &>/dev/null; then
    pass "Vicon PC 응답 (192.168.0.2)"
else
    warn "Vicon PC 미응답 — 실험실 네트워크에서만 가능"
fi

info "지상국 FSS_VICON의 UDP 목적지를 Jetson IP와 config/hardware/vicon.yaml의 port(기본 9870)로 설정"
info "UDP는 TCP 포트 검사로 확인할 수 없음 — 브릿지 기동 후 /sensors/vicon_state 수신 확인"

# ─────────────────────────────
header "7/8" "잔여 프로세스 정리"
# ─────────────────────────────
STALE=""
for PROC in xsens_mti_node imu_bridge_node vicon_bridge_node realsense2_camera_node ekf_node gnc_node valve_driver_node plant_model_node manager_node; do
    PID=$(pgrep -f "$PROC" 2>/dev/null)
    if [ -n "$PID" ]; then
        STALE="$STALE $PROC($PID)"
    fi
done

if [ -n "$STALE" ]; then
    warn "잔여 ROS2 노드 발견:$STALE"
    info "정리: pkill -f '<노드명>' 또는 아래 명령으로 전체 정리"
    info "  pkill -f 'ros2|xsens_mti|fss_'"
else
    pass "잔여 ROS2 노드 없음"
fi

# ─────────────────────────────
header "8/8" "빌드 상태"
# ─────────────────────────────
if [ -d ~/fss-fsw/install ]; then
    PKG_COUNT=$(ls ~/fss-fsw/install/ 2>/dev/null | grep -c "fss_\|xsens")
    if [ "$PKG_COUNT" -gt 0 ]; then
        pass "FSS 빌드 완료 ($PKG_COUNT 패키지)"
    else
        fail "install/ 존재하나 fss_ 패키지 0개 — colcon build 실패 의심 (재빌드 필요)"
    fi
else
    fail "FSS 미빌드 — colcon build --symlink-install 필요"
fi

# ─────────────────────────────
# 결과 요약
# ─────────────────────────────
echo ""
echo -e "${BOLD}═══════════════════════════════════════════${NC}"
echo -e "  ${GREEN}PASS: $PASS${NC}  ${YELLOW}WARN: $WARN${NC}  ${RED}FAIL: $FAIL${NC}"
echo -e "${BOLD}═══════════════════════════════════════════${NC}"

if [ $FAIL -gt 0 ]; then
    echo -e "\n${RED}${BOLD}FAIL 항목을 해결한 후 다시 실행하세요.${NC}"
    exit 1
elif [ $WARN -gt 0 ]; then
    echo -e "\n${YELLOW}WARN 항목은 일부 기능 제한 가능. 필요 시 해결.${NC}"
    exit 0
else
    echo -e "\n${GREEN}${BOLD}모든 항목 정상 — FSS 시스템 준비 완료!${NC}"
    exit 0
fi
