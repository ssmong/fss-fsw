#!/bin/bash
# Xsens MTi-630 드라이버만 시작한다.
# IMU 브리지는 fss_full/sensors 런치가 Hydra 설정으로 한 번만 실행한다.
#
# 사용법: bash scripts/launch/start_imu.sh
# 포트 잠김 시: fuser -k /dev/ttyUSB0

set -e

source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH="/opt/ros/humble/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"
source ~/xsens_mti_ros2_driver/install/setup.bash

if ! id -nG | grep -qw dialout; then
    echo "❌ dialout 그룹 미소속 — IMU 시리얼 포트 접근 불가"
    echo ""
    echo "해결 방법:"
    echo "  sudo usermod -a -G dialout \$USER"
    echo "  # 이후 SSH 재접속 (VSCode: Ctrl+Shift+P → Kill VS Code Server → 재접속)"
    exit 1
fi

echo "[IMU] Xsens 드라이버 시작..."
echo "[IMU] raw 토픽: /imu/data"
echo "[IMU] /sensors/imu 브리지는 fss_full.launch.py 또는 sensors.launch.py로 실행하세요."
exec ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
