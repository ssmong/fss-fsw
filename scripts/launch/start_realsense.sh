#!/bin/bash
# RealSense D435i 드라이버 시작 (독립 실행용 빠른 런처)
#
# 사용법:
#   bash scripts/launch/start_realsense.sh          # 기본: config/hardware/realsense.yaml 로드
#   bash scripts/launch/start_realsense.sh low      # 저대역폭 (영상 15fps, 보조 IMU 유지)
#   bash scripts/launch/start_realsense.sh depth    # Depth만 (Color/IMU 비활성)
#
# 기본 모드는 config/hardware/realsense.yaml(SSoT)을 사용 — sensors.launch.py와 동일.
# low/depth는 USB·처리·DDS 부하를 줄이는 즉석 오버라이드 (파일 미사용).

source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH="/opt/ros/humble/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CONFIG_FILE="$REPO_ROOT/config/hardware/realsense.yaml"

MODE="${1:-raw}"

case "$MODE" in
  low)
    echo "[RealSense] 저대역폭 모드 (MJPEG 640x480@15fps)"
    ros2 launch realsense2_camera rs_launch.py \
      serial_no:=_247122073297 \
      device_type:=d435i \
      initial_reset:=true \
      wait_for_device_timeout:=10.0 \
      enable_color:=true \
      enable_depth:=true \
      rgb_camera.color_format:=MJPEG \
      rgb_camera.color_profile:=640x480x15 \
      depth_module.depth_profile:=424x240x15 \
      enable_sync:=true \
      align_depth.enable:=false \
      enable_gyro:=true \
      enable_accel:=true \
      gyro_fps:=200 \
      accel_fps:=100 \
      unite_imu_method:=2 \
      diagnostics_period:=1.0 \
      pointcloud.enable:=false
    ;;
  depth)
    echo "[RealSense] Depth 전용 모드"
    ros2 launch realsense2_camera rs_launch.py \
      serial_no:=_247122073297 \
      device_type:=d435i \
      initial_reset:=true \
      wait_for_device_timeout:=10.0 \
      enable_color:=false \
      enable_depth:=true \
      depth_module.depth_profile:=424x240x15 \
      enable_gyro:=false \
      enable_accel:=false \
      unite_imu_method:=0 \
      diagnostics_period:=1.0 \
      pointcloud.enable:=false
    ;;
  raw)
    echo "[RealSense] 기본 모드 (config/hardware/realsense.yaml)"
    ros2 launch realsense2_camera rs_launch.py config_file:="$CONFIG_FILE"
    ;;
  *)
    echo "[RealSense] 알 수 없는 모드: '$MODE' (raw|low|depth 중 하나여야 함)" >&2
    exit 1
    ;;
esac
