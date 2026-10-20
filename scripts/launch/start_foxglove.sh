#!/bin/bash
# Foxglove Bridge 시작 (원격 모니터링용)
# 노트북 브라우저에서 Foxglove Studio로 Jetson 토픽을 실시간 확인한다.
#
# 사용법:
#   bash scripts/launch/start_foxglove.sh
#
# 노트북에서:
#   1) SSH 터널: ssh -L 8765:localhost:8765 smong@<IP> -p <PORT>
#   2) 브라우저: https://app.foxglove.dev → Foxglove WebSocket → ws://localhost:8765

source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH="/opt/ros/humble/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"

echo "[Foxglove] Bridge 시작 (포트 8765)..."
echo "[Foxglove] 노트북에서 SSH 터널 후 https://app.foxglove.dev 접속"
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
