#!/bin/bash
# FSS FSW 의존성 설치 스크립트
# Jetson Orin Nano (JetPack 6.2, Ubuntu 22.04, ROS2 Humble) 환경
#
# 사용법: bash scripts/install_dependencies.sh

set -e

echo "=== FSS FSW 의존성 설치 ==="

# Python 패키지
echo "--- Python 런타임 ---"
pip3 install --upgrade pip
pip3 install \
    pyserial \
    hydra-core \
    omegaconf \
    numpy \
    scipy \
    matplotlib \
    pyyaml

# C++ 온보드 노드 빌드 도구
sudo apt-get install -y build-essential cmake ninja-build
# Vicon은 지상국 FSS_VICON의 UDP JSON을 표준 socket으로 수신한다.

# 실시간 대시보드 (노트북에서 실행하는 경우에도 필요)
echo "--- 대시보드 (PySide6 + pyqtgraph) ---"
pip3 install PySide6 pyqtgraph PyOpenGL

# 개발 도구
echo "--- 개발 도구 ---"
pip3 install black ruff mypy pytest

# ROS2 패키지
echo "--- ROS2 패키지 ---"
sudo apt-get update
sudo apt-get install -y \
    ros-humble-realsense2-camera \
    ros-humble-foxglove-bridge \
    ros-humble-rosbridge-server \
    ros-humble-dynamixel-sdk \
    fonts-noto-cjk \
    libeigen3-dev \
    libyaml-cpp-dev \
    libmodbus-dev \
    nlohmann-json3-dev

# Python 연구용 GNC를 선택할 때만 추가 설치한다:
# sudo apt-get install pybind11-dev python3-dev
# colcon build --cmake-args -DFSS_GNC_BUILD_PYTHON_BINDINGS=ON

# Xsens IMU 드라이버 (소스 빌드 필요)
echo "--- Xsens ROS2 드라이버 ---"
if [ ! -d ~/xsens_mti_ros2_driver ]; then
    echo "공식 Xsens ROS2 드라이버 (xsens_mti_ros2_driver) 설치 필요:"
    echo "  # ros2 브랜치를 클론 (Humble 호환, MTi-630 테스트됨)"
    echo "  git clone -b ros2 https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client.git ~/xsens_mti_ros2_driver"
    echo "  # xspublic 라이브러리 먼저 빌드 후 colcon 빌드"
    echo "  cd ~/xsens_mti_ros2_driver && pushd src/xsens_mti_ros2_driver/lib/xspublic && make && popd && colcon build"
else
    echo "이미 설치됨: ~/xsens_mti_ros2_driver"
fi

# 시리얼 권한
echo "--- 시리얼 권한 (dialout) ---"
if ! id -nG | grep -qw dialout; then
    sudo usermod -a -G dialout $USER
    echo "dialout 추가됨 — SSH 재접속 필요"
else
    echo "dialout 이미 소속"
fi

echo ""
echo "=== 설치 완료 ==="
echo "다음 단계: colcon build --symlink-install"
