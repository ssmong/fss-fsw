# fss-fsw

에어베어링 기체의 비행 소프트웨어와 지상국 FSS Mission Control.

## 지상국 · Windows

```powershell
# 최초 설치 · Python 3.10+
git clone https://github.com/ssmong/fss-fsw.git
cd fss-fsw
py -m pip install "PySide6>=6.8" pyqtgraph numpy paramiko pyte

# 실행
py scripts/live_plot.py
```

실험실 기체 `smrl@192.168.0.9`

지상국 연결 중 Self-tossing 기록은 `experiments/self-tossing/날짜/시각/`에 자동 저장됩니다. 자세는 `pose.csv`, 궤적·고스트 영상은 `trajectory.mp4`입니다.

## Jetson · Ubuntu 22.04 / ROS 2 Humble

```bash
git clone https://github.com/ssmong/fss-fsw.git ~/fss-fsw
cd ~/fss-fsw
source /opt/ros/humble/setup.bash
bash scripts/install_dependencies.sh
MAKEFLAGS=-j2 colcon build --symlink-install --executor sequential
```

[시뮬레이션](docs/ops/Cxx_Runtime.md#build-and-launch) · [실장비 운용](docs/ops/013_Arm_Execution.md) · [Self-tossing 실행](https://github.com/SNU-SMRL/FSS_FSW/blob/main/runtime/cpp/README.md)
