# fss-fsw

에어베어링 우주선 모형의 비행 소프트웨어와 지상국 **FSS Mission Control**입니다.

## PC에서 관제하기

1. PC를 Jetson과 같은 실험실 네트워크에 연결하고 **FSS Mission Control**을 여세요.
2. **+ FSS 연결**에서 주소 `192.168.0.9`, 계정 `smrl`과 SSH 암호를 입력하세요. 기체마다 원하는 ID와 **로봇팔 장착** 여부를 지정하세요. 진행 중인 실험을 지켜볼 때는 **모니터링만**을 선택하세요.
3. 상단에서 기체를 선택하세요. 파란색 기체가 터미널과 조작 버튼의 대상입니다. 본체 위치 제어 실험은 **에어베어링 ON → PID 또는 LQR → X·Y(m)와 Yaw(°) 입력 → 목표 전송** 순서입니다.
4. 지도에서 기체 옆의 X·Y·Yaw를 확인하세요. 관절각은 **상세 → 로봇팔**에 있습니다. 오른쪽의 **SSH**와 **로그**를 전환하고 **+**로 셸을 추가할 수 있습니다.
5. 기록하려면 **실험 시작**을 누르고 이름을 입력하세요. 종료하면 관제 PC의 `experiments/<이름>/YYYY-MM-DD/HH-MM-SS/`에 수신 데이터, 로그, 궤적 그림이 남습니다. 첫 이름은 `default`이며 다음부터는 직전에 쓴 이름을 수정할 수 있습니다.

### 새 Windows PC에서 처음 열 때

Python 3.10 이상을 설치한 뒤 저장소 폴더에서 PowerShell로 실행하세요. 두 번째부터는 마지막 줄만 실행하면 됩니다.

```powershell
py -m pip install PySide6 pyqtgraph numpy paramiko pyte
py scripts/live_plot.py
```

## 새 Jetson 준비

Ubuntu 22.04에 ROS 2 Humble과 `colcon`이 설치된 Jetson에서 한 번 실행하세요.

```bash
git clone https://github.com/ssmong/fss-fsw.git ~/fss-fsw
cd ~/fss-fsw
source /opt/ros/humble/setup.bash
bash scripts/install_dependencies.sh
MAKEFLAGS=-j2 colcon build --symlink-install --executor sequential
```

기체 기동과 종료는 [실장비 운용 절차](docs/ops/013_Arm_Execution.md), 모의 실험은 [시뮬레이션 실행 방법](docs/ops/Cxx_Runtime.md#build-and-launch)을 참고하세요.
