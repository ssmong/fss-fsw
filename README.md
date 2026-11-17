# fss-fsw

에어베어링 기체의 C++ 비행 소프트웨어와 Windows 지상국 **FSS Mission Control**입니다.

## Windows에서 지상국 열기

PC에 처음 받는다면 이 페이지의 **Code → Download ZIP**으로 내려받아 압축을 푸세요. Python 3.10 이상을 설치한 뒤 `scripts` 폴더가 보이는 위치에서 PowerShell을 여세요. 처음에는 두 줄을 모두 실행하고, 다음부터는 마지막 줄만 실행하세요.

```powershell
py -m pip install PySide6 pyqtgraph numpy paramiko pyte
py scripts/live_plot.py
```

## 기체 연결과 실험 기록

1. PC를 기체와 같은 네트워크에 연결하세요. **+ FSS 연결**에서 주소(실험실 기체 `192.168.0.9`), 계정(`smrl`), SSH 암호를 입력하세요. 원하는 기체 ID를 넣고 팔이 있으면 **로봇팔 장착**을 선택하세요.
2. 진행 중인 실험을 볼 때는 **모니터링만**을 선택하세요. 여러 대를 연결하면 상단의 파란 버튼이 현재 기체입니다. 왼쪽 3 × 4 m 지도에서 위치·방향과 궤적을 보고, 오른쪽에서 **SSH** 또는 **로그**를 여세요.
3. 본체 위치 제어 실험은 **에어베어링 ON → PID 또는 LQR → X·Y(m), Yaw(°) 입력 → 목표 전송** 순서입니다. 드라이버가 보고한 상태에 따라 ON은 초록색, OFF는 진회색으로 표시됩니다.
4. **실험 시작**에서 이름을 정하고 **실험 종료**를 누르면 관제 PC의 `experiments/<이름>/YYYY-MM-DD/HH-MM-SS/`에 수신 데이터, 로그, XY 궤적 그림이 저장됩니다. 첫 이름은 `default`이고 다음부터는 직전 이름이 표시됩니다.

## 새 Jetson 준비

Ubuntu 22.04, ROS 2 Humble, `colcon`이 설치된 새 기체에서 한 번 실행하세요. 이미 준비된 기체는 PC에서 바로 연결하면 됩니다.

```bash
git clone https://github.com/ssmong/fss-fsw.git ~/fss-fsw
cd ~/fss-fsw
source /opt/ros/humble/setup.bash
bash scripts/install_dependencies.sh
MAKEFLAGS=-j2 colcon build --symlink-install --executor sequential
```

본체 모의 비행은 [시뮬레이션 실행 방법](docs/ops/Cxx_Runtime.md#build-and-launch), 기동과 종료는 [실장비 운용 절차](docs/ops/013_Arm_Execution.md)를 참고하세요.

실험실 self-tossing 기체는 같은 C++ 본체에 팔·그리퍼 서비스를 추가합니다. Python과 C++ 실험 코드는 같은 ROS 2 인터페이스를 사용합니다. [self-tossing 연결 방법](https://github.com/SNU-SMRL/FSS_FSW/blob/main/runtime/cpp/README.md)
