# Jetson Nano Codex Handoff: STM32H753 Tracked AMR

이 문서는 Jetson Nano에서 Codex를 열어 현재 프로젝트를 이어서 작업할 때 가장 먼저 읽힐 인수인계 문서이다.

## 1. 현재 기준

- 정리 날짜: 2026-05-27
- 펌웨어/원격 구동 저장소: `https://github.com/JayBee2292/H753zi_AMR.git`
- PC 작업 경로: `/home/jongbeom/h753_ros_humble`
- Jetson 예정 경로: `/home/jongbeom/h753_ros_humble`
- 현재 Git 기준점: `42326b0`까지 GitHub `main`에 push 확인 완료
- 이 문서를 추가한 커밋은 이후 한 번 더 push해야 Jetson에서 clone/pull로 받는다.
- 로컬의 `.idea/editor.xml` 변경과 PNG 이미지 파일들은 기능 변경과 무관하여 저장소에 넣지 않았다.

## 2. 시스템 목표와 통신 역할

목표는 4개 무한궤도 구동축을 가진 STM32H753 로봇을 Xbox 패드로 원격 주행시키고, encoder/CAN odometry와 YDLIDAR Tmini Pro를 이용해 ROS2 2D SLAM으로 확장하는 것이다.

현재 역할 분리는 다음과 같다.

```text
Xbox controller
  -> Jetson/PC Python program
  -> UART through ST-LINK VCP
  -> STM32H753 motor drive

STM32H753 encoder/robot state
  -> CAN FD+BRS
  -> Makerbase CANable V2.0 Pro
  -> ROS2 h753_can_odom
  -> /odom, /odom_vel, TF odom -> base_link

YDLIDAR Tmini Pro
  -> ydlidar_ros2_driver
  -> /scan
  -> slam_toolbox with odom/TF
```

중요 원칙:

- 원격 주행 명령은 UART로 보낸다.
- 주행 데이터/엔코더 텔레메트리는 CAN으로 받는다.
- Xbox Python 구동 프로그램과 ROS2 CAN odom 노드가 동시에 동작할 때, Python은 `--no-can`으로 실행해야 CANable 장치 충돌이 없다.

## 3. 하드웨어 정보

- MCU 보드: STM32H753ZI 계열 프로젝트, ST-LINK VCP UART 사용
- PC/Jetson CAN 인터페이스: Makerbase CANable V2.0 Pro
- STM CAN 트랜시버: WeAct ISOCANFD V1 CA-IS2062A CAN FD 절연 보드
- LiDAR: YDLIDAR Tmini Pro, ROS 설정 baudrate `230400`, scan frequency `10.0 Hz`
- 구동체: 좌/우 트랙으로 모델링되는 4개 무한궤도형 바퀴
- 측정된 접지 길이 참조값: 약 `0.26 m`
- 트랙 벨트 폭 참조값: `0.075 m`
- 현재 유효 좌우 트랙 중심 거리 설정: `0.45 m`

PC에서 사용하던 stable by-id 장치명:

```text
ST-LINK UART:
/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_001F00223532511331333430-if02

CANable:
/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_209F33833630-if00
```

Jetson에서는 `/dev/ttyACM*` 번호가 달라질 수 있으므로 항상 먼저 확인한다.

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

## 4. 현재 펌웨어 구조

핵심 경로:

```text
App/Src/app_uart_smoke.c       UART CMD_TWIST 파싱, open-loop/PID 선택, 상태 UART 출력
App/Src/app_robot.c            바퀴 PPR/유효 지름, 엔코더 상태 초기화
App/Src/app_motor_drive.c      모터 PWM/방향 출력
App/Src/can_telemetry.c        CAN FD 텔레메트리 0x100/0x101
Algorithm/Src/calc_input_data.c 엔코더 delta, 64비트 누적, Kalman, 정기구학
Algorithm/Src/calc_output_data.c 역기구학
Algorithm/Src/pid_controller.c PID 구현
Core/Src/main.c                엔코더 타이머 매핑
```

엔코더 타이머 순서:

```text
FL = TIM3
FR = TIM4
RL = TIM2
RR = TIM8
```

현재 구동 런타임은 **PID 비활성화 open-loop** 상태이다.

```c
// App/Src/app_uart_smoke.c
#define APP_UART_SMOKE_USE_PID 0
```

- PID 코드는 구현되어 있지만, PID 활성화 시 직진/회전이 이상해지는 현상을 먼저 해결해야 한다.
- `calc_input_data.c`의 엔코더 속도 Kalman 필터는 현재도 측정 속도 계산 경로에서 실행된다.
- UART 상태 출력의 선두 문자열이 `OPENLOOP`이면 PID가 꺼져 있는 펌웨어가 실행 중인 것이다.

## 5. Xbox 원격 주행 구현

실행 파일:

```text
tools/run_xbox_uart_drive.sh
tools/xbox_uart_can_telemetry.py
tools/xbox_drive_config.py
tools/xbox_drive_core.py
tools/drive_test_requirements.txt
```

입력 방식:

```text
왼쪽 스틱 Y  = 전진/후진
오른쪽 스틱 X = 정지 상태 제자리 회전, 주행 중 좌/우 회전 혼합
왼쪽 스틱 X  = 사용하지 않음
```

명령 경로:

```text
joystick input
  -> body velocity v, w
  -> 좌/우 최대 트랙 속도 초과 시 v,w 동시 축소
  -> UART CMD_TWIST packet
  -> STM inverse kinematics
  -> 좌/우 PWM 출력
```

UART CMD_TWIST 프레임:

```text
A5 5A 10 v_i16_le w_i16_le checksum
v: mm/s
w: mrad/s
```

현재 Python 기본값:

```text
max_linear_mps       = 0.60
max_angular_radps    = 2.67
track_gauge_m        = 0.45
max_track_speed_mps  = 0.60
track_contact_length = 0.26
track_belt_width     = 0.075
```

구동 프로그램 안전 조치:

- 한 번에 한 UART 구동 프로세스만 실행되도록 lock 파일을 사용한다.
- 실행 로그는 `tools/logs/xbox_drive_latest.log`에 즉시 flush된다.
- `Ctrl+C`, `SIGTERM`, 터미널 종료 시 STM 정지 프레임을 전송하고 UART/로그/lock을 정리한다.
- Xbox/SDL 종료 과정이 멈추는 문제가 있어, 안전 정리 후 `os._exit()`으로 남는 Python 프로세스를 방지했다.
- 실제 중립 조종 실행 후 `Ctrl+C` 종료 시 프로세스가 남지 않는 것까지 확인했다.

Jetson에서 UART 원격 구동만 확인할 때:

```bash
cd ~/h753_ros_humble/tools
chmod +x run_xbox_uart_drive.sh
./run_xbox_uart_drive.sh
```

자동 ST-LINK 선택이 실패하면:

```bash
./run_xbox_uart_drive.sh /dev/ttyACM0
```

낮은 속도로 처음 시험할 때:

```bash
MAX_LINEAR=0.30 MAX_TRACK_SPEED=0.30 MAX_ANGULAR=0.60 ./run_xbox_uart_drive.sh
```

실시간 로그:

```bash
tail -f ~/h753_ros_humble/tools/logs/xbox_drive_latest.log
```

## 6. 엔코더 보정과 검증 결과

이전 문제:

- 전진하는데 속도 부호가 음수로 출력되던 문제를 바퀴 방향 보정으로 처리했다.
- encoder delta가 비정상적으로 튀는 경우에 대비해 STM에서 비현실적 sample을 거부하도록 구현했다.
- 64비트 누적은 타이머 update interrupt 누적 방식이 아니라, 매 주기 raw CNT delta를 wrap 보정한 뒤 `int64_t total_tick`에 누적하는 방식이다.

PPR 보정 근거:

```text
손으로 측정한 모터 출력축 10회전 count = 2,439,004
구동 바퀴 1회전당 측정 축 회전 = 2.25 = 9 / 4
구동 바퀴 1회전당 count = (2,439,004 / 10) * 2.25 = 548,775.9
적용 PPR = 548,776
```

STM 현재 설정:

```c
APP_ROBOT_MEASURED_SHAFT_COUNTS_10_REV = 2439004
APP_ROBOT_SHAFT_TURNS_PER_WHEEL_REV_NUM = 9
APP_ROBOT_SHAFT_TURNS_PER_WHEEL_REV_DEN = 4
APP_ROBOT_WHEEL_DIAMETER_M = 0.21f
APP_ROBOT_WHEEL_PPR = 548776
```

직진 검증 결과:

```text
실측: 3 m / 6.05 s = 0.496 m/s
STM 로그 평균 V: 0.501 m/s
오차: 약 +0.96%
OPENLOOP PWM 100%, target(L,R)=600,600
fb 평균: L=0.503 m/s, R=0.499 m/s
fault=0x0
```

결론:

- 직진 거리/속도 환산용 `PPR=548776`, 유효 지름 `0.21 m`는 현재 유지한다.
- 바퀴 외형 둘레 `620 mm`만으로 값을 다시 변경하지 않는다. 실제 바닥 주행 검증 결과가 현재 보정을 지지한다.
- 아직 제자리 회전/곡선 주행 실측이 부족하므로 `track_width/track_gauge=0.45 m`는 회전 시험 후 조정한다.

STM encoder fault 방어:

```text
ENCODER_MAX_WHEEL_SPEED_MPS = 3.5
ENCODER_MAX_DELTA_MARGIN_TICKS = 2048
```

- 비현실적인 delta는 `total_tick`에 합산하지 않는다.
- 해당 sample은 velocity를 0으로 하고 fault bit를 CAN/UART에 보낸다.
- PC/ROS 측도 fault, packet loss, timestamp jump, 불가능한 속도 변화를 추가 검증해야 한다.

## 7. CAN 텔레메트리와 ROS2 현황

CAN FD telemetry:

```text
0x100: 상태/속도/duty/flags
0x101: int64 encoder_tick[4], delta_abs[4], valid/fault mask
0x1FF: host telemetry enable/disable 명령
```

ROS2 작업공간은 **현재 GitHub 펌웨어 저장소 바깥**에 있다.

```text
/home/jongbeom/ros2_ws/src/h753_can_odom
/home/jongbeom/ros2_ws/src/ydlidar_ros2_driver
```

따라서 Jetson에서 ROS2까지 계속하려면 `ros2_ws/src/h753_can_odom` 및 필요한 YDLIDAR 설정/드라이버를 별도로 복사하거나 별도 저장소로 관리해야 한다.

`h753_can_odom` 구현:

- CANable에서 `0x100`, `0x101` 프레임을 읽는다.
- `/odom`, `/odom_vel`, `odom -> base_link` TF를 publish한다.
- `can_odom.launch.py`, `lidar_odom_bringup.launch.py`, `mapping_bringup.launch.py`가 존재한다.

2026-05-27에 발견하고 수정한 필수 항목:

```text
ROS2 h753_can_odom ppr가 과거 값 244000으로 남아 있었다.
STM과 일치하도록 아래 파일을 ppr: 548776으로 수정했다.

/home/jongbeom/ros2_ws/src/h753_can_odom/config/h753_can_odom.yaml
/home/jongbeom/ros2_ws/src/h753_can_odom/h753_can_odom/can_odom_node.py
```

이 수정은 `h753_ros_humble` Git 저장소 밖에 있으므로 Jetson으로 별도 이전해야 한다.

## 8. YDLIDAR/SLAM 상태

YDLIDAR Tmini Pro:

- `/scan` 출력은 약 `10 Hz`로 확인했다.
- 적용한 파라미터 파일:

```text
/home/jongbeom/ros2_ws/src/ydlidar_ros2_driver/params/Tmini_no_intensity.yaml
```

- 주요 설정:

```text
port: /dev/ttyUSB0
baudrate: 230400
frame_id: laser_frame
intensity: false
intensity_bit: 0
frequency: 10.0
```

- 이전 실행에서 `health status bad`, `Lidar internal error[0x202]`, checksum error가 발생했다. `/scan`은 출력되었지만, 실제 SLAM 전에는 재부팅/USB 재연결 후 지속 에러 여부를 다시 확인해야 한다.

ROS2 Humble은 기존 PC에서 Ubuntu 24.04 호스트 위 `distrobox` 컨테이너 `cosmos-env`로 테스트했다. Jetson Nano에서 ROS2를 쓸 때는 Jetson OS/JetPack 버전과 컨테이너 또는 설치 방식을 먼저 확인해야 한다. 원격 UART 주행 시험 자체는 ROS2 없이 Python만으로 가능하다.

## 9. Jetson Nano 이전 및 실행 순서

Jetson 정보:

```text
hostname: jyl1015
IPv4: 192.168.0.57
username: jongbeom
```

저장소 가져오기:

```bash
cd ~
git clone https://github.com/JayBee2292/H753zi_AMR.git h753_ros_humble
cd ~/h753_ros_humble
```

Python 구동 준비:

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip
sudo usermod -aG dialout $USER
```

`dialout` 적용을 위해 로그아웃 후 재접속한다. 이후 Xbox 컨트롤러와 STM32 ST-LINK USB를 Jetson에 연결하고:

```bash
cd ~/h753_ros_humble/tools
./run_xbox_uart_drive.sh
```

Jetson에서 Codex를 시작할 때 첫 요청 예시:

```text
이 프로젝트의 JETSON_CODEX_HANDOFF.md를 먼저 읽고 현재 상태를 확인해줘.
기존 동작을 깨지 않도록 git status와 장치 경로를 확인한 후,
UART Xbox 원격 주행 테스트부터 이어서 진행하자.
```

ROS2 소스는 이 저장소에 아직 없으므로, ROS2 작업까지 필요하면 원 PC에서 별도 전송한다.

```bash
rsync -av ~/ros2_ws/src/h753_can_odom/ jongbeom@192.168.0.57:~/ros2_ws/src/h753_can_odom/
rsync -av ~/ros2_ws/src/ydlidar_ros2_driver/ jongbeom@192.168.0.57:~/ros2_ws/src/ydlidar_ros2_driver/
```

## 10. 다음 작업 우선순위

1. Jetson에서 STM UART 장치와 Xbox 인식 여부를 확인하고, 낮은 속도로 open-loop 주행을 재확인한다.
2. 제자리 회전과 곡선 주행의 실제 각도/시간을 측정해 유효 `track_gauge/track_width`를 보정한다.
3. 수정된 `PPR=548776`을 적용한 ROS2 `h753_can_odom`으로 `/odom` 직진 거리를 검증한다.
4. CAN odom과 YDLIDAR `/scan`이 안정적으로 함께 출력되는지 확인한다.
5. 그 후 SLAM Toolbox mapping을 실행한다.
6. PID는 open-loop 기하/엔코더/회전 보정이 끝난 뒤 낮은 속도에서 다시 튜닝한다. 현재 펌웨어에서 PID를 바로 켜고 시험하지 않는다.

## 11. Codex가 바로 확인할 명령

```bash
cd ~/h753_ros_humble
git status --short --branch
git log --oneline --decorate --max-count=6
grep -n 'APP_UART_SMOKE_USE_PID' App/Src/app_uart_smoke.c
grep -n 'APP_ROBOT_MEASURED_SHAFT\|APP_ROBOT_SHAFT_TURNS\|APP_ROBOT_WHEEL_DIAMETER' App/Src/app_robot.c
grep -n 'DEFAULT_MAX_LINEAR\|DEFAULT_TRACK_GAUGE\|DEFAULT_MAX_TRACK_SPEED' tools/xbox_drive_config.py
ls -l /dev/serial/by-id/ /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```
