# H753zi_AMR

STM32H753ZI 기반 **4륜 스키드 스티어 AMR 하위 제어기 펌웨어**입니다.
엔코더 오도메트리 계산, 역기구학, PID 속도 제어, FD-CAN 통신, Micro-ROS 브리지를 담당합니다.

상위 제어기(Jetson)의 ROS 2 자율주행 스택은 별도 저장소에 있습니다.
→ [ros2_graduation_project_ws](https://github.com/JayBee2292/ros2_graduation_project_ws)

---

## 시스템 구성

```
┌─────────────────────────────┐        FD-CAN (BRS)        ┌──────────────────────────┐
│  Jetson  (상위 제어기)       │  0x100  odometry  ──────>  │  STM32H753ZI  (본 저장소) │
│  SLAM · Nav2 · YOLO · VLM   │  <──────  cmd_vel  0x200   │  엔코더 · IK · PID · PWM  │
└─────────────────────────────┘                            └──────────────────────────┘
                                                                 │
                                                    TIM1 PWM + DIR GPIO
                                                                 ▼
                                                     4륜 모터 드라이버 (FL/FR/RL/RR)
```

**차체 사양**

| 항목 | 값 |
|---|---|
| 구동 방식 | 4륜 스키드 스티어 |
| 윤거 (좌우 바퀴 중심 간) | 0.45 m |
| 축거 (앞뒤 바퀴 중심 간) | 0.50 m |
| MCU | STM32H753ZI (Cortex-M7) / NUCLEO-H753ZI |
| RTOS | FreeRTOS (CMSIS-RTOS v2) |

---

## 주요 기능

### 1. 4륜 엔코더 오도메트리

16비트 타이머의 CNT를 그대로 쓰면 오버플로우마다 값이 접히면서 누적 거리에 드리프트가 생깁니다.
이를 막기 위해 **CNT delta를 64비트 절대 틱으로 누적**하고, 누적 거리는 절대 틱에서 직접 환산합니다.

- 타이머 인코더 모드 4채널 (TIM2 / TIM3 / TIM4 / TIM8)
- 오버플로우·언더플로우 인터럽트를 부호 보정해 반영
- 물리적으로 불가능한 delta를 `fault_count`로 집계하는 엔코더 이상 감지
- 바퀴별 부호 반전 플래그로 차량 전진을 항상 양수로 정규화

### 2. 역기구학과 모터 제어

```c
void calculate_inverse_kinematics(
    float target_linear_v,      // 목표 차체 선속도 (m/s)
    float target_angular_w,     // 목표 차체 각속도 (rad/s)
    target_wheel_velocities_t *target_wheels);
```

- 목표 차체 속도 → 4륜 목표 선속도 변환 (스키드 스티어 유효 윤거 반영)
- TIM1 PWM + DIR GPIO로 바퀴별 duty(-100 ~ 100) 출력
- 좌/우 보정용 PID 제어기 — **적분 와인드업 제한**과 **출력 클램프** 포함

### 3. 신호 필터링

엔코더에서 계산한 속도는 샘플링 노이즈가 커서 그대로 쓰면 PID가 진동합니다.
바퀴마다 **1차 칼만 필터**를 붙여 속도를 안정화했습니다.

| 파라미터 | 값 |
|---|---|
| 프로세스 노이즈 Q | 0.04 |
| 측정 노이즈 R | 0.12 |

연산은 CMSIS-DSP(`arm_math`)를 사용합니다.

### 4. FD-CAN 통신 프로토콜

FDCAN1을 **CAN FD + BRS**로 구성하고 프레임 포맷을 직접 정의했습니다.

**주행 명령 / 오도메트리** (DLC 8, little-endian float)

| CAN ID | 방향 | 페이로드 |
|---|---|---|
| `0x100` | STM32 → Jetson | `float linear_v (m/s)`, `float angular_w (rad/s)` |
| `0x200` | Jetson → STM32 | `float linear_v (m/s)`, `float angular_w (rad/s)` |

**텔레메트리** (64바이트, 1 Mbps nominal / 5 Mbps data)

```
u32 seq              u32 timestamp_ms
f32 robot_linear_v   f32 robot_angular_w
f32 wheel_velocity[4]        // FL, FR, RL, RR
i16 left_duty  i16 right_duty  i16 duty  i16 curve_ratio
u8  motion
u8  flags            // bit0 ready / bit1 motion active / bit2 encoder fault
u32 bus_off_count    u8 last_error_code   // PSR.LEC 스냅샷
```

**통신 워치독** — `cmd_vel`이 **500 ms** 이상 수신되지 않으면 자동으로 속도를 0으로 만듭니다.
상위 제어기가 죽거나 케이블이 빠져도 로봇이 계속 달리지 않도록 하는 안전 장치입니다.

### 5. Micro-ROS 브리지

`Mros/`에 executor · publisher · subscriber 래퍼를 두어, 애플리케이션 계층이 rclc API를 직접 다루지 않도록 분리했습니다.

- `geometry_msgs/Twist` 퍼블리시 (기본 토픽 `odom_vel`)
- 메시지 버퍼를 컨텍스트 안에 유지해 publish마다 재할당하지 않음

---

## 디렉터리 구조

```
Algorithm/          오도메트리 계산, 역기구학, PID, 칼만 필터
  ├ calc_input_data.*     엔코더 → 바퀴/차체 상태
  ├ calc_output_data.*    역기구학 (차체 속도 → 바퀴 목표 속도)
  ├ pid_controller.*      PID (와인드업 제한 · 출력 클램프)
  └ kalman_filter.*       1차 칼만 필터
App/                애플리케이션 계층
  ├ app_robot.*           로봇 상태 저장소, 엔코더 타이머 매핑
  ├ app_motor_drive.*     TIM1 PWM · DIR GPIO 출력
  ├ can_odom.*            0x100 / 0x200 오도메트리·cmd_vel
  ├ can_telemetry.*       64바이트 텔레메트리 프레임
  └ app_*_smoke.*         FD-CAN · UART · 모터 단위 점검 루틴
Mros/               Micro-ROS executor / publisher / subscriber 래퍼
Core/               CubeMX 생성 코드 (main, freertos, tim, fdcan, dma, usart)
Drivers/            STM32 HAL · CMSIS
Middlewares/        FreeRTOS
tools/              Xbox 패드 UART 주행 · CAN 텔레메트리 모니터 (Python)
```

---

## 빌드

STM32CubeMX 프로젝트(`h753_ros_humble.ioc`)와 CMake 빌드를 함께 사용합니다.

```bash
cmake --preset Debug        # 또는 Release
cmake --build build/Debug
```

STM32CubeIDE에서 프로젝트를 열어 빌드·플래시해도 됩니다.

**사용 페리페럴** — TIM1(PWM) · TIM2/3/4/8(엔코더) · FDCAN1 · USART3 · DMA · FreeRTOS

---

## 테스트 도구

`tools/`에 하드웨어 점검용 스크립트가 있습니다.

```bash
./run_xbox_uart_drive.sh          # Xbox 패드로 UART 오픈루프 주행
python xbox_uart_can_telemetry.py # CAN 텔레메트리 실시간 모니터
```

`App/`의 `app_fdcan_smoke` · `app_uart_smoke` · `app_motor_smoke`는
전체 시스템을 올리기 전에 통신·모터를 개별로 확인하는 루틴입니다.

---

## 관련 저장소

| 저장소 | 역할 |
|---|---|
| **H753zi_AMR** (현재) | 하위 제어기 펌웨어 — 엔코더 · IK · PID · FD-CAN |
| [ros2_graduation_project_ws](https://github.com/JayBee2292/ros2_graduation_project_ws) | 상위 제어기 — SLAM · Nav2 · YOLO · VLM 게이트웨이 |
