# KU Polishing Robot Driver (ROS1 Noetic)

건국대(KU) 폴리싱 로봇용 **ROS1 Noetic** 드라이버입니다 (Navifra 제공).
구동부·리프트·배터리(BMS)·조명(Crevis I/O)·Safety PLC I/O 를 각각 별도 패키지로 구현하고,
외부(고객사) ROS1 노드가 **표준 메시지 토픽만으로** 로봇을 제어할 수 있게 합니다.

- **내 PC(개발기)**: devcontainer / Docker 컨테이너로 빌드 + 테스트 (`install` 굽기)
- **로봇 PC**: 도커 없이 **네이티브 실행** — 넘겨받은 `install` 만 source 해서 구동 (systemd 자동 기동)

> 📄 **전체 ROS 인터페이스(API) 명세·파라미터·고객 검증 절차는 [`docs/INTERFACE.txt`](docs/INTERFACE.txt) 가 정본입니다.**
> 이 README 는 개발자용 요약 + 빌드/배포/실행 절차만 다룹니다. 토픽 상세·검증 시나리오는 항상 `INTERFACE.txt` 를 보세요
> (같은 내용의 고객 배포본: `docs/interface_guide.html`, `docs/KU_Polishing_Robot_Driver_인터페이스_가이드.pdf`).

> 원본 참고: `navicomm_hyundai_2509NC98/src/motor_driver`(동일 Kinco 모터, ROS2).
> 본 프로젝트는 **ROS1 Noetic** 으로 재작성했습니다. 구동모터(Kinco CANopen 2개)는
> `can_interface`/`motor_controller`(ROS 비의존, 순수 SocketCAN/C++)를 재사용하고,
> 리프트(MDROBOT RS485)·배터리(Daly BMS CAN)·조명(Crevis Modbus TCP)·Safety PLC(PILZ Modbus TCP)는
> 별도 노드로 구현했습니다. 턴테이블(차상모터) 로직은 이 로봇에 해당 없어 제외했습니다.

---

## 1. 구성 개요

| 항목 | 값 |
|------|-----|
| ROS | Noetic (ROS1), Ubuntu 20.04 |
| 아키텍처 | x86_64 |
| 구동모터 | Kinco CANopen/CiA402 2개 (Node ID 1, 2 — 속도 제어) |
| `can0` | **500 kbps** — 구동모터 |
| `can1` | **250 kbps** — Daly BMS |
| RS485 | `/dev/ttyS0`(COM1) — MDROBOT 리프트 드라이버 (USB-485 테스트 시 `/dev/ttyUSB0`) |
| 이더넷 (MODBUS TCP) | Crevis GN-9289 (조명/충전 릴레이), PILZ PNOZmulti 2 (Safety I/O) |
| 외부 연동 | TCPROS (`ROS_MASTER_URI`) ↔ 고객사 ROS1 Noetic |

> 로봇/LiDAR/Safety PLC/Crevis 의 **IP 맵**은 `docs/INTERFACE.txt` §1 참조 (중복 관리 안 함).
> 로봇 접속 계정·비밀번호는 저장소에 두지 않습니다 — 핸드오프 노트 참조.

### 패키지 구성 (`catkin_ws/src/`)

| 패키지 | 노드 | 역할 |
|--------|------|------|
| `motor_driver` | `motor_driver_node` | 저수준 구동모터 (CAN/CiA402, `/motor/*`) |
| | `base_controller` | 표준 차동구동 인터페이스 (`/cmd_vel` ↔ `/odom` + TF) |
| `can_interface` | (lib) | 공용 SocketCAN 래퍼 `libcan_interface.so` — `motor_driver`·`bms_driver` 가 링크 |
| `bms_driver` | `bms_driver` | Daly BMS 배터리 모니터링 (`can1`, `/bms/state`·`/bms/soc`) |
| `lift_driver` | `lift_driver` | MDROBOT 리프트 (RS485, `/lift/*`) + `serial_interface` 자체 보유 |
| `crevis_io_driver` | `crevis_io_node` | 조명 LED 6개 / 충전 릴레이 / DI (Crevis GN-9289, Modbus TCP) |
| `safety_io_driver` | `safety_io_node` | Safety PLC I/O 상태 + 하드 비상정지 통합 토픽 (`/safety/*`) |

> 헤더 include 규칙 = **패키지명 prefix**. 예) `#include "can_interface/can_interface.hpp"`,
> `"lift_driver/serial_interface.hpp"`.

### 디렉토리 구조
```
Polishing/
├── Dockerfile               # (내 PC) Noetic 테스트/빌드 이미지 (배포용 install 굽기)
├── docker-compose.yml       # (내 PC) 테스트 컨테이너
├── docker/entrypoint.sh
├── .devcontainer/           # VS Code Dev Containers (컨테이너 진입 후 반복 빌드)
│   ├── Dockerfile           #   noetic-ros-base + 빌드도구/can-utils + 비루트 navifra
│   ├── devcontainer.json    #   소스 bind-mount, privileged+host net (CAN/ROS)
│   └── postcreate.sh        #   rosdep + 최초 catkin_make install
├── .vscode/tasks.json       # 빌드 태스크 (Ctrl+Shift+B → ROS: build (install))
├── docs/                    # 고객 전달용 산출물 (정본)
│   ├── INTERFACE.txt        #   ROS 인터페이스 API + 검증 가이드 (계속 갱신)
│   ├── interface_guide.html #   위 내용 HTML 소스
│   └── KU_Polishing_Robot_Driver_인터페이스_가이드.pdf
├── scripts/
│   ├── deploy_to_robot.sh   # (내 PC) install + 스크립트 → 로봇 ~/navifra 로 rsync
│   ├── run_robot.sh         # (로봇 PC) 전체 브링업 (robot.launch, USE_* 로 서브시스템 on/off)
│   ├── param.yaml           # 현장 보정값 템플릿 → 로봇 ~/navifra/param.default.yaml
│   ├── sync_param.py        # 로봇 param.yaml 에 신규 키만 additive merge (현장값 보존)
│   ├── teleop.sh            # (로봇 PC) 키보드 텔레옵 (/cmd_vel)
│   ├── teleop_ssh.sh        # (내 PC) ssh -t 로 위 스크립트 원터치 실행
│   ├── run_local.sh         # (개발용) 저수준 motor_driver.launch 만 네이티브 실행
│   ├── caninit_script.sh    # (로봇 PC) 벤더 CAN 모듈 로드 + can0/can1 up (사본, /usr/bin 에 설치)
│   └── systemd/
│       ├── caninit.service      # (로봇 PC) 부팅 시 CAN 초기화 (oneshot)
│       └── navifra-robot.service # (로봇 PC) 부팅 시 run_robot.sh 자동 기동
└── catkin_ws/
    └── src/
        ├── motor_driver/
        │   ├── include/motor_driver/*.hpp
        │   ├── src/
        │   │   ├── motor_controller.cpp     # Kinco CiA402 상태머신/PDO/속도제어 (ROS 비의존)
        │   │   ├── motor_driver_node.cpp    # 저수준 노드: /motor/cmd·/motor/velocity·/motor/status …
        │   │   └── base_controller_node.cpp # 상위 표준: /cmd_vel↔/odom (차동구동, TF)
        │   ├── scripts/teleop_keyboard.py   # 키보드 텔레옵 (/cmd_vel 발행)
        │   ├── launch/
        │   │   ├── robot.launch             # ★ 전체 브링업 (서브시스템 include, use_* 인자)
        │   │   ├── bringup.launch           # 구동부 턴키 (드라이버 + base_controller)
        │   │   └── motor_driver.launch      # 저수준 드라이버만
        │   └── config/{motor_driver,base_controller}.yaml
        ├── can_interface/          # 공용 SocketCAN lib (src/can_interface.cpp)
        ├── bms_driver/             # src/bms_driver_node.cpp, launch/bms.launch, config/
        ├── lift_driver/            # src/{lift_driver_node,serial_interface}.cpp, launch/lift.launch
        ├── crevis_io_driver/       # src/crevis_io_node.cpp (순수 소켓 Modbus TCP, 외부 의존 0)
        └── safety_io_driver/       # src/safety_io_node.cpp (PILZ Modbus TCP)
```

---

## 2. 토픽 인터페이스 (요약)

> 전체 명세(필드 단위 설명·메시지 예시·파라미터·검증 절차)는 **`docs/INTERFACE.txt` §3~§5**.
> 아래는 개발자용 한눈 요약입니다.

### 상위 표준 인터페이스 (고객사 사용 권장)
| 방향 | 토픽 | 타입 | 설명 |
|------|------|------|------|
| 입력 | `/cmd_vel` | `geometry_msgs/Twist` | 주행: `linear.x` [m/s] + `angular.z` [rad/s] |
| 출력 | `/odom` | `nav_msgs/Odometry` | 추정 위치·자세·속도 (+ TF `odom`→`base_link`) |
| 출력 | `/bms/state` | `sensor_msgs/BatteryState` | 잔량(`percentage`)·전압·전류·충전상태·온도 |
| 출력 | `/bms/soc` | `std_msgs/Float32` | 잔량 [%] 0~100 (편의용) |
| 입력 | `/lift/command` | `std_msgs/String` | 리프트 `"up"`/`"down"`/`"stop"` |
| 입력 | `/lift/position_cmd`·`/lift/inc_position_cmd`·`/lift/home` | `Int32`·`Int32`·`Bool` | 절대/상대 위치이동, 홈잉 |
| 출력 | `/lift/position`·`/lift/status`·`/lift/homed` | `Int32`·`String`·`Bool` | 위치(홀카운트)·상태문자열·원점확립 |
| 입력 | `/crevis/led/*` (6개) | `std_msgs/Bool` | 조명 on/off (vision/front/side/status_rgb) |
| 입력 | `/crevis/charging` | `std_msgs/Bool` | 충전 릴레이(Y01.06) ON/OFF |
| 출력 | `/crevis/charge_port_on`·`/crevis/led_state*`·`/crevis/connected` | `Bool`·`Bool`/`String`·`Bool` | 릴레이·LED readback, 연결상태 |
| 출력 | `/safety/input/*`·`/safety/output/*`·`/safety/state_all` | `Bool`·`Bool`·`String` | Safety PLC 물리 I/O raw 상태 |
| 출력 | `/safety/estop` | `std_msgs/Bool` | **하드 비상정지 통합상태** (`true`=비상 활성) |

> ⚠️ **긴급정지는 하드웨어 Safety PLC 전담**입니다. 소프트웨어 `/estop` 토픽은 **제거되었습니다**(2026-07-27, v0.11).
> ROS 는 `/safety/estop` 으로 하드 비상정지 상태를 **감지만** 하며, 물리 차단은 PILZ 안전회로가 담당합니다.
> 판정 = (비상버튼 1b/2b · 범퍼 front/rear 중 하나라도 triggered) **OR** (traction 전원 출력 off).
> 노드 시작 직후·PLC 통신두절 시 fail-safe `true`. 상세는 `INTERFACE.txt` §3.6/§3.7.

### 저수준 인터페이스 (`motor_driver_node`, 디버그/직접제어)
| 방향 | 토픽 | 타입 | 설명 |
|------|------|------|------|
| 입력 | `/motor/cmd` | `std_msgs/Float32MultiArray` | `data=[motor1_rpm, motor2_rpm]` 목표 속도(모터축) |
| 출력 | `/motor/velocity` | `std_msgs/Float32MultiArray` | 실측 속도 |
| 출력 | `/motor/status` | `std_msgs/Float32MultiArray` | 모터별 `[전압V, 전류raw, statusword, error_code]` (순서=`drive_motor_ids`) |
| 출력 | `/motor/error` | `std_msgs/Bool` | 에러 유무 (`true`=드라이브 fault 또는 피드백 타임아웃) |
| 출력 | `/motor/alarm` | `std_msgs/String` | 알람 메시지 (에러코드 / `[id] MOTOR_FEEDBACK_TIMEOUT`) |

- 두 바퀴 모두 **"양수 = 전진"** 규약 (좌/우 방향 반전은 `motor_directions` 로 드라이버가 처리).
- `cmd_timeout_sec`(기본 1.0s) 동안 `/motor/cmd` 미수신 시 0속도 유지(안전).
- `feedback_timeout_sec`(기본 0.5s) 동안 TPDO2 미수신 시 `/motor/alarm` 에 `MOTOR_FEEDBACK_TIMEOUT`
  + `/motor/error=true`. CAN 통신 두절 감지용이라 드라이브 fault reset 은 하지 않는다.
- `base_controller` 는 `/cmd_vel` → 역기구학 → `/motor/cmd`, `/motor/velocity` → 적분 → `/odom`.
  물리값(`wheel_radius` 0.0825·`wheel_separation` 0.65·`gear_ratio` 9)은 실측 확정값.

### 안전 인터록 (traction_enable + 브레이크) — 미구성
> 🚧 `/traction_enable`, `/motor_brakeon_feedback` 구독은 아직 배선/신호 확정 전이라
> `motor_driver_node.cpp` 에서 **주석 처리**되어 있습니다. 인터록 로직·파라미터
> (`require_traction_enable`, `use_brake_interlock` — 둘 다 기본 `false`)는 그대로 남아 있어,
> 신호 확정 후 해당 `subscribe`/콜백 주석만 해제하면 됩니다.

모터는 **(traction 인가) AND (브레이크 해제)** 일 때만 servo ON 되며, 조건이 **변할 때만(edge)**
enable/disable 을 수행합니다(매 주기 재호출 방지). 기본값(둘 다 `false`)에서는 기동 즉시 servo ON.

> ⚠️ 토픽명 `/traction_enable`·`/motor_brakeon_feedback` 과 "브레이크 체결=`true`" 극성은 **가정값**입니다.
> 실제 PLC 신호명/극성에 맞게 launch remap 또는 코드에서 조정하세요.

---

## 3. 내 PC(개발기) — 빌드 & Docker 테스트

### 빌드 (install 굽기)
devcontainer 는 소스를 bind-mount 하므로, 빌드 결과물이 **호스트의 `catkin_ws/install` 에 그대로** 생긴다.
- VS Code: "Dev Containers: Reopen in Container" → `Ctrl+Shift+B` (ROS: build (install))
- 또는 CLI:
```bash
docker build -f .devcontainer/Dockerfile -t polishing-dev:noetic .
docker run --rm -v ~/Polishing:/workspaces/Polishing \
  -w /workspaces/Polishing/catkin_ws --user navifra polishing-dev:noetic \
  bash -c "source /opt/ros/noetic/setup.bash && catkin_make install"
# → ~/Polishing/catkin_ws/install (호스트에 생성됨)
```

### 컨테이너로 노드 띄워 테스트
```bash
cd ~/Polishing
docker compose up --build
```
기본 커맨드(`Dockerfile` 의 `CMD`)는 **`roslaunch motor_driver motor_driver.launch`** — 즉
**저수준 모터 노드만** 띄웁니다(`roscore` 는 roslaunch 가 자동 기동). `base_controller`·BMS·리프트·
조명·Safety 는 포함되지 않으므로, `/cmd_vel`·`/odom` 등을 테스트하려면 커맨드를 덮어쓴다:
```bash
docker compose run --rm motor_driver roslaunch motor_driver bringup.launch   # 구동부 + base_controller
docker compose run --rm motor_driver roslaunch motor_driver robot.launch     # 전체
```
`network_mode: host` 라 호스트/다른 컨테이너에서 `localhost:11311` master 로 접속됩니다.
`CAN_AUTO_UP=1`(기본)이라 컨테이너가 `can0` 를 직접 올립니다(`NET_ADMIN`).
반복 빌드·디버그는 위 devcontainer 쪽이 편합니다(소스 bind-mount).

> ⚠️ **컨테이너 밖에서** 테스트해야 실제 연동을 검증할 수 있습니다.
```bash
export ROS_MASTER_URI=http://localhost:11311
rostopic list

# 저수준 (인터록 기본 off 이므로 바로 구동) motor1=500rpm, motor2=500rpm
rostopic pub -r 10 /motor/cmd std_msgs/Float32MultiArray "{data: [500.0, 500.0]}"
rostopic pub -1  /motor/cmd std_msgs/Float32MultiArray "{data: [0.0, 0.0]}"   # 정지
rostopic echo /motor/velocity        # 실측 rpm
rostopic echo /motor/status          # [전압V, 전류raw, statusword, error_code]
rostopic echo /motor/error           # true=에러
rostopic echo /motor/alarm

# 표준 인터페이스 (bringup/robot.launch 로 base_controller 포함 기동한 경우)
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
rostopic echo /odom
rosrun tf tf_echo odom base_link
```
CAN 프레임 직접 확인: `candump can0` (BMS 는 `candump can1`)

서브시스템별 단독 실행:
```bash
roslaunch motor_driver bringup.launch          # 구동부 (드라이버 + base_controller)
roslaunch bms_driver   bms.launch              # 배터리 (can1)
roslaunch lift_driver  lift.launch             # 리프트 (RS485)
roslaunch crevis_io_driver crevis_io.launch    # 조명/충전 릴레이 (Modbus TCP)
roslaunch safety_io_driver safety_io.launch    # Safety PLC I/O (Modbus TCP)
roslaunch motor_driver robot.launch            # ★ 전체 (use_lift:=false 등으로 개별 제외)
```
> 각 검증 절차(기대값 포함)는 `docs/INTERFACE.txt` §4 에 시나리오로 정리돼 있습니다.

---

## 4. 로봇 PC — 배포 & 실행 (`~/navifra`)

전제: 로봇 PC 에 **ROS1 Noetic + tf2-ros + can-utils** (개발기와 **동일 아키텍처 x86_64**).
```bash
# (Noetic 미설치 시)
# sudo apt install ros-noetic-ros-base ros-noetic-tf2-ros can-utils
```
> ⚠️ install 은 `/opt/ros/noetic/lib/*` 에 **동적 링크**되므로 자기완결 번들이 아니다.
> 로봇에 Noetic 이 설치돼 있어야 하고, 개발기와 ROS 버전·아키텍처가 일치해야 한다.

### (1) 로봇으로 rsync — 개발기 **호스트 셸에서** (컨테이너 안 아님)
install 은 폴더째 옮기면 어느 경로든 동작한다(relocatable — `setup.bash` 가 자기 위치를 계산).
```bash
cd ~/Polishing
./scripts/deploy_to_robot.sh <user>@<robot_ip>
```
`deploy_to_robot.sh` 가 하는 일:

| 대상 | 로봇 경로 | 비고 |
|------|-----------|------|
| `catkin_ws/install/` | `~/navifra/install/` | `rsync --delete` (지운 파일도 정리) |
| `run_robot.sh`, `teleop.sh` | `~/navifra/` | 실행 스크립트 |
| `scripts/param.yaml` | `~/navifra/param.default.yaml` | 최신 템플릿 (항상 갱신) |
| `sync_param.py` 실행 결과 | `~/navifra/param.yaml` | **additive merge** — 현장 편집값·주석 보존, 신규 키만 추가 |

> `deploy_to_robot.sh` 는 **개발기에서 실행**한다(로봇 아님). 최초 구조 생성 + 이후 업데이트 겸용.
> **systemd 유닛 파일과 `caninit_script.sh` 는 배포 대상이 아니다** — 아래 (3)처럼 손으로 설치한다(1회).
> **컨테이너는 꺼져 있어도 됨**: install 은 호스트에 있으므로 rsync 는 그냥 호스트에서 실행한다.

스크립트 없이 직접:
```bash
ssh <user>@<robot_ip> 'mkdir -p ~/navifra'
rsync -avz --delete ~/Polishing/catkin_ws/install/ <user>@<robot_ip>:navifra/install/
rsync -avz ~/Polishing/scripts/run_robot.sh ~/Polishing/scripts/teleop.sh <user>@<robot_ip>:navifra/
```
> 끝 슬래시(`install/`) 주의 — 폴더 **내용**을 `~/navifra/install/` 안으로.

### (2) 현장 보정값 — `~/navifra/param.yaml` (재빌드 불필요, 권장 튜닝 경로)
자주 만지는 값(주행 캘리브레이션·모터 방향·리프트 속도 등)은 **로봇의 `~/navifra/param.yaml` 한 파일**에서 편집한다.
런치가 각 패키지 기본 config **뒤에** 로드하므로 같은 키를 덮어쓴다(param.yaml 이 이김).
```bash
nano ~/navifra/param.yaml          # 최상위 키 = 노드 이름
sudo systemctl restart navifra-robot   # 반영 (또는 수동 실행 재시작)
```
- 구조: `base_controller:` / `motor_driver_node:` / `lift_driver:` 아래 파라미터. 없는 키는 기본값 그대로.
- 노출 항목·의미는 `docs/INTERFACE.txt` §5. 항목 추가는 저장소 `scripts/param.yaml` 을 편집 후 재배포.
- 최신 템플릿은 배포 시 `~/navifra/param.default.yaml` 로 갱신되니 신규 항목은 여기서 확인.

### (3) 역할 분담 & 최초 세팅 (1회)

| 역할 | 담당 |
|------|------|
| CAN 브링업 (`can0` 500K + `can1` 250K, 벤더 모듈 `can-ahc0512.ko`) | `caninit.service` → `/usr/bin/caninit_script.sh` (oneshot, 부팅 시) |
| 전체 노드 기동 (roscore + 드라이버 6노드) | `navifra-robot.service` → `~/navifra/run_robot.sh` |
| 수동 SSH 셸 환경 | `~/.bashrc` 의 source 구문 |

```bash
# ~/.bashrc 말미 (파일 없을 때 에러 방지 가드) — 수동 실행용
source /opt/ros/noetic/setup.bash
[ -f ~/navifra/install/setup.bash ] && source ~/navifra/install/setup.bash
```

systemd 유닛 설치 — 저장소의 유닛 파일을 로봇에 복사해 넣는다(**`deploy_to_robot.sh` 대상 아님**):
```bash
# 개발기에서 로봇으로 전송
scp scripts/systemd/navifra-robot.service scripts/systemd/caninit.service <user>@<robot_ip>:~
scp scripts/caninit_script.sh <user>@<robot_ip>:~

# 로봇에서 설치
sudo cp ~/navifra-robot.service ~/caninit.service /etc/systemd/system/
sudo install -m 755 ~/caninit_script.sh /usr/bin/caninit_script.sh
sudo systemctl daemon-reload
sudo systemctl enable --now caninit.service navifra-robot.service
```
> ⚠️ `--now` 는 **즉시 전체 드라이버를 기동**한다(모터 servo ON 포함). 로봇 주변 안전을 확보한 뒤
> 실행하거나, `enable` 만 하고 원하는 시점에 `start` 하라.
> ⚠️ `navifra-robot.service` 는 로봇 계정을 **하드코딩**한다(`User=abc`, `Group=abc`,
> `Environment=HOME=/home/abc`, `ExecStart=/home/abc/navifra/run_robot.sh`).
> 계정이 다르면 복사 후 이 4줄을 수정해야 한다. `caninit_script.sh` 의 `insmod /home/abc/can-ahc0512.ko`
> 경로도 같이 확인.

### (4) 프로덕션 운용 — systemd
```bash
sudo systemctl start   navifra-robot    # 기동 (roscore + 전체 노드)
sudo systemctl stop    navifra-robot    # 정지 (SIGINT 로 깔끔히 종료)
sudo systemctl restart navifra-robot    # 재시작 (param.yaml·config 수정 후 반영)

systemctl status navifra-robot          # 상태 요약 (sudo 불필요)
journalctl -u navifra-robot -f          # 실시간 로그
journalctl -u navifra-robot -n 50       # 최근 50줄

sudo systemctl enable/disable navifra-robot   # 부팅 자동실행 on/off
```
> ⚠️ 서비스가 켜져 있으면 roscore·노드가 이미 떠 있다. 수동 `roslaunch` 전에 반드시 `stop`
> (노드 이름 충돌 방지).
> 유닛은 `After=caninit.service network.target` + `Requires=caninit.service` 로 **CAN 이 올라온 뒤** 시작하고,
> `Restart=on-failure`(3s) 로 복원력을 갖는다.

### (5) 개발/디버그 — 수동 실행
```bash
ssh <user>@<robot_ip>
sudo systemctl stop navifra-robot     # 먼저 서비스 정지
bash ~/navifra/run_robot.sh
```
`run_robot.sh`: ROS/install 방어적 source → `can0` up 여부 확인(경고만, CAN 은 건드리지 않음) →
`param.yaml` 존재 확인 → `roslaunch motor_driver robot.launch` (라인버퍼링 `stdbuf -oL -eL` 로
systemd journal 에 로그가 실시간으로 뜨게 함).

서브시스템 on/off — 스크립트 상단 `USE_*` 기본값을 편집하거나 환경변수로 덮어쓴다:
```bash
USE_DRIVE=false bash ~/navifra/run_robot.sh                      # 모터만 제외
USE_DRIVE=false USE_BMS=false USE_CREVIS=false USE_SAFETY=false \
  bash ~/navifra/run_robot.sh                                    # 리프트만
```
- `INSTALL_DIR` 로 install 위치 변경 가능(기본 `~/navifra/install`), `PARAM_FILE` 로 보정값 파일 지정.
- `scripts/run_local.sh` 는 **저수준 `motor_driver.launch` 만** 띄우는 개발용 스크립트다
  (`CAN_AUTO_UP=1` 기본 → 자기가 `sudo ip link` 로 CAN 을 올린다). `base_controller`·BMS·리프트·조명·Safety 가
  없으므로 로봇 프로덕션 경로로 쓰지 말 것 — 로봇에서는 `run_robot.sh` 를 쓴다.

> 로봇에서 **소스째 빌드**하려면: 소스 복사 후 `cd catkin_ws && catkin_make install`,
> `INSTALL_DIR=~/Polishing/catkin_ws/install bash scripts/run_robot.sh`.
> lib 호환 문제가 원천 차단되어 가장 견고하다.

### (6) 키보드 조종 — 노트북에서 SSH
키보드 teleop 은 **키를 누르는 TTY** 가 필요해서 systemd 자동실행 대상이 아니다.
teleop 은 `/cmd_vel`(Twist) 만 발행하므로 **`base_controller` 가 떠 있어야** 실제로 움직인다
(`navifra-robot.service` / `run_robot.sh` / `bringup.launch` 는 포함).

**방법 A — 접속 후 직접 실행** (대화형 셸이라 TTY 있음):
```bash
ssh <user>@<robot_ip>
~/navifra/teleop.sh                       # 경로 앞 ~ 필수 (=/home/<user>/navifra)
# 속도 지정:  ~/navifra/teleop.sh _linear_speed:=0.3 _angular_speed:=0.8   # [m/s],[rad/s]
```

**방법 B — 노트북에서 원터치** (한 줄 원격 실행이므로 `ssh -t` 로 TTY 할당):
```bash
cd ~/Polishing
./scripts/teleop_ssh.sh <user>@<robot_ip>  # 내부적으로 ssh -t ... 'bash ~/navifra/teleop.sh'
```
> ⚠️ `ssh <user>@<ip> 'cmd'` 형태(원격 한 줄)는 기본적으로 TTY 가 없어 키 입력이 안 된다.
> `teleop_ssh.sh` 는 `ssh -t` 로 이를 해결한다. 접속 후 직접 치는 방법 A 는 `-t` 불필요.

조작 키: `u/i/o`(전진 좌/직/우), `j/l`(반시계/시계 스핀), `m/,/.`(후진 좌/직/우),
`k`·space(정지), `↑/↓`(속도 ±10%), `Ctrl-C`(종료).

### 배포·실행 순서 요약
```
[내 PC]   devcontainer 빌드                        → ~/Polishing/catkin_ws/install 생성
[내 PC]   ./scripts/deploy_to_robot.sh <user>@<ip> → 로봇 ~/navifra 로 rsync (최초+업데이트)
[로봇]    (최초 1회) systemd 유닛 + caninit_script.sh 설치·enable
[로봇]    sudo systemctl restart navifra-robot     → 전체 노드 반영
[내 PC→로봇] 키보드 조종                            → ./scripts/teleop_ssh.sh <user>@<ip>
```
코드 수정 시: **[내 PC] 재빌드 → deploy 재실행 → [로봇] `systemctl restart navifra-robot`** 순으로 반영.
설정만 만질 때는 로봇의 `~/navifra/param.yaml` 편집 + restart (재빌드 불필요).

---

## 5. ROS1 네트워킹 / 실무 안정화 포인트

- **`ROS_MASTER_URI`**: 모든 노드가 동일 master 를 가리켜야 통신됨.
  고객사 roscore 를 쓰면 그 주소로, 로봇이 master 면 로봇 IP:11311.
- **`ROS_IP` / `ROS_HOSTNAME`**: 다른 머신과 연동 시 각 노드가 자기 IP 를
  광고해야 상대가 접속 가능. 같은 머신 테스트면 `127.0.0.1` 로 충분.
- **QoS**: ROS1 은 기본 TCPROS(신뢰성 보장)라 별도 설정 없이 통신됨.
- **CAN 비트레이트**: 드라이브 설정과 반드시 동일. 다르면 프레임 미수신.
  두 버스가 **비트레이트가 달라**(can0 500K / can1 250K) 물리적으로 분리돼 있다.
- **CAN 브링업 위치**:
  - 로봇(프로덕션): `caninit.service` 가 부팅 시 벤더 모듈 로드 + `can0`/`can1` 을 올린다.
    `navifra-robot.service` 는 `After=`/`Requires=caninit.service` 로 그 뒤에 시작한다.
  - 개발기(Docker 테스트): `CAN_AUTO_UP=1` 로 컨테이너/`run_local.sh` 가 직접 up (+ `NET_ADMIN`).
- **systemd 는 `~/.bashrc` 를 읽지 않는다**: 그래서 `run_robot.sh` 안에서 ROS/install 을 직접 source 한다
  (`.bashrc` source 는 수동 SSH 실행에만 적용됨).
- **로그가 안 보일 때**: 비-TTY(systemd) 에서 노드 stdout 이 블록 버퍼링돼 journal 에 늦게 뜬다.
  `run_robot.sh` 가 `stdbuf -oL -eL` 로 라인버퍼링을 강제한다(자식 노드에 전파).
- **시리얼 권한**: 리프트(RS485)를 쓰려면 로봇 계정이 **dialout 그룹**에 있어야 한다
  (`sudo usermod -aG dialout <user>` 후 재로그인). USB-RS485 테스트 시 `ModemManager` 가
  포트를 프로빙해 방해하면 `sudo systemctl stop ModemManager`.
- ROS1↔ROS2 는 직접 통신 불가. 고객사가 Noetic 이므로 본 드라이버도 Noetic 으로 맞췄습니다.

---

## 6. 알려진 이슈 (요약)

전체 목록·조치 내역은 **`docs/INTERFACE.txt` §6** 참조.

- ⚠️ **2번 구동모터 하드웨어 고장 — 수리 의뢰 중** (2026-07-27 현재). 로봇의 `~/navifra/param.yaml` 에
  `drive_motor_ids: [1]` / `motor_directions: [-1]` 을 손으로 넣어 **1번 모터만** 구동 중(소스·템플릿 변경 없음).
  좌우 차동주행이 성립하지 않으므로 `/cmd_vel` 주행·`/odom` 정확도 실기 검증은 **입고 후**. 복구 시
  `drive_motor_ids` 줄 삭제 + `motor_directions: [-1, 1]` 로 원복.
- **모터 enable 실패 = STO(Safe Torque Off) 하드웨어 이슈**. 드라이브가 `0x000F`(operation enable) 순간
  FAULT(`0x2601`/`0x2602`)면 STO 안전입력 미충족을 의심한다 — 비상정지/안전회로가 리셋되어 STO 단자에
  전원이 들어와 있어야 servo ON 된다. **SW/CiA402 시퀀스는 정상**으로 확인됨.
- **리프트 위치는 증분(홀 카운트)** — 전원 재부팅 시 원점이 소실된다. 절대위치(`/lift/position_cmd`)를
  쓰려면 전원 사이클마다 1회 `/lift/home` 홈잉이 필요하고, 홈잉 전에는 절대위치 명령이 거부된다
  (상대이동·수동 up/down 은 동작). 자동 홈잉 `auto_home_on_start` 는 안전상 **기본 `false`**
  (켜면 부팅 시 리프트가 자동 하강) — 스트로크 실측 0~7000 카운트.
- **안전 인터록 미구성**: `/traction_enable`·`/motor_brakeon_feedback` 은 배선 확정 전이라 구독 주석 처리(§2).
