# Polishing Motor Driver (ROS1 Noetic)

Kinco CANopen(CiA402) **구동모터 2개**를 ROS1 토픽으로 제어하는 드라이버입니다.
외부(고객사) ROS1 Noetic 노드가 **표준 메시지 토픽만으로** 모터를 제어할 수 있습니다.

- **내 PC(개발기)**: Docker 컨테이너로 테스트
- **로봇 PC**: 도커 없이 **로컬(네이티브) 실행** — 넘겨받은 `install` 만 source 해서 구동

> 원본 참고: `navicomm_hyundai_2509NC98/src/motor_driver`(동일 Kinco 모터, ROS2).
> 본 프로젝트는 **ROS1 Noetic** 으로 재작성했고, 구동모터 2개만 사용하므로
> 리프트/턴테이블(차상모터) 로직은 제외했습니다.
> `can_interface`, `motor_controller` 는 ROS 비의존(순수 SocketCAN/C++)이라 그대로 재사용합니다.

---

## 1. 구성 개요

| 항목 | 값 |
|------|-----|
| ROS | Noetic (ROS1) |
| 아키텍처 | x86_64 |
| 모터 | Kinco 구동모터 2개 (CANopen Node ID 1, 2, 속도 제어) |
| 통신 | SocketCAN(`can0`) ↔ 모터 / TCPROS(ROS_MASTER_URI) ↔ 외부 ROS1 |

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
├── scripts/
│   ├── run_local.sh         # (로봇 PC) 도커 없이 네이티브 실행 (catkin_ws/install 기본)
│   ├── deploy_to_robot.sh   # (내 PC) install → 로봇 ~/navifra/install 로 rsync
│   ├── run_robot.sh         # (로봇 PC) 수동/디버그 실행 래퍼 (CAN 은 caninit.service 위임)
│   └── systemd/
│       └── motor_driver.service  # (로봇 PC) 부팅 자동 실행 유닛 (After/Requires=caninit.service)
└── catkin_ws/
    └── src/motor_driver/
        ├── package.xml           # catkin (format 2)
        ├── CMakeLists.txt
        ├── include/motor_driver/*.hpp
        ├── src/
        │   ├── can_interface.cpp      # SocketCAN RAW 래퍼 (ROS 비의존)
        │   ├── motor_controller.cpp   # Kinco CiA402 상태머신/PDO/속도제어 (ROS 비의존)
        │   └── motor_driver_node.cpp  # roscpp 노드 + 표준 토픽 + 안전 인터록
        ├── launch/motor_driver.launch
        └── config/motor_driver.yaml
```

---

## 2. 토픽 인터페이스 (표준 메시지)

| 방향 | 토픽 | 타입 | 설명 |
|------|------|------|------|
| 입력 | `/motor/cmd` | `std_msgs/Float32MultiArray` | `data=[motor1_rpm, motor2_rpm]` 목표 속도 |
| 입력 | `/traction_enable` | `std_msgs/Bool` | 구동 인가 지령 (인터록) — **현재 구독 주석 처리(미구성)** |
| 입력 | `/motor_brakeon_feedback` | `std_msgs/Bool` | 브레이크 체결 피드백 (`true`=체결) — **현재 구독 주석 처리(미구성)** |
| 출력 | `/motor/velocity` | `std_msgs/Float32MultiArray` | `data=[motor1_rpm, motor2_rpm]` 실측 속도 |
| 출력 | `/motor/alarm` | `std_msgs/String` | 알람/에러 메시지 (드라이브 에러 또는 `[id] MOTOR_FEEDBACK_TIMEOUT`) |
| 출력 | `/motor/error` | `std_msgs/Bool` | 에러 유무 플래그 (`true`=에러) |
| 출력 | `/motor/status` | `std_msgs/Float32MultiArray` | 모터별 `[전압V, 전류raw, statusword, error_code]` (순서=`drive_motor_ids`) |

- `motor2` 는 좌/우 대칭 장착 가정으로 내부에서 방향 반전(양수=전진).
- `cmd_timeout_sec`(기본 1.0s) 동안 `/motor/cmd` 미수신 시 0속도 유지(안전).
- `feedback_timeout_sec`(기본 0.5s) 동안 모터 피드백(TPDO2) 미수신 시 `/motor/alarm` 에
  `[id] MOTOR_FEEDBACK_TIMEOUT` 발행 + `/motor/error=true`. CAN 통신 두절 감지용이라
  드라이브 fault reset 은 하지 않는다.

### 안전 인터록 (traction_enable + 브레이크)
> 🚧 **현재 상태**: `/traction_enable`, `/motor_brakeon_feedback` 구독은
> 아직 구성 전이라 `motor_driver_node.cpp` 에서 **주석 처리**되어 있습니다.
> 인터록 로직·파라미터(`require_traction_enable`, `use_brake_interlock`)는 그대로 남아
> 있어(기본 `false`), 신호 배선 확정 후 해당 `subscribe`/콜백 주석만 해제하면 됩니다.

모터는 **(traction 인가) AND (브레이크 해제)** 일 때만 servo ON 되며,
조건이 **변할 때만(edge)** enable/disable 을 수행합니다(매 주기 재호출 방지).

| 파라미터 | 기본 | 의미 |
|----------|------|------|
| `require_traction_enable` | `false` | `true`면 `/traction_enable=true` 전까지 구동 금지 |
| `use_brake_interlock` | `false` | `true`면 브레이크 해제 전까지 구동 금지 |

> 기본값(둘 다 `false`)에서는 기동 즉시 servo ON 되어 `/motor/cmd` 로 바로 구동됩니다.
> PLC/안전 연동 시 `config/motor_driver.yaml` 에서 두 값을 `true` 로 설정하세요.
>
> ⚠️ **확인 필요(가정)**: 토픽명 `/traction_enable`, `/motor_brakeon_feedback` 과
> "브레이크 체결=`true`" 극성은 가정값입니다. 실제 PLC 신호명/극성에 맞게
> launch 의 remap 또는 코드에서 조정하세요.

---

## 3. 내 PC(개발기) — Docker 로 테스트

```bash
cd ~/Polishing
docker compose up --build
```
컨테이너가 `roslaunch` 로 **roscore + 노드**를 함께 띄웁니다.
`network_mode: host` 라 호스트/다른 컨테이너에서 `localhost:11311` master 로 접속됩니다.

### 다른 셸(호스트 또는 다른 컨테이너)에서 테스트
> ⚠️ **컨테이너 밖에서** 테스트해야 실제 연동을 검증할 수 있습니다.
```bash
export ROS_MASTER_URI=http://localhost:11311

rostopic list
#   /motor/cmd   /motor/velocity   /motor/alarm   /motor/error   /motor/status

# 피드백 확인 (속도 / 상태 / 에러)
rostopic echo /motor/velocity
rostopic echo /motor/status     # 모터별 [전압V, 전류raw, statusword, error_code]
rostopic echo /motor/error      # true=에러

# (인터록 기본 off 이므로 바로 구동) motor1=500rpm, motor2=500rpm
rostopic pub -r 10 /motor/cmd std_msgs/Float32MultiArray "{data: [500.0, 500.0]}"

# 정지
rostopic pub -1 /motor/cmd std_msgs/Float32MultiArray "{data: [0.0, 0.0]}"

# 알람 모니터
rostopic echo /motor/alarm

# (인터록 사용 시) 구동 인가 / 브레이크 해제
rostopic pub -1 /traction_enable std_msgs/Bool "data: true"
rostopic pub -1 /motor_brakeon_feedback std_msgs/Bool "data: false"
```
CAN 프레임 직접 확인: `candump can0`

---

## 4. 로봇 PC — 도커 없이 로컬 실행 (`~/navifra`)

전제: 로봇 PC 에 **ROS1 Noetic + can-utils** (개발기와 **동일 아키텍처 x86_64**).
```bash
# (Noetic 미설치 시)
# sudo apt install ros-noetic-ros-base can-utils
```
> ⚠️ install 은 `/opt/ros/noetic/lib/*` 에 **동적 링크**되므로 자기완결 번들이 아니다.
> 로봇에 Noetic 이 설치돼 있어야 하고, 개발기와 ROS 버전·아키텍처가 일치해야 한다.

### (1) install 만들기 — 개발기
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

### (2) 로봇으로 rsync — 개발기 **호스트 셸에서** (컨테이너 안 아님)
install 은 폴더째 옮기면 어느 경로든 동작한다(relocatable — `setup.bash` 가 자기 위치를 계산).
로봇의 `~/navifra/install` 로 동기화한다:
```bash
cd ~/Polishing
./scripts/deploy_to_robot.sh robot@<robot_ip>   # install + run_robot.sh + teleop.sh 배포
```
> `deploy_to_robot.sh` 는 **개발기(내 PC)에서 실행**한다(로봇 아님). 최초 구조 생성 + 이후 업데이트 겸용
> (재빌드 후 다시 실행하면 rsync 가 바뀐 파일만 전송). 전제: 로컬·로봇 양쪽에 `rsync`, 로봇 SSH 접속 가능.

스크립트 없이 직접:
```bash
ssh robot@<robot_ip> 'mkdir -p ~/navifra'
rsync -avz --delete ~/Polishing/catkin_ws/install/ robot@<robot_ip>:navifra/install/
rsync -avz ~/Polishing/scripts/run_robot.sh ~/Polishing/scripts/teleop.sh robot@<robot_ip>:navifra/
```
> 끝 슬래시(`install/`) 주의 — 폴더 **내용**을 `~/navifra/install/` 안으로. `--delete` 는 원본에서 지운 파일을 로봇에서도 정리.
> **컨테이너는 꺼져 있어도 됨**: install 은 호스트에 있으므로 rsync 는 그냥 호스트에서 실행한다.

### (3) 로봇에서 실행

역할 분담:
- **CAN(can0) up** → `caninit.service` (oneshot, 부팅 시 벤더 드라이버 로드 + can0 up)
- **환경 source** → `~/.bashrc` 에 넣어 수동 실행 시 자동 적용
  ```bash
  # ~/.bashrc 말미 (파일 없을 때 에러 방지 가드)
  source /opt/ros/noetic/setup.bash
  [ -f ~/navifra/install/setup.bash ] && source ~/navifra/install/setup.bash
  ```
- **노드 실행** → 프로덕션은 systemd, 개발/디버그는 수동 래퍼

#### 프로덕션: systemd 자동 실행 (권장)
`scripts/systemd/motor_driver.service` 를 설치한다. systemd 는 `.bashrc` 를 읽지 않으므로
유닛의 `ExecStart` 안에서 ROS/install 을 직접 source 하고, `caninit.service` 뒤에 시작한다.
```bash
# <USER> 를 로봇 계정으로 치환하여 설치 (.service 파일도 로봇에 복사돼 있어야 함)
sudo sed 's/<USER>/robot/g' ~/navifra/motor_driver.service \
  | sudo tee /etc/systemd/system/motor_driver.service
sudo systemctl daemon-reload
sudo systemctl enable --now motor_driver.service
journalctl -u motor_driver.service -f          # 로그 확인
```

#### 개발/디버그: 수동 실행
```bash
ssh robot@<robot_ip>
bash ~/navifra/run_robot.sh
```
`run_robot.sh`: ROS/install 을 방어적으로 source → `can0` up 여부 확인(경고만) →
`roslaunch motor_driver motor_driver.launch`. **CAN 은 건드리지 않는다**(caninit.service 담당).
(`INSTALL_DIR` 환경변수로 install 위치 변경 가능, 기본 `~/navifra/install`.)

> 로봇에서 **소스째 빌드**하려면: 소스 복사 후 `cd catkin_ws && catkin_make install`,
> `INSTALL_DIR=~/Polishing/catkin_ws/install bash scripts/run_robot.sh` (또는 `run_local.sh`).
> lib 호환 문제가 원천 차단되어 가장 견고하다.

### (4) 키보드 조종 — 노트북에서 SSH
키보드 teleop 은 **키를 누르는 TTY** 가 필요해서 systemd 자동실행 대상이 아니다.
노트북에서 SSH 로 접속한 세션에서 실행한다. (드라이버 노드가 먼저 떠 있어야 함 — teleop 은 `/motor/cmd` 만 발행)

**방법 A — 접속 후 직접 실행** (대화형 셸이라 TTY 있음):
```bash
ssh robot@<robot_ip>
~/navifra/teleop.sh                       # 경로 앞 ~ 필수 (=/home/<user>/navifra)
# 속도/좌우 지정:  ~/navifra/teleop.sh _linear_rpm:=200 _swap_lr:=true
```

**방법 B — 노트북에서 원터치** (한 줄 원격 실행이므로 `ssh -t` 로 TTY 할당):
```bash
cd ~/Polishing
./scripts/teleop_ssh.sh robot@<robot_ip>  # 내부적으로 ssh -t ... 'bash ~/navifra/teleop.sh'
```
> ⚠️ `ssh robot 'cmd'` 형태(원격 한 줄)는 기본적으로 TTY 가 없어 키 입력이 안 된다.
> `teleop_ssh.sh` 는 `ssh -t` 로 이를 해결한다. 접속 후 직접 치는 방법 A 는 `-t` 불필요.

조작 키: `u/i/o`(전진 좌/직/우), `j/l`(반시계/시계 스핀), `m/,/.`(후진 좌/직/우),
`k`·space(정지), `↑/↓`(속도 ±10%), `Ctrl-C`(종료).

---

### 배포·실행 순서 요약
```
[내 PC]   devcontainer 빌드            → ~/Polishing/catkin_ws/install 생성
[내 PC]   ./scripts/deploy_to_robot.sh robot@<ip>   → 로봇 ~/navifra 로 rsync (최초+업데이트)
[로봇]    드라이버 노드 실행           → systemd(motor_driver.service) 또는 bash ~/navifra/run_robot.sh
[내 PC→로봇] 키보드 조종               → ./scripts/teleop_ssh.sh robot@<ip>  (또는 SSH 후 ~/navifra/teleop.sh)
```
코드 수정 시: **[내 PC] 재빌드 → deploy 재실행 → [로봇] 노드 재시작** 순으로 반영.

---

## 5. ROS1 네트워킹 / 실무 안정화 포인트

- **`ROS_MASTER_URI`**: 모든 노드가 동일 master 를 가리켜야 통신됨.
  고객사 roscore 를 쓰면 그 주소로, 로봇이 master 면 로봇 IP:11311.
- **`ROS_IP` / `ROS_HOSTNAME`**: 다른 머신과 연동 시 각 노드가 자기 IP 를
  광고해야 상대가 접속 가능. 같은 머신 테스트면 `127.0.0.1` 로 충분.
- **QoS**: ROS1 은 기본 TCPROS(신뢰성 보장)라 별도 설정 없이 통신됨.
- **CAN 비트레이트**: 드라이브 설정과 반드시 동일. 다르면 프레임 미수신.
- **CAN 브링업 위치**:
  - 로봇(프로덕션): `caninit.service` 가 부팅 시 can0 를 올린다. 노드 systemd 유닛은
    `After=/Requires=caninit.service` 로 그 뒤에 시작한다.
  - 개발기(Docker 테스트): `CAN_AUTO_UP=1` 로 컨테이너/스크립트가 직접 up (+ `NET_ADMIN`).
- **systemd 는 `~/.bashrc` 를 읽지 않는다**: 서비스로 노드를 돌릴 땐 유닛의 `ExecStart` 안에서
  ROS/install 을 직접 source 해야 한다(`.bashrc` source 는 수동 SSH 실행에만 적용됨).
- **시작 순서**: CAN 이 up 되기 전 노드가 뜨면 CAN open 에서 실패할 수 있다.
  systemd 의 `After=/Requires=caninit.service` + `Restart=on-failure` 로 순서·복원력을 보장한다.
- ROS1↔ROS2 는 직접 통신 불가. 고객사가 Noetic 이므로 본 드라이버도 Noetic 으로 맞췄습니다.
