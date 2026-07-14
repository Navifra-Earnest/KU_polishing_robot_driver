# safety_io_driver (ROS1 Noetic)

PILZ **PNOZ m B0.1** Safety PLC의 물리 I/O 상태를 **PNOZ m ES ETH / Modbus TCP**로
읽어 ROS 토픽으로 퍼블리시하고, 충전 요청을 PNOZ virtual input으로 전달하는
패키지입니다. Modbus 통신은 C++ POSIX 소켓으로 구현되어 별도 `libmodbus`
의존성이 없습니다.

> ROS와 Ethernet은 안전 기능을 구성하지 않습니다. 충전 요청은 반드시 검증된
> PNOZmulti 로직에서 안전 조건과 조합한 뒤 물리 출력 O2를 구동해야 합니다.

## 하드웨어와 주소

| 항목 | 값 |
|---|---|
| Base unit | PILZ PNOZ m B0.1 |
| Ethernet module | PILZ PNOZ m ES ETH |
| IP | `192.168.100.103` |
| Port | `502` (PNOZmulti 고정) |
| 물리 입력 전체 `i0..i31` | FC02 주소 `0x4000..0x401F` |
| 현재 읽는 입력 `I0..I10` | FC02 주소 `0x4000..0x400A` |
| 물리 출력 전체 `o0..o31` | FC02 주소 `0x4020..0x403F` |
| 현재 읽는 출력 `O0..O3` | FC02 주소 `0x4020..0x4023` |

주소 근거는 PILZ 공식 문서
[PNOZmulti 2 Communication Interfaces (1002971-EN)](https://www.pilz.com/download/open/OM_CI_PNOZmulti_2_1002971-EN-14.pdf)의
`Process data addressing on base unit`입니다. PNOZmulti 주소는 0부터 시작합니다.

PILZ 서비스 데이터는 장치 내부에서 나누어 갱신되므로 전체 데이터 갱신에 최대
약 500 ms가 걸릴 수 있습니다. `publish_rate`는 ROS 폴링 주기이며 PLC 내부 갱신
지연을 줄이지 않습니다.

## 회로도 I/O 매핑

`건국대학교 폴리싱 로봇 SPLC.pdf`, page 018의 `SAFETY PLC CIRCUIT -01` 기준입니다.
신호 이름은 이전 구현인
`/home/max/ws/project_mando_L_26008ITK/src/project/nc_modbus_agent/scripts/nc_modbus_agent_L.py`와
호환되도록 `safety_*` 형식을 사용했습니다.

| 방향 | PLC I/O | ROS 이름 | 회로 신호 |
|---|---:|---|---|
| 입력 | I0 / `0x4000` | `safety_emergency_1b` | EMERGENCY 1b |
| 입력 | I1 / `0x4001` | `safety_emergency_2b` | EMERGENCY 2b |
| 입력 | I4 / `0x4004` | `safety_auto_mode` | AUTO MODE |
| 입력 | I5 / `0x4005` | `safety_manual_mode` | MANUAL MODE |
| 입력 | I6 / `0x4006` | `safety_reset_switch` | RESET SWITCH |
| 입력 | I7 / `0x4007` | `safety_brake_release_sw` | BRAKE RELEASE SWITCH |
| 입력 | I9 / `0x4009` | `safety_bumper_front` | BUMPER FRONT |
| 입력 | I10 / `0x400A` | `safety_bumper_rear` | BUMPER REAR |
| 출력 | O0 | `motor_sto_01sr` | 01SR MOTO STO ON/OFF |
| 출력 | O1 | `motor_sto_02sr` | 02SR MOTO STO ON/OFF |
| 출력 | O2 | `charge_port_on` | CHARGE PORT ON |
| 출력 | O3 | `traction_motor_power_on` | TRACTION MOTOR POWER ON |

이전 `nc_modbus_agent_L.py`는 `0x4004`에서 6점을 연속으로 읽어 범퍼를
`0x4008/0x4009`로 매핑했습니다. 현재 PDF에서는 `I8`이 비어 있고 범퍼가
`I9/I10`이므로 이 패키지는 `0x4009/0x400A`를 사용합니다. 현장 PLC 프로젝트가
이전 배치와 같다면 반드시 실제 PNOZ 표시창과 대조한 뒤 YAML 매핑을 조정해야 합니다.

각 Bool 값은 회로 의미를 재해석한 `safe/unsafe` 값이 아니라 PLC 물리 I/O의 raw
논리 상태입니다. 예를 들어 `safety_emergency_1b=true`만으로 비상정지 해제 상태라고
가정하면 안 됩니다.

## 충전 릴레이 제어

구독 토픽:

```text
/safety/charging    std_msgs/Bool    true=충전 요청, false=충전 해제 요청
```

명령 경로:

```text
/safety/charging
       ↓ FC05
PNOZ virtual input i2 (Modbus coil 0x0002)
       ↓ PNOZmulti 안전 로직
physical O2 (CHARGE PORT ON)
       ↓
충전 릴레이
```

`config/safety_io.yaml`의 `i2`는 임시값이며 충전 명령 기능은 기본 비활성화되어
있습니다. PNOZmulti 프로젝트 또는 아래 수동 진단으로 실제 가상 입력 번호를 확인한
뒤 `charging_virtual_input`을 수정하고 `charging_command_enabled: true`로 변경합니다.

물리 출력 readback 주소 `0x4022`에는 쓰지 않습니다. `/safety/output/charge_port_on`은
PNOZ 안전 로직을 거친 실제 O2 상태를 확인하는 퍼블리시 토픽입니다.

드라이버는 PNOZ Control Register 255에 기본 1초 watchdog을 설정하고, 현재 충전
명령을 매 poll마다 다시 씁니다. 노드 또는 통신이 1초 이상 끊기면 PNOZ가 virtual
inputs를 `0`으로 복귀시키도록 구성했습니다. 종료 시에도 `false`를 한 번 씁니다.

### 가상 입력 수동 진단

`diagnose_virtual_input.py`는 지정한 가상 입력 한 점만 잠시 ON하고 물리 출력
O0~O3의 변화를 비교한 뒤 반드시 OFF 복구를 시도합니다. 물리 입력 `0x4000` 영역과
물리 출력 readback `0x4020` 영역에는 쓰지 않습니다.

테스트 전에 모터와 충전 릴레이의 **출력 부하를 물리적으로 차단**하고 PLC와 Ethernet만
살려둡니다. `safety_io_node`와 진단 스크립트는 같은 로컬 잠금 파일을 사용하므로 같은
PLC에 동시 접속하려 하면 나중 프로세스가 실행을 거부합니다. 다른 PC의 Modbus
클라이언트까지 감지하지는 못하므로 별도로 중지해야 합니다.

후보 하나를 지정해 실행합니다. 실행 중 화면에 표시되는 확인 문자열을 정확히
입력해야 실제 쓰기가 수행됩니다.

```bash
rosrun safety_io_driver diagnose_virtual_input.py --candidate 2
```

기본 ON 관찰 시간과 OFF 복구 대기 시간은 각각 0.5초입니다. PLC 로직 또는 서비스
데이터 갱신이 더 느리면 `--settle 1.0 --restore-wait 1.0`처럼 늘릴 수 있습니다.

결과 의미:

- `likely charging command candidate`: O2만 `OFF -> ON`으로 바뀐 유력 후보
- `inconclusive`: O2가 바뀌지 않음. 미사용 입력이거나 충전 안전 조건 미충족
- `UNSAFE/UNEXPECTED`: O0, O1 또는 O3가 변함. 충전 명령으로 사용하면 안 됨
- 이미 ON인 가상 입력과 기준 상태에서 이미 ON인 O2는 덮어쓰지 않고 중단

다음 후보는 이전 실행이 정상적으로 OFF 복구된 것을 확인한 뒤 별도 실행합니다.
전체 0~127 자동 스캔은 제공하지 않습니다.

명령 예시:

```bash
rostopic pub -1 /safety/charging std_msgs/Bool "data: true"
rostopic echo /safety/output/charge_port_on

rostopic pub -1 /safety/charging std_msgs/Bool "data: false"
```

## 토픽

아래 상태 토픽은 모두 퍼블리시 전용이며 마지막 값이 latch됩니다.

```text
/safety/connected                         std_msgs/Bool
/safety/state_all                         std_msgs/String

/safety/input/safety_emergency_1b         std_msgs/Bool
/safety/input/safety_emergency_2b         std_msgs/Bool
/safety/input/safety_auto_mode            std_msgs/Bool
/safety/input/safety_manual_mode          std_msgs/Bool
/safety/input/safety_reset_switch         std_msgs/Bool
/safety/input/safety_brake_release_sw     std_msgs/Bool
/safety/input/safety_bumper_front         std_msgs/Bool
/safety/input/safety_bumper_rear          std_msgs/Bool

/safety/output/motor_sto_01sr             std_msgs/Bool
/safety/output/motor_sto_02sr             std_msgs/Bool
/safety/output/charge_port_on              std_msgs/Bool
/safety/output/traction_motor_power_on     std_msgs/Bool
```

`connected`는 입력과 출력 블록을 한 주기에서 모두 정상적으로 읽었을 때만
`true`입니다. 충전 명령 기능이 활성화된 경우에는 watchdog 설정과 virtual input
쓰기까지 성공해야 `true`가 됩니다. 통신 실패 시 I/O 토픽의 이전 latch 값은
유지되므로, 소비 노드는 반드시 `connected`와 함께 확인해야 합니다.

## 빌드와 실행

```bash
cd catkin_ws
catkin_make
source devel/setup.bash

roslaunch safety_io_driver safety_io.launch
```

다른 IP로 임시 실행할 때는 launch 인자를 사용합니다.

```bash
roslaunch safety_io_driver safety_io.launch ip:=192.168.100.103
```

상태 확인:

```bash
rostopic echo /safety/connected
rostopic echo /safety/state_all
rostopic echo /safety/input/safety_bumper_front
rostopic echo /safety/output/traction_motor_power_on
```

## 현장 확인 순서

1. 로봇 PC NIC가 `192.168.100.0/24` 대역인지 확인합니다.
2. `ping 192.168.100.103`으로 네트워크 연결을 확인합니다.
3. `roslaunch` 후 `/safety/connected`가 `true`인지 확인합니다.
4. 장치 표시창의 I/O 상태와 ROS의 개별 토픽 및 mask를 하나씩 대조합니다.
5. 회로 또는 PNOZmulti 프로젝트가 변경되면 `config/safety_io.yaml`의 bit 매핑도
   함께 수정합니다.

Modbus/TCP에는 인증/암호화 기능이 없으므로 Safety PLC는 외부망과 분리된 기계망에
두고, 필요한 통신만 방화벽에서 허용하는 구성을 권장합니다.
