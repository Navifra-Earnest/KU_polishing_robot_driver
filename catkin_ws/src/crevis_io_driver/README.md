# crevis_io_driver (ROS1 Noetic)

Crevis **GN-9289 (MODBUS TCP) 네트워크 어댑터** + **GT-225F 16-DO 모듈** 을 통해
폴리싱 로봇 LED 6개를 개별 ON/OFF 하고, Crevis 입력/출력 상태를 읽어 퍼블리시하는 드라이버.

토비카(`core_protocol/scripts/Crevis_QD.py`)에서 검증된 **pyModbusTCP 방식**을 그대로 사용한다.

---

## 1. 하드웨어 / IO 맵

| 장비 | 역할 |
|------|------|
| GN-9289 | MODBUS TCP 네트워크 어댑터 (IP 셋팅 대상) |
| GT-225F | 16점 디지털 출력 모듈 (OUT01, SLOT #01) |

출력 IO 맵 (`출력 IO.png`):

| I/O | bit | LED |
|-----|-----|-----|
| Y01.00 | 0 | VISION LED |
| Y01.01 | 1 | FRONT LED |
| Y01.02 | 2 | SIDE LED |
| Y01.03 | 3 | STATUS LED RED |
| Y01.04 | 4 | STATUS LED GREEN |
| Y01.05 | 5 | STATUS LED BLUE |

### MODBUS 레지스터 맵 (GN-9289 매뉴얼 REV1.06 p.28)

| 영역 | 시작주소 | Func Code | 용도 |
|------|----------|-----------|------|
| 출력 image **BIT (coil)** | `0x1000~` | 1/5/15 | **LED 개별 제어(coil 모드)** |
| 출력 image REGISTER | `0x0800~` | 3/16 | 16bit 묶음(register 모드) |
| 입력 image REGISTER | `0x0000~` | 3/4 | DI 읽기 |

- **coil 주소 = 0x1000 + bit** (첫 출력슬롯 기준). 첫 출력슬롯이면 `0x1000 = Y01.00`.
- 앞에 다른 출력슬롯이 있으면 `output_coil_base` 를 슬롯당 16씩 가산해서 맞춘다.
  (토비카는 출력모듈이 두 번째 슬롯이라 register 를 `0x0801` 에 썼음 — 슬롯 위치 주의!)

---

## 2. IP 셋팅 (GN-9289)

두 가지 방법. **로봇 서브넷에 맞는 IP** 를 정한 뒤 진행. (예: `192.168.0.41`)

### 방법 A — CREVIS Bootp Server (권장, 전체 IP/서브넷/게이트웨이 지정)
1. 어댑터 **DIP #9 ON** (BOOTP/DHCP enable) 후 전원 재인가.
   → 어댑터가 2초마다 20회 BOOTP 요청 송출.
2. PC 에서 **CREVIS IO Guide Pro** 의 `Bootp Server` 실행 → `Start Bootp`.
3. Request History 에 잡힌 MAC 더블클릭 → `Setup IP Address` 에서
   IP / Subnet / Gateway 입력 → `Ok`.
4. Setup History 에 반영되면 완료. `ping <설정IP>` 로 확인.
> BOOTP 서버 응답이 없으면 어댑터는 EEPROM 의 마지막 저장 IP 를 사용.

### 방법 B — DIP 스위치 수동 (마지막 옥텟만)
- **DIP #10 ON** = 수동 모드. **DIP #1~#8** 이진값으로 IP 마지막 옥텟 지정.
  - 예) `...1` → #1 ON,  `...2` → #2 ON,  `...253` → 253 이진 조합.
- 상위 3옥텟(예 `192.168.0.`)은 EEPROM 저장값을 따름 → 처음엔 방법 A 로 한 번 세팅 권장.

설정 후 `config/crevis_io.yaml` 의 `ip:` 를 실제 IP 로 맞춘다.

---

## 3. 토픽

### 쓰기 (LED 개별 제어) — `std_msgs/Bool`, True=ON / False=OFF
```
/crevis/led/vision
/crevis/led/front
/crevis/led/side
/crevis/led/status_red
/crevis/led/status_green
/crevis/led/status_blue
```

### 읽기
```
/crevis/led_state/<name>   std_msgs/Bool           출력 코일 readback (실제 상태)
/crevis/di                 std_msgs/Int32MultiArray  입력 레지스터 raw (16bit 워드)
/crevis/connected          std_msgs/Bool           Modbus 연결 상태
```

---

## 4. 실행 / 테스트

```bash
# 의존성 (최초 1회)
pip3 install pyModbusTCP

# 빌드 (컨테이너/로봇 catkin_ws 에서)
catkin_make            # 또는 catkin build
source devel/setup.bash

# 실행
roslaunch crevis_io_driver crevis_io.launch

# LED 켜기/끄기 테스트
rostopic pub -1 /crevis/led/front       std_msgs/Bool "data: true"
rostopic pub -1 /crevis/led/status_red  std_msgs/Bool "data: true"
rostopic pub -1 /crevis/led/front       std_msgs/Bool "data: false"

# 상태/데이터 확인
rostopic echo /crevis/connected
rostopic echo /crevis/led_state/front
rostopic echo /crevis/di
```

---

## 5. 트러블슈팅
- `connected: false` → IP/전원/배선 확인. `ping <ip>` 먼저.
- LED 가 안 켜짐(쓰기는 성공) → `output_coil_base` 슬롯 오프셋 의심.
  `rostopic echo /crevis/led_state/*` 로 readback 확인하며 base 를 16씩 조정.
- coil 모드가 안 먹으면 `write_mode: "register"` 로 전환(토비카식 16bit 묶음).
- 값이 안 바뀜 = 배선 미결선 가능성 (토비카에서도 동일 사례). 실물 배선 확인.
