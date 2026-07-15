#!/usr/bin/env bash
set -e

INSTALL_DIR="${INSTALL_DIR:-$HOME/navifra/install}"

# --- ROS/워크스페이스 환경 (이미 .bashrc 에서 source 됐어도 무해) ---
if [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo "[run_robot] ERROR: ROS1 Noetic 미설치 (/opt/ros/noetic 없음)." >&2
    echo "            sudo apt install ros-noetic-ros-base ros-noetic-tf2-ros can-utils" >&2
    exit 1
fi
source /opt/ros/noetic/setup.bash

if [ ! -f "${INSTALL_DIR}/setup.bash" ]; then
    echo "[run_robot] ERROR: install 을 찾을 수 없습니다: ${INSTALL_DIR}/setup.bash" >&2
    echo "            개발기에서 deploy_to_robot.sh 로 rsync 했는지 확인하세요." >&2
    exit 1
fi
source "${INSTALL_DIR}/setup.bash"

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"

# --- CAN 은 caninit.service 가 올린다. 여기선 up 여부만 확인(경고). ---
CAN_DEV="${CAN_DEVICE:-can0}"
if ! ip -brief link show "${CAN_DEV}" 2>/dev/null | grep -qiw 'UP'; then
    echo "[run_robot] WARN: ${CAN_DEV} 가 up 상태가 아닙니다 (CAN 은 caninit.service 담당)."
    echo "            systemctl status caninit.service   # 상태 확인"
    echo "            sudo systemctl restart caninit.service"
fi

echo "[run_robot] ROS_MASTER_URI=$ROS_MASTER_URI  INSTALL=${INSTALL_DIR}  CAN=${CAN_DEV}"

# =============================================================================
#  실행할 서브시스템 (사용: true / 미사용: false)
#  ▶ 필요 없는 서브시스템은 값을 false 로 바꾸세요.
#     (편집:  nano ~/navifra/run_robot.sh   →  저장 Ctrl+O, 종료 Ctrl+X)
# =============================================================================
USE_DRIVE=true      # 구동부    : 모터 + 주행(/cmd_vel, /odom)
USE_BMS=true        # 배터리    : BMS 모니터링(/bms/state)        [CAN can1]
USE_LIFT=true       # 리프트    : 상승/하강(/lift/*)              [RS485 COM1]
USE_CREVIS=true     # 조명      : Crevis LED(/crevis/*)           [이더넷]
USE_SAFETY=true     # Safety PLC: 안전 I/O·충전(/safety/*)        [이더넷]
# =============================================================================

echo "[run_robot] drive=$USE_DRIVE bms=$USE_BMS lift=$USE_LIFT crevis=$USE_CREVIS safety=$USE_SAFETY"

# 켜둔 서브시스템만 단일 roscore 로 실행 (robot.launch 통합).
exec roslaunch motor_driver robot.launch \
    use_drive:=$USE_DRIVE \
    use_bms:=$USE_BMS \
    use_lift:=$USE_LIFT \
    use_crevis:=$USE_CREVIS \
    use_safety:=$USE_SAFETY
