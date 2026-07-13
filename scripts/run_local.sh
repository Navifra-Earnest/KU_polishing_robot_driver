#!/usr/bin/env bash
# =============================================================================
# 로봇 PC 네이티브 실행 스크립트 (도커 미사용, ROS1 Noetic)
#   넘겨받은 install 폴더를 source 하여 motor_driver 를 바로 실행한다.
#   전제: 로봇 PC 에 ROS1 Noetic + can-utils 가 설치되어 있어야 함.
#
# 사용:
#   ./run_local.sh [install_경로]
#   (기본 install 경로: <이 스크립트의 상위>/catkin_ws/install)
#
# 환경변수:
#   ROS_MASTER_URI  기본 http://localhost:11311 (고객사 master 사용 시 그 주소로)
#   ROS_IP          이 로봇의 IP (다른 머신과 연동 시 필수)
#   CAN_AUTO_UP     1이면 canX 자동 up (sudo 필요), 0이면 이미 올려둔 것으로 간주
#   CAN_DEVICE      기본 can0
#   CAN_BITRATE     기본 500000 (드라이브 설정과 동일하게)
# =============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="${1:-${SCRIPT_DIR}/../catkin_ws/install}"

# --- ROS 환경 ---
if [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo "[run_local] ERROR: ROS1 Noetic 이 설치되어 있지 않습니다 (/opt/ros/noetic 없음)." >&2
    exit 1
fi
source /opt/ros/noetic/setup.bash

if [ ! -f "${INSTALL_DIR}/setup.bash" ]; then
    echo "[run_local] ERROR: install 을 찾을 수 없습니다: ${INSTALL_DIR}/setup.bash" >&2
    echo "            넘겨받은 install 경로를 인자로 지정하세요: ./run_local.sh /path/to/install" >&2
    exit 1
fi
source "${INSTALL_DIR}/setup.bash"

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
echo "[run_local] ROS_MASTER_URI=$ROS_MASTER_URI  ROS_IP=${ROS_IP:-<unset>}"

# --- CAN 자동 up ---
if [ "${CAN_AUTO_UP:-1}" = "1" ]; then
    CAN_DEV="${CAN_DEVICE:-can0}"
    CAN_BR="${CAN_BITRATE:-500000}"
    echo "[run_local] Bringing up ${CAN_DEV} @ ${CAN_BR} bps (sudo)"
    sudo ip link set "${CAN_DEV}" down 2>/dev/null || true
    sudo ip link set "${CAN_DEV}" type can bitrate "${CAN_BR}" || \
        echo "[run_local] WARN: 비트레이트 설정 실패 (이미 up 이거나 실제 CAN 디바이스 아님)."
    sudo ip link set "${CAN_DEV}" up || echo "[run_local] WARN: ${CAN_DEV} up 실패."
fi

# --- 실행 ---
exec roslaunch motor_driver motor_driver.launch
