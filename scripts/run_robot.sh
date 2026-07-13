#!/usr/bin/env bash
# =============================================================================
# 로봇 PC 수동/디버그 실행 래퍼 (ROS1 Noetic)
#   프로덕션은 systemd(scripts/systemd/motor_driver.service)로 돌린다.
#   이 스크립트는 수동/디버그 실행용 얇은 래퍼다.
#
#   역할 분담:
#     - CAN(can0) up  : caninit.service 담당 (이 스크립트는 CAN 을 건드리지 않음)
#     - 환경 source    : ~/.bashrc 에 넣어도 되나, 비대화형 셸 대비로 여기서도 방어적 source
#
# 사용:  bash ~/navifra/run_robot.sh
# 환경변수:
#   INSTALL_DIR     기본 $HOME/navifra/install
#   ROS_MASTER_URI  기본 http://localhost:11311 (고객사 master 사용 시 그 주소)
#   ROS_IP          이 로봇의 IP (다른 머신과 연동 시)
#   CAN_DEVICE      up 여부 확인용 인터페이스명 (기본 can0)
# =============================================================================
set -e

INSTALL_DIR="${INSTALL_DIR:-$HOME/navifra/install}"

# --- ROS/워크스페이스 환경 (이미 .bashrc 에서 source 됐어도 무해) ---
if [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo "[run_robot] ERROR: ROS1 Noetic 미설치 (/opt/ros/noetic 없음)." >&2
    echo "            sudo apt install ros-noetic-ros-base can-utils" >&2
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

# --- 실행 ---
exec roslaunch motor_driver motor_driver.launch
