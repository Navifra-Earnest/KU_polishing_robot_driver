#!/usr/bin/env bash
# =============================================================================
# motor_driver 컨테이너 엔트리포인트 (ROS1 Noetic, 개발기 테스트용)
#   1) ROS1 / 워크스페이스 환경 source
#   2) (옵션) SocketCAN 인터페이스 자동 up
#   3) 전달된 커맨드 실행 (기본: roslaunch)
# =============================================================================
set -e

# --- 1) ROS 환경 ---
source /opt/ros/noetic/setup.bash
if [ -f /catkin_ws/install/setup.bash ]; then
    source /catkin_ws/install/setup.bash
elif [ -f /catkin_ws/devel/setup.bash ]; then
    source /catkin_ws/devel/setup.bash
fi

# ROS1 네트워킹: master 위치와 자기 IP.
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
if [ -n "$ROS_IP" ]; then
    echo "[entrypoint] ROS_IP=$ROS_IP"
elif [ -n "$ROS_HOSTNAME" ]; then
    echo "[entrypoint] ROS_HOSTNAME=$ROS_HOSTNAME"
fi
echo "[entrypoint] ROS_MASTER_URI=$ROS_MASTER_URI"

# --- 2) (옵션) SocketCAN 자동 up ---
if [ "${CAN_AUTO_UP:-0}" = "1" ]; then
    CAN_DEV="${CAN_DEVICE:-can0}"
    CAN_BR="${CAN_BITRATE:-500000}"
    echo "[entrypoint] Bringing up ${CAN_DEV} @ ${CAN_BR} bps"
    ip link set "${CAN_DEV}" down 2>/dev/null || true
    if ip link set "${CAN_DEV}" type can bitrate "${CAN_BR}"; then
        ip link set "${CAN_DEV}" up || echo "[entrypoint] WARN: failed to bring up ${CAN_DEV}"
    else
        echo "[entrypoint] WARN: '${CAN_DEV}' 비트레이트 설정 실패 (이미 up 이거나 실제 CAN 디바이스 아님). 계속 진행."
    fi
    ip -details link show "${CAN_DEV}" 2>/dev/null || true
fi

exec "$@"
