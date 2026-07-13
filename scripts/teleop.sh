#!/usr/bin/env bash
# =============================================================================
# 로봇 PC 에서 키보드 텔레옵 실행 (SSH 세션용)
#   deploy_to_robot.sh 가 이 파일을 ~/navifra/ 에 함께 배포한다.
#
#   노트북에서 원격 조종:
#     ssh -t <user>@<robot_ip> 'bash ~/navifra/teleop.sh'
#     (⚠️ 반드시 ssh -t : 원격 TTY 를 할당해야 키 입력이 동작한다)
#   또는 노트북의 scripts/teleop_ssh.sh 로 원터치 실행.
#
#   전제: motor_driver_node 가 이미 실행 중(systemd 등). teleop 은 /motor/cmd 만 발행한다.
#
# 인자는 그대로 rosrun 에 전달:
#   bash ~/navifra/teleop.sh _linear_speed:=0.3 _angular_speed:=0.8   # [m/s], [rad/s]
# =============================================================================
set -e

INSTALL_DIR="${INSTALL_DIR:-$HOME/navifra/install}"

source /opt/ros/noetic/setup.bash
if [ -f "${INSTALL_DIR}/setup.bash" ]; then
    source "${INSTALL_DIR}/setup.bash"
else
    echo "[teleop] ERROR: install 없음: ${INSTALL_DIR}/setup.bash" >&2
    exit 1
fi

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
echo "[teleop] ROS_MASTER_URI=$ROS_MASTER_URI  (노드가 실행 중이어야 함)"

exec rosrun motor_driver teleop_keyboard.py "$@"
