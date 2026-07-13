#!/usr/bin/env bash
# =============================================================================
# 노트북(개발기) → 로봇 SSH 로 키보드 텔레옵 원터치 실행
#   ssh -t 로 원격 TTY 를 할당해 로봇의 ~/navifra/teleop.sh 를 실행한다.
#   (전제: deploy_to_robot.sh 로 install + teleop.sh 가 로봇 ~/navifra 에 배포돼 있어야 함)
#
# 사용:
#   ./teleop_ssh.sh <user>@<robot_ip> [teleop 인자...]
#   예) ./teleop_ssh.sh robot@192.168.0.50
#       ./teleop_ssh.sh robot@192.168.0.50 _linear_rpm:=200 _swap_lr:=true
# =============================================================================
set -e

DEST="${1:?사용: $0 <user>@<robot_ip> [teleop 인자...]}"
shift

exec ssh -t "${DEST}" "bash ~/navifra/teleop.sh $*"
