#!/usr/bin/env bash
# =============================================================================
# 개발기 → 로봇 PC 로 install 스페이스를 rsync 배포.
#   대상 구조:  로봇:~/navifra/install  (+ run_robot.sh 동봉)
#   전제: 로봇에 ROS1 Noetic + can-utils, 동일 아키텍처(x86_64) 설치되어 있어야 함.
#         (install 은 ROS 런타임 라이브러리에 동적 링크되므로 자기완결 번들이 아님)
#
# 사용:
#   ./deploy_to_robot.sh <robot_user>@<robot_ip> [install_경로]
#   예) ./deploy_to_robot.sh robot@192.168.0.50
#       ./deploy_to_robot.sh robot@192.168.0.50 ~/Polishing/catkin_ws/install
#
# install 이 없으면 먼저 컨테이너/로컬에서 catkin_make install 로 생성하세요.
# =============================================================================
set -euo pipefail

DEST="${1:?사용: $0 <robot_user>@<robot_ip> [install_경로]}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="${2:-${SCRIPT_DIR}/../catkin_ws/install}"

if [ ! -f "${INSTALL_DIR}/setup.bash" ]; then
    echo "[deploy] ERROR: install 을 찾을 수 없습니다: ${INSTALL_DIR}/setup.bash" >&2
    echo "         먼저 빌드하세요 (devcontainer 안에서 catkin_make install) 또는" >&2
    echo "         install 경로를 두 번째 인자로 지정하세요." >&2
    exit 1
fi

echo "[deploy] 대상   : ${DEST}:~/navifra/install"
echo "[deploy] 원본   : ${INSTALL_DIR}"

# 1) 로봇에 ~/navifra 준비
ssh "${DEST}" 'mkdir -p ~/navifra'

# 2) install 을 폴더째 동기화 (--delete: 원본에서 지운 파일도 로봇에서 제거)
#    끝의 슬래시 주의: "install/" → 로봇의 ~/navifra/install/ 안으로 내용 동기화
rsync -avz --delete "${INSTALL_DIR}/" "${DEST}:navifra/install/"

# 3) 로봇 실행/텔레옵 스크립트 동봉
rsync -avz "${SCRIPT_DIR}/run_robot.sh" "${SCRIPT_DIR}/teleop.sh" "${DEST}:navifra/"

echo "[deploy] 완료."
echo "  드라이버 노드 (로봇):   bash ~/navifra/run_robot.sh   (또는 systemd)"
echo "  키보드 텔레옵 (노트북): ssh -t ${DEST} 'bash ~/navifra/teleop.sh'"
