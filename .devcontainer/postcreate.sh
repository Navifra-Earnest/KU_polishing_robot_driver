#!/usr/bin/env bash
# =============================================================================
# devcontainer 생성 후 1회 실행 (postCreateCommand).
#   1) rosdep 의존성 설치 (best-effort)
#   2) 최초 catkin_make install 빌드
# =============================================================================
set -e

source /opt/ros/noetic/setup.bash

WS=/workspaces/Polishing/catkin_ws

# --- rosdep (이미 init 되어 있으면 무시) ---
sudo rosdep init 2>/dev/null || true
rosdep update || true
rosdep install --from-paths "${WS}/src" --ignore-src -y || true

# --- 최초 빌드 (install 스페이스) ---
cd "${WS}"
catkin_make install

echo "[postcreate] 완료. 이후 빌드는 VS Code task 'ROS: build (install)' (Ctrl+Shift+B) 또는:"
echo "             cd ${WS} && source /opt/ros/noetic/setup.bash && catkin_make install"
