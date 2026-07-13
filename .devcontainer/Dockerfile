# =============================================================================
# motor_driver 개발용 devcontainer 이미지 (ROS1 Noetic, x86_64)
#   * 용도: VS Code "Dev Containers" 로 컨테이너에 진입해 catkin_make 로 반복 빌드/디버깅.
#   * 소스는 이미지에 굽지 않고 bind-mount 한다(devcontainer.json 의 workspaceMount).
#     → 호스트 편집이 즉시 반영, 컨테이너 안에서 catkin_make install 만 다시 실행.
#   * 배포용 install 산출물은 상위 Dockerfile(빌드 시 COPY + catkin_make)로 별도 생성.
# =============================================================================
FROM ros:noetic-ros-base

ARG USERNAME=navifra
ARG USER_UID=1000
ARG USER_GID=1000

# --- 개발/빌드 도구 + CAN 유틸 ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    gdb \
    ccache \
    clang-format \
    python3-catkin-tools \
    python3-rosdep \
    can-utils \
    iproute2 \
    sudo \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

# --- 호스트 UID/GID(1000) 와 맞춘 비루트 사용자 (bind-mount 권한 문제 방지) ---
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# --- 셸 진입 시 ROS/워크스페이스 환경 자동 source ---
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f /workspaces/Polishing/catkin_ws/devel/setup.bash ] && source /workspaces/Polishing/catkin_ws/devel/setup.bash" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /workspaces/Polishing
