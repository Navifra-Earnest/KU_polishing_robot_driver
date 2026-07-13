# =============================================================================
# motor_driver 테스트/빌드 이미지 (ROS1 Noetic, x86_64)
#   * 용도: "내 PC(개발기)"에서 컨테이너로 테스트.  로봇 PC 는 도커 없이 로컬 실행.
#   * catkin_make install 로 /catkin_ws/install 산출물 생성 → 로봇에 넘길 install.
# =============================================================================
FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    can-utils \
    iproute2 \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws
COPY catkin_ws/src ./src

# 의존성 설치 후 install 스페이스로 빌드
RUN . /opt/ros/noetic/setup.sh && \
    rosdep install --from-paths src --ignore-src -y || true && \
    catkin_make install

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
# roslaunch 는 master(roscore)가 없으면 자동으로 띄운다.
CMD ["roslaunch", "motor_driver", "motor_driver.launch"]
