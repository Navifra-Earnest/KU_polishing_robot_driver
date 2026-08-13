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
#   전제: motor_driver_node 가 이미 실행 중(systemd 등). teleop 은 /cmd_vel 만 발행한다.
#
# 인자는 그대로 전달:
#   bash ~/navifra/teleop.sh _linear_speed:=0.3 _angular_speed:=0.8   # [m/s], [rad/s]
#   bash ~/navifra/teleop.sh _key_hold:=0.65                          # 키 홀드 유지시간 [s]
#
# ── 키를 누르고 있는데 한 번 끊겼다 들어가는 현상 ──────────────────────────
#   원인은 로봇이 아니라 조종하는 노트북의 키보드 오토리피트다.
#   키를 누르면 1회 전송 → 리피트 지연(측정값 500ms) → 그 뒤 30Hz 반복.
#   설치본 teleop_keyboard.py 는 0.1s 무입력이면 즉시 정지 → 그 500ms 동안 멈춘다.
#   여기서는 마지막 키를 _key_hold 초 동안 유지(latch)해서 그 구멍을 메운다.
#   ⚠️ 대신 키를 떼도 _key_hold 초 뒤에 멈춘다 (기본 0.65s).
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

# 설치본 .py 를 건드리지 않고(재빌드 불필요) 홀드 기능이 들어간 버전을 임시 생성해 실행.
TELEOP_PY="$(mktemp /tmp/teleop_hold.XXXXXX.py)"
trap 'rm -f "$TELEOP_PY"' EXIT

cat > "$TELEOP_PY" <<'PYEOF'
#!/usr/bin/env python3
# teleop.sh 가 생성하는 임시 파일. 원본: motor_driver/scripts/teleop_keyboard.py
# 차이점: (1) 마지막 키를 key_hold 초 유지 → 오토리피트 지연 구멍 메움
#         (2) 입력 버퍼를 매 루프 전부 비움 → 키를 뗀 뒤 밀린 입력으로 더 가는 것 방지
import os
import sys
import time
import termios
import tty
import select

import rospy
from geometry_msgs.msg import Twist

MOVE_BINDINGS = {
    'u': (1,  1), 'i': (1,  0), 'o': (1, -1),
    'j': (0,  1), 'k': (0,  0), 'l': (0, -1),
    'm': (-1, 1), ',': (-1, 0), '.': (-1, -1),
}

KEY_UP = '\x1b[A'
KEY_DOWN = '\x1b[B'


def drain(settings, timeout):
    """timeout 초 대기 후, 버퍼에 쌓인 입력을 전부 읽어 문자열로 반환.
    ⚠️ os.read 로 fd 를 직접 읽는다. sys.stdin.read(1) 은 파이썬 내부 버퍼에
    나머지 바이트를 가둬버려서 select 는 '입력 없음' 이라 하고 문자는 유실된다
    (ESC 시퀀스가 쪼개지고 키가 씹히는 원인)."""
    fd = sys.stdin.fileno()
    tty.setraw(fd)
    buf = b''
    t = timeout
    while True:
        rlist, _, _ = select.select([fd], [], [], t)
        if not rlist:
            break
        chunk = os.read(fd, 1024)
        if not chunk:
            break
        buf += chunk
        t = 0.0
    termios.tcsetattr(fd, termios.TCSADRAIN, settings)
    return buf.decode('utf-8', 'ignore')


def main():
    rospy.init_node('motor_teleop_keyboard')

    if not sys.stdin.isatty():
        rospy.logerr("teleop_keyboard 는 TTY(키보드) 가 필요합니다. ssh -t 로 접속하세요.")
        return

    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    linear_speed = float(rospy.get_param('~linear_speed', 0.3))    # [m/s]
    # 0.5 는 체감상 선속도보다 빨라서 0.3 으로 내렸다(2026-08-11 현장 피드백).
    # 윤거 0.65m 기준 제자리회전 시 바퀴속도 = 0.3*0.325 = 0.0975 m/s (선속도 0.3 의 약 1/3).
    angular_speed = float(rospy.get_param('~angular_speed', 0.3))  # [rad/s]
    # ponytail: 노트북 오토리피트 지연(측정 500ms)보다 커야 구멍이 메워진다.
    # 다른 노트북에서 조종하면 `xset q | grep 'auto repeat delay'` 로 재보고 _key_hold 조정.
    key_hold = float(rospy.get_param('~key_hold', 0.65))           # [s]
    rate = rospy.Rate(20)  # base_controller 의 cmd_vel_timeout 보다 빠르게 재발행

    print("""\
------------------------------------------------
motor_driver 키보드 컨트롤러 (/cmd_vel)
   u  i  o     u:전진좌 i:전진 o:전진우
   j  k  l     j:반시계스핀 k:정지 l:시계스핀
   m  ,  .     m:후진좌 ,:후진 .:후진우
 space/k: 즉시 정지   ↑/↓: 속도+/-10%%   Ctrl-C: 종료
 ⚠️ 키를 떼면 %.2f초 뒤 정지 (오토리피트 끊김 보정). 즉시 정지는 space/k.
------------------------------------------------""" % key_hold)
    print("[teleop] linear=%.2f m/s, angular=%.2f rad/s, key_hold=%.2f s"
          % (linear_speed, angular_speed, key_hold))

    held = None       # 유지 중인 이동 키
    held_at = 0.0     # 마지막으로 그 키가 들어온 시각

    try:
        while not rospy.is_shutdown():
            buf = drain(settings, 0.02)

            if '\x03' in buf:  # Ctrl-C
                break

            # 방향키(ESC 시퀀스) 먼저 처리하고 버퍼에서 제거 — 남은 '[' 'A' 가 오인식되지 않게.
            for seq, factor, label in ((KEY_UP, 1.1, '+'), (KEY_DOWN, 0.9, '-')):
                n = buf.count(seq)
                if n:
                    buf = buf.replace(seq, '')
                    linear_speed *= factor ** n
                    angular_speed *= factor ** n
                    print("[teleop] speed%s : linear=%.2f m/s, angular=%.2f rad/s"
                          % (label, linear_speed, angular_speed))

            now = time.time()
            for ch in buf:
                if ch in MOVE_BINDINGS:
                    held, held_at = ch, now
                elif ch == ' ':
                    held = None

            if held is not None and (now - held_at) < key_hold:
                x, th = MOVE_BINDINGS[held]
            else:
                held = None
                x, th = 0, 0

            twist = Twist()
            twist.linear.x = x * linear_speed
            twist.angular.z = th * angular_speed
            pub.publish(twist)
            rate.sleep()
    finally:
        pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass  # rate.sleep() 중엔 Ctrl-C 가 SIGINT 로 오므로 트레이스백 대신 조용히 종료
PYEOF

python3 "$TELEOP_PY" "$@"
