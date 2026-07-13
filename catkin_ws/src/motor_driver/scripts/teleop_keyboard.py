#!/usr/bin/env python3
# =============================================================================
# motor_driver 간이 키보드 컨트롤러 (ROS1 Noetic)
#   /cmd_vel (geometry_msgs/Twist) 발행 — 선속도 v(linear.x) + 각속도 w(angular.z).
#   차동구동 역기구학/좌우 매핑은 base_controller 가 담당한다.
#   → 실행하려면 base_controller 가 떠 있어야 함 (roslaunch motor_driver bringup.launch).
#
#   키 배치 (teleop_twist_keyboard 스타일):
#       u  i  o        u=전진좌회전   i=전진      o=전진우회전
#       j  k  l        j=반시계 스핀   k=정지      l=시계 스핀
#       m  ,  .        m=후진좌회전   ,=후진      .=후진우회전
#
#       키를 누르고 있는 동안만 해당 속도 발행, 떼면 0
#       k 또는 space : 정지
#       ↑ / ↓        : 속도 +/- 10%
#       Ctrl-C       : 종료(정지 후)
# =============================================================================
import sys
import termios
import tty
import select

import rospy
from geometry_msgs.msg import Twist

# key -> (x, th): x=전후(+전진), th=선회(+반시계=좌)
MOVE_BINDINGS = {
    'u': (1,  1),   # 전진 좌회전
    'i': (1,  0),   # 전진
    'o': (1, -1),   # 전진 우회전
    'j': (0,  1),   # 반시계 스핀턴 (제자리 좌회전)
    'l': (0, -1),   # 시계 스핀턴 (제자리 우회전)
    'm': (-1, 1),   # 후진 좌회전
    ',': (-1, 0),   # 후진
    '.': (-1, -1),  # 후진 우회전
    'k': (0,  0),   # 정지
}

KEY_UP = '\x1b[A'
KEY_DOWN = '\x1b[B'

BANNER = """\
------------------------------------------------
motor_driver 키보드 컨트롤러 (/cmd_vel)
   u  i  o     u:전진좌 i:전진 o:전진우
   j  k  l     j:반시계스핀 k:정지 l:시계스핀
   m  ,  .     m:후진좌 ,:후진 .:후진우
 space/k: 정지   ↑/↓: 속도+/-10%   Ctrl-C: 종료
 (키를 떼면 자동 정지)
------------------------------------------------"""


def get_key(settings, timeout):
    """termios raw 모드로 키를 timeout(초) 안에 읽는다. 없으면 ''.
    방향키는 ESC 시퀀스(\\x1b[A/B) 로 읽는다."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if not rlist:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return ''
    key = sys.stdin.read(1)
    if key == '\x1b':
        # 방향키 등: ESC [ A/B/...
        for _ in range(2):
            rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
            if not rlist:
                break
            key += sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rospy.init_node('motor_teleop_keyboard')

    # 키보드 입력에는 TTY 가 필요하다. roslaunch 는 노드 stdin 을 터미널로 주지 않으므로
    # launch-prefix="xterm -e" 없이 띄우면 여기서 걸러 크래시(termios) 대신 안내 후 종료.
    if not sys.stdin.isatty():
        rospy.logerr("teleop_keyboard 는 TTY(키보드) 가 필요합니다. "
                     "roslaunch 사용 시 launch-prefix=\"xterm -e\"(X 필요), "
                     "헤드리스(로봇)에서는 SSH 터미널에서 "
                     "'rosrun motor_driver teleop_keyboard.py' 로 따로 실행하세요.")
        return

    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    linear_speed = float(rospy.get_param('~linear_speed', 0.3))    # [m/s] 전/후진 속도
    angular_speed = float(rospy.get_param('~angular_speed', 0.5))  # [rad/s] 선회 속도
    rate = rospy.Rate(10)  # 10Hz 재발행 → base_controller 의 cmd_vel_timeout 안 걸리게 유지

    print(BANNER)
    print("[teleop] linear=%.2f m/s, angular=%.2f rad/s" % (linear_speed, angular_speed))

    try:
        while not rospy.is_shutdown():
            key = get_key(settings, 0.1)

            if key in MOVE_BINDINGS:
                x, th = MOVE_BINDINGS[key]
            elif key == ' ':
                x, th = 0, 0
            elif key == KEY_UP:
                linear_speed *= 1.1
                angular_speed *= 1.1
                print("[teleop] speed+ : linear=%.2f m/s, angular=%.2f rad/s" % (linear_speed, angular_speed))
                x, th = 0, 0
            elif key == KEY_DOWN:
                linear_speed *= 0.9
                angular_speed *= 0.9
                print("[teleop] speed- : linear=%.2f m/s, angular=%.2f rad/s" % (linear_speed, angular_speed))
                x, th = 0, 0
            elif key == '\x03':  # Ctrl-C
                break
            else:
                # 무입력 또는 그 외 키: 정지
                x, th = 0, 0

            twist = Twist()
            twist.linear.x = x * linear_speed
            twist.angular.z = th * angular_speed
            pub.publish(twist)
            rate.sleep()
    finally:
        # 종료 시 안전 정지 + 터미널 복구
        pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
