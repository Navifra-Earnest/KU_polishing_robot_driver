#!/usr/bin/env python3
# =============================================================================
# motor_driver 간이 키보드 컨트롤러 (ROS1 Noetic)
#   /motor/cmd (std_msgs/Float32MultiArray, data=[left_rpm, right_rpm]) 발행.
#   차동구동(differential drive): 두 바퀴 rpm 을 따로 계산해 전진/후진/선회/스핀턴.
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
#
#   ⚠️ 좌/우 매핑 가정: data[0]=모터ID1=왼쪽, data[1]=모터ID2=오른쪽.
#      실제로 반대면 swap_lr 파라미터를 true 로 (또는 아래 SWAP 기본값 변경).
#      드라이버가 모터2 방향을 내부 반전하므로, 여기선 두 바퀴 모두 "양수=전진".
# =============================================================================
import sys
import termios
import tty
import select

import rospy
from std_msgs.msg import Float32MultiArray

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
motor_driver 키보드 컨트롤러
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
    pub = rospy.Publisher('/motor/cmd', Float32MultiArray, queue_size=10)

    linear_rpm = float(rospy.get_param('~linear_rpm', 300.0))    # 전/후진 목표 rpm
    angular_rpm = float(rospy.get_param('~angular_rpm', 200.0))  # 선회 성분 rpm
    swap_lr = bool(rospy.get_param('~swap_lr', False))           # 좌/우 뒤바뀌면 true
    rate = rospy.Rate(10)  # 10Hz 재발행 → cmd_timeout(기본 1.0s) 안 걸리게 유지

    print(BANNER)
    print("[teleop] linear=%.0f rpm, angular=%.0f rpm" % (linear_rpm, angular_rpm))

    try:
        while not rospy.is_shutdown():
            key = get_key(settings, 0.1)

            if key in MOVE_BINDINGS:
                x, th = MOVE_BINDINGS[key]
            elif key == ' ':
                x, th = 0, 0
            elif key == KEY_UP:
                linear_rpm *= 1.1
                angular_rpm *= 1.1
                print("[teleop] speed+ : linear=%.0f, angular=%.0f" % (linear_rpm, angular_rpm))
                x, th = 0, 0
            elif key == KEY_DOWN:
                linear_rpm *= 0.9
                angular_rpm *= 0.9
                print("[teleop] speed- : linear=%.0f, angular=%.0f" % (linear_rpm, angular_rpm))
                x, th = 0, 0
            elif key == '\x03':  # Ctrl-C
                break
            else:
                # 무입력 또는 그 외 키: 정지
                x, th = 0, 0

            # 차동구동: 왼쪽/오른쪽 바퀴 rpm (둘 다 양수=전진)
            left = x * linear_rpm - th * angular_rpm
            right = x * linear_rpm + th * angular_rpm
            if swap_lr:
                left, right = right, left

            pub.publish(Float32MultiArray(data=[left, right]))
            rate.sleep()
    finally:
        # 종료 시 안전 정지 + 터미널 복구
        pub.publish(Float32MultiArray(data=[0.0, 0.0]))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
