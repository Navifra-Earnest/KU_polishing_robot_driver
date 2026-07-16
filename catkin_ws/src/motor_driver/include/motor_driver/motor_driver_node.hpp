#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "motor_driver/motor_controller.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ============================================================================
// ROS1 Noetic 구동모터(속도 제어 2개) 드라이버 노드.
//   외부(고객사) 연동은 표준 메시지만 사용한다.
//     입력  /motor/cmd               std_msgs/Float32MultiArray  data=[m1_rpm, m2_rpm]
//     입력  /traction_enable         std_msgs/Bool               구동 인가 지령 (현재 구독 주석)
//     입력  /motor_brakeon_feedback  std_msgs/Bool               브레이크 체결 피드백 (현재 구독 주석)
//     출력  /motor/velocity          std_msgs/Float32MultiArray  data=[m1_rpm, m2_rpm] (실측 속도)
//     출력  /motor/alarm             std_msgs/String             알람 메시지 (에러 시)
//     출력  /motor/error             std_msgs/Bool               에러 유무 플래그
//     출력  /motor/status            std_msgs/Float32MultiArray  모터별 [전압V,전류raw,statusword,error_code]
//
//   안전 인터록: 모터는 (traction 인가) && (브레이크 해제) 일 때만 enable 한다.
//   traction_enable / brake feedback 은 값이 "변할 때만"(edge) enable/disable 을
//   호출한다(매 주기 재호출 방지 — PLC/safety_locked 처리와 동일 패턴).
// ============================================================================
class MotorDriverNode {
public:
    explicit MotorDriverNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~MotorDriverNode();

private:
    // 콜백
    void cmdCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void estopCallback(const std_msgs::Bool::ConstPtr& msg);  // /estop (소프트웨어 긴급정지)
    // void tractionEnableCallback(const std_msgs::Bool::ConstPtr& msg);  // /traction_enable 미구성 (주석)
    // void brakeFeedbackCallback(const std_msgs::Bool::ConstPtr& msg);  // /motor_brakeon_feedback 미구성 (주석)
    void controlLoop(const ros::TimerEvent&);

    // 인터록 상태 계산 및 edge 기반 enable/disable
    bool computeWantEnabled() const;
    void updateEnableState();

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::shared_ptr<MotorController> controller_;
    std::vector<uint8_t> drive_motor_ids_;
    std::vector<int> motor_dir_;   // 모터별 방향 부호(+1/-1), drive_motor_ids_ 와 평행. 명령·피드백에 곱함

    // 파라미터
    std::string can_device_;
    double control_frequency_{50.0};
    double cmd_timeout_sec_{1.0}; // 명령 타임아웃 시간 (초)
    double feedback_timeout_sec_{0.5}; // 피드백 미수신 시 MOTOR_FEEDBACK_TIMEOUT 알람 (초)
    double fault_reset_interval_sec_{2.0}; // 드라이브 fault 자동 리셋 재시도 최소 간격 (초)
    bool require_traction_enable_{false};   // true면 traction 지령 전까지 구동 금지
    bool use_brake_interlock_{false};       // true면 브레이크 해제 전까지 구동 금지

    // 통신
    ros::Subscriber cmd_sub_;
    ros::Subscriber estop_sub_;      // /estop  긴급정지 지령 (true=정지, false=해제)
    // ros::Subscriber traction_enable_sub_;  // /traction_enable 미구성 (주석)
    // ros::Subscriber brake_fb_sub_;  // /motor_brakeon_feedback 미구성 (주석)
    ros::Publisher velocity_pub_;   // /motor/velocity  실측 속도 [m1_rpm, m2_rpm]
    ros::Publisher alarm_pub_;       // /motor/alarm     알람 문자열
    ros::Publisher error_pub_;       // /motor/error     에러 유무 (Bool)
    ros::Publisher status_pub_;      // /motor/status    [voltage,current,statusword,error_code] × 모터
    ros::Timer control_timer_;

    ros::Time last_cmd_time_;
    ros::Time last_reset_attempt_;   // 마지막 fault reset 시도 시각 (재시도 rate-limit용, 타이머 스레드 전용)
    std::atomic<bool> motors_enabled_{false};
    std::atomic<bool> transition_in_progress_{false};
    std::atomic<bool> is_error_{false};
    std::atomic<bool> estop_engaged_{false};  // /estop 긴급정지 상태

    // traction enable 지령 (-1: 미수신, 0: disable, 1: enable)
    std::atomic<int> last_traction_enable_cmd_{-1};
    std::mutex traction_enable_mutex_;

    // motor_brakeon_feedback edge detect (PLC/safety_locked 와 동일하게 변화 시에만 동작)
    std::atomic<bool> brake_engaged_{false};

    // enable/disable edge detect 상태
    bool brake_fb_initialized_{false};
    bool last_brake_want_enabled_{false};
};
