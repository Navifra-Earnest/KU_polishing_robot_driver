#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <string>

// ============================================================================
// base_controller — 표준 이동로봇 인터페이스 계층 (차동구동, ROS1 Noetic)
//   저수준 motor_driver_node 위에 얹혀 상위(nav 스택 등)와 표준 메시지로 연동한다.
//     입력  /cmd_vel         geometry_msgs/Twist          v=linear.x, w=angular.z
//     출력  /motor/cmd       std_msgs/Float32MultiArray   [left_rpm, right_rpm] (모터축)
//     입력  /motor/velocity  std_msgs/Float32MultiArray   [left_rpm, right_rpm] 실측(모터축)
//     출력  /odom            nav_msgs/Odometry  + TF(odom->base_link)
//
//   좌/우는 대칭 처리한다: 전진 = 두 바퀴 모두 양수. (모터2 방향 반전은 드라이버가
//   이미 /motor/cmd·/motor/velocity 양쪽에서 처리 → 여기서 추가 반전 금지)
//   물리 매핑이 반대면 swap_lr 로만 뒤집는다.
// ============================================================================
class BaseController {
public:
    BaseController(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void velocityCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void controlTimer(const ros::TimerEvent&);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber velocity_sub_;
    ros::Publisher motor_cmd_pub_;
    ros::Publisher odom_pub_;
    ros::Timer control_timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // --- 파라미터 (기본값은 placeholder — 실제 로봇 값으로 설정 필수) ---
    double wheel_radius_{0.1};       // [m]
    double wheel_separation_{0.5};   // [m]
    double gear_ratio_{1.0};         // 모터축:바퀴 (｜/motor/cmd 는 모터축 rpm)
    double max_linear_vel_{0.0};     // [m/s], 0=제한 없음
    double max_angular_vel_{0.0};    // [rad/s], 0=제한 없음
    double control_rate_{50.0};      // [Hz] /motor/cmd 재발행 주기
    double cmd_vel_timeout_{0.5};    // [s] cmd_vel 미수신 시 정지
    std::string odom_frame_{"odom"};
    std::string base_frame_{"base_link"};
    bool publish_tf_{true};
    bool swap_lr_{false};

    // --- cmd_vel 상태 ---
    double target_v_{0.0};
    double target_w_{0.0};
    ros::Time last_cmd_vel_time_;

    // --- odom 적분 상태 ---
    double x_{0.0};
    double y_{0.0};
    double theta_{0.0};
    ros::Time last_odom_time_;
    bool odom_initialized_{false};

    static constexpr double kRpmToRadPerSec = 2.0 * M_PI / 60.0;
    static constexpr double kRadPerSecToRpm = 60.0 / (2.0 * M_PI);
};
