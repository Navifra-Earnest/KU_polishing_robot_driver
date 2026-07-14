#include "motor_driver/base_controller_node.hpp"

#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

BaseController::BaseController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh)
{
    pnh_.param("wheel_radius", wheel_radius_, 0.1);
    pnh_.param("wheel_separation", wheel_separation_, 0.5);
    pnh_.param("gear_ratio", gear_ratio_, 1.0);
    pnh_.param("max_linear_vel", max_linear_vel_, 0.0);
    pnh_.param("max_angular_vel", max_angular_vel_, 0.0);
    pnh_.param("control_rate", control_rate_, 50.0);
    pnh_.param("cmd_vel_timeout", cmd_vel_timeout_, 0.5);
    pnh_.param<std::string>("odom_frame", odom_frame_, "odom");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param("publish_tf", publish_tf_, true);
    pnh_.param("swap_lr", swap_lr_, false);

    // 필수 물리 파라미터 미설정 시 큰 경고 (placeholder 기본값으로는 cmd_vel/odom 부정확)
    if (!pnh_.hasParam("wheel_radius") || !pnh_.hasParam("wheel_separation") || !pnh_.hasParam("gear_ratio")) {
        ROS_WARN("base_controller: wheel_radius/wheel_separation/gear_ratio not all set; using placeholder "
                 "defaults. cmd_vel/odom will be inaccurate until real values are set (config/base_controller.yaml).");
    }
    if (wheel_radius_ <= 0.0 || wheel_separation_ <= 0.0 || gear_ratio_ == 0.0) {
        ROS_ERROR("base_controller: require wheel_radius>0, wheel_separation>0, gear_ratio!=0 "
                  "(now r=%.4f, L=%.4f, G=%.4f). Publishing zero velocity on invalid values.",
                  wheel_radius_, wheel_separation_, gear_ratio_);
    }

    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &BaseController::cmdVelCallback, this);
    velocity_sub_ = nh_.subscribe("/motor/velocity", 10, &BaseController::velocityCallback, this);
    motor_cmd_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/motor/cmd", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);

    last_cmd_vel_time_ = ros::Time(0);  // 시작 시 stale → 정지 (cmd_vel 오기 전까지)

    control_timer_ = nh_.createTimer(
        ros::Duration(1.0 / control_rate_), &BaseController::controlTimer, this);

    ROS_INFO("base_controller started: r=%.3fm, L=%.3fm, G=%.2f, tf=%s, swap_lr=%s",
             wheel_radius_, wheel_separation_, gear_ratio_,
             publish_tf_ ? "on" : "off", swap_lr_ ? "true" : "false");
}

void BaseController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    target_v_ = msg->linear.x;
    target_w_ = msg->angular.z;
    last_cmd_vel_time_ = ros::Time::now();
}

void BaseController::controlTimer(const ros::TimerEvent&)
{
    double v = target_v_;
    double w = target_w_;

    // cmd_vel 타임아웃 → 정지 (상위가 발행을 멈추면 로봇도 멈춤)
    if ((ros::Time::now() - last_cmd_vel_time_).toSec() > cmd_vel_timeout_) {
        v = 0.0;
        w = 0.0;
    }
    // 속도 제한 (0=비활성)
    if (max_linear_vel_ > 0.0)  v = std::max(-max_linear_vel_, std::min(v, max_linear_vel_));
    if (max_angular_vel_ > 0.0) w = std::max(-max_angular_vel_, std::min(w, max_angular_vel_));

    // 파라미터 오류 시 안전상 0속도
    std_msgs::Float32MultiArray cmd;
    if (wheel_radius_ <= 0.0 || gear_ratio_ == 0.0) {
        cmd.data = {0.0f, 0.0f};
        motor_cmd_pub_.publish(cmd);
        return;
    }

    // 역기구학: (v,w) → 좌/우 바퀴 선속도[m/s] → 모터축 rpm
    const double v_l = v - w * wheel_separation_ / 2.0;
    const double v_r = v + w * wheel_separation_ / 2.0;
    const double rpm_l = (v_l / wheel_radius_) * kRadPerSecToRpm * gear_ratio_;
    const double rpm_r = (v_r / wheel_radius_) * kRadPerSecToRpm * gear_ratio_;

    // 좌/우는 대칭. 물리 매핑이 반대면 swap_lr 로만 뒤집는다.
    if (swap_lr_) cmd.data = {static_cast<float>(rpm_r), static_cast<float>(rpm_l)};
    else          cmd.data = {static_cast<float>(rpm_l), static_cast<float>(rpm_r)};
    motor_cmd_pub_.publish(cmd);
}

void BaseController::velocityCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() < 2) return;
    if (wheel_radius_ <= 0.0 || gear_ratio_ == 0.0) return;

    // 실측 모터축 rpm → 좌/우 (swap 반영: 발행과 대칭)
    double m_l = msg->data[0];
    double m_r = msg->data[1];
    if (swap_lr_) std::swap(m_l, m_r);

    // 모터 rpm → 바퀴 선속도[m/s] → (v, w)
    const double v_l = (m_l / gear_ratio_) * kRpmToRadPerSec * wheel_radius_;
    const double v_r = (m_r / gear_ratio_) * kRpmToRadPerSec * wheel_radius_;
    const double v = (v_l + v_r) / 2.0;
    const double w = (v_r - v_l) / wheel_separation_;

    const ros::Time now = ros::Time::now();
    if (!odom_initialized_) {
        last_odom_time_ = now;
        odom_initialized_ = true;
        return;  // 첫 콜백: 이전 시각 없음 → dt 계산 불가, 스킵
    }
    const double dt = (now - last_odom_time_).toSec();
    last_odom_time_ = now;
    if (dt <= 0.0) return;

    // 적분 (중간각 사용 → 회전 중 정확도 향상)
    const double dtheta = w * dt;
    const double theta_mid = theta_ + dtheta / 2.0;
    x_ += v * std::cos(theta_mid) * dt;
    y_ += v * std::sin(theta_mid) * dt;
    theta_ += dtheta;
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));  // [-pi, pi] 정규화

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    nav_msgs::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = w;
    odom_pub_.publish(odom);

    if (publish_tf_) {
        geometry_msgs::TransformStamped tf;
        tf.header.stamp = now;
        tf.header.frame_id = odom_frame_;
        tf.child_frame_id = base_frame_;
        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        tf_broadcaster_.sendTransform(tf);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    BaseController node(nh, pnh);

    ros::spin();
    return 0;
}
