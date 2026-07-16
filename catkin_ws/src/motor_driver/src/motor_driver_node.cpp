#include "motor_driver/motor_driver_node.hpp"

#include <sstream>

MotorDriverNode::MotorDriverNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh)
{
    // --- 파라미터 ---
    pnh_.param<std::string>("can_device", can_device_, "can0");
    pnh_.param<double>("control_frequency", control_frequency_, 50.0);
    pnh_.param<double>("cmd_timeout_sec", cmd_timeout_sec_, 1.0);
    pnh_.param<double>("feedback_timeout_sec", feedback_timeout_sec_, 0.5);
    pnh_.param<double>("fault_reset_interval_sec", fault_reset_interval_sec_, 2.0);
    pnh_.param<bool>("require_traction_enable", require_traction_enable_, false);
    pnh_.param<bool>("use_brake_interlock", use_brake_interlock_, false);

    std::vector<int> ids_param;
    if (!pnh_.getParam("drive_motor_ids", ids_param) || ids_param.empty()) {
        ids_param = {1, 2};
    }
    drive_motor_ids_.clear();
    for (int id : ids_param) {
        drive_motor_ids_.push_back(static_cast<uint8_t>(id));
    }

    // 모터별 방향 부호 (drive_motor_ids 와 평행). 미설정/크기불일치 시 전부 +1(무동작).
    std::vector<int> dir_param;
    motor_dir_.assign(drive_motor_ids_.size(), 1);
    if (pnh_.getParam("motor_directions", dir_param)) {
        if (dir_param.size() == drive_motor_ids_.size()) {
            for (size_t i = 0; i < dir_param.size(); ++i) motor_dir_[i] = (dir_param[i] < 0) ? -1 : 1;
        } else {
            ROS_WARN("motor_directions size (%zu) != drive_motor_ids (%zu) — ignored, all +1.",
                     dir_param.size(), drive_motor_ids_.size());
        }
    }

    // --- 컨트롤러 초기화 (CAN open + PDO mapping + sync thread) ---
    controller_ = std::make_shared<MotorController>();
    controller_->init(can_device_, drive_motor_ids_);

    // --- 통신 설정 ---
    cmd_sub_ = nh_.subscribe("/motor/cmd", 10, &MotorDriverNode::cmdCallback, this);
    // 긴급정지: /estop=true 면 즉시 0속도 + servo OFF(인터록 경유). false 로 명시적 해제.
    estop_sub_ = nh_.subscribe("/estop", 10, &MotorDriverNode::estopCallback, this);
    // TODO(traction): /traction_enable 은 아직 구성 전이라 구독 비활성화.
    //                 고객사 PLC 인가 신호 확정 후 주석 해제 (require_traction_enable 도 함께 true 로).
    // traction_enable_sub_ = nh_.subscribe("/traction_enable", 10, &MotorDriverNode::tractionEnableCallback, this);
    // TODO(brake): /motor_brakeon_feedback 은 아직 구성 전이라 구독 비활성화.
    //              브레이크 배선/토픽 확정 후 주석 해제 (use_brake_interlock 도 함께 true 로).
    // brake_fb_sub_ = nh_.subscribe("/motor_brakeon_feedback", 10, &MotorDriverNode::brakeFeedbackCallback, this);

    velocity_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/motor/velocity", 10);
    alarm_pub_ = nh_.advertise<std_msgs::String>("/motor/alarm", 10);
    error_pub_ = nh_.advertise<std_msgs::Bool>("/motor/error", 10);
    status_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/motor/status", 10);

    last_cmd_time_ = ros::Time::now();
    last_reset_attempt_ = ros::Time(0);

    control_timer_ = nh_.createTimer(
        ros::Duration(1.0 / control_frequency_),
        &MotorDriverNode::controlLoop, this);

    ROS_INFO("motor_driver initialized (can=%s, ids=%zu, require_traction_enable=%s, use_brake_interlock=%s)",
             can_device_.c_str(), drive_motor_ids_.size(),
             require_traction_enable_ ? "true" : "false",
             use_brake_interlock_ ? "true" : "false");

    // 인터록이 모두 비활성이면 시작 즉시 enable 되도록 초기 상태 갱신
    updateEnableState();
}

MotorDriverNode::~MotorDriverNode()
{
    if (controller_) {
        controller_->disable_motor();
    }
}

void MotorDriverNode::cmdCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() < drive_motor_ids_.size()) {
        ROS_WARN_THROTTLE(2.0, "/motor/cmd expects %zu values, got %zu — ignored.",
                          drive_motor_ids_.size(), msg->data.size());
        return;
    }
    last_cmd_time_ = ros::Time::now();

    // 안전: 긴급정지 중이면 지령 무시 (0 유지)
    if (estop_engaged_.load()) {
        return;
    }
    // 안전: enable 상태가 아니면 속도 지령을 적용하지 않고 0 유지
    if (!motors_enabled_.load()) {
        return;
    }
    for (size_t i = 0; i < drive_motor_ids_.size(); ++i) {
        controller_->set_target_velocity(drive_motor_ids_[i], motor_dir_[i] * msg->data[i]);
    }
}

// /traction_enable 미구성으로 콜백 비활성화 (위 subscribe 주석 참조).
// void MotorDriverNode::tractionEnableCallback(const std_msgs::Bool::ConstPtr& msg)
// {
//     last_traction_enable_cmd_.store(msg->data ? 1 : 0);
//     updateEnableState();
// }

// /motor_brakeon_feedback 미구성으로 콜백 비활성화 (위 subscribe 주석 참조).
// void MotorDriverNode::brakeFeedbackCallback(const std_msgs::Bool::ConstPtr& msg)
// {
//     brake_engaged_.store(msg->data);
//     updateEnableState();
// }

void MotorDriverNode::estopCallback(const std_msgs::Bool::ConstPtr& msg)
{
    const bool prev = estop_engaged_.exchange(msg->data);

    // 즉시 반영: 긴급정지 시 sync 스레드가 다음 주기에 0 을 내보내도록 목표속도부터 0 으로.
    // (servo OFF 로의 전이는 updateEnableState 가 백그라운드에서 수행 — 콜백 블로킹 방지)
    if (msg->data) {
        for (const auto& id : drive_motor_ids_) {
            controller_->set_target_velocity(id, 0.0f);
        }
        if (!prev) ROS_WARN("E-STOP engaged (/estop=true): motors -> 0 speed, servo OFF.");
    } else if (prev) {
        ROS_INFO("E-STOP released (/estop=false).");
    }

    updateEnableState();
}

bool MotorDriverNode::computeWantEnabled() const
{
    // 긴급정지 중이면 무조건 disable
    if (estop_engaged_.load()) {
        return false;
    }

    // traction 인가 조건
    bool traction_ok;
    if (require_traction_enable_) {
        traction_ok = (last_traction_enable_cmd_.load() == 1);
    } else {
        traction_ok = true;  // 인터록 미사용 → 항상 인가로 간주
    }

    // 브레이크 해제 조건 (브레이크 체결 시 구동 금지)
    bool brake_ok;
    if (use_brake_interlock_) {
        brake_ok = !brake_engaged_.load();
    } else {
        brake_ok = true;
    }

    return traction_ok && brake_ok;
}

void MotorDriverNode::updateEnableState()
{
    // edge-detect 상태(brake_fb_initialized_, last_brake_want_enabled_)와 전이 시작을
    // 여러 콜백 스레드로부터 보호한다.
    std::lock_guard<std::mutex> lock(traction_enable_mutex_);

    const bool want = computeWantEnabled();

    // edge detect: last_brake_want_enabled_ 는 "마지막으로 적용을 시작한(=커밋한) 목표".
    // 이미 그 목표와 같으면 할 일 없음.
    if (brake_fb_initialized_ && want == last_brake_want_enabled_) {
        return;
    }

    // enable/disable 는 CAN 상태전이(수백 ms~수 초 블로킹)를 수반하므로
    // 콜백/타이머를 막지 않도록 별도 스레드에서 수행한다.
    // 전이가 진행 중이면 목표를 아직 커밋하지 않고 반환 → 진행 중 스레드가
    // 종료 시 최신 상태로 재평가하여 이 변화를 반영한다(적용 목표 유실 방지).
    if (transition_in_progress_.exchange(true)) {
        return;
    }

    // 여기서부터 실제 전이를 커밋
    last_brake_want_enabled_ = want;
    brake_fb_initialized_ = true;

    std::thread([this, want]() {
        if (want) {
            ROS_INFO("Interlock satisfied -> enabling motors (Servo ON)");
            // 실제 OPERATION_ENABLED 도달 여부를 그대로 반영 (드라이브 FAULT 시 false).
            const bool ok = controller_->enable_motor();
            motors_enabled_.store(ok);
            if (!ok) {
                ROS_ERROR("Motor enable FAILED (drive not OPERATION_ENABLED). "
                          "Check drive fault/STO/hardware enable. motors_enabled=false.");
            }
        } else {
            ROS_INFO("Interlock not satisfied -> disabling motors (Servo OFF)");
            for (const auto& id : drive_motor_ids_) {
                controller_->set_target_velocity(id, 0.0f);
            }
            controller_->disable_motor();
            motors_enabled_.store(false);
        }
        transition_in_progress_.store(false);

        // 전이 중 인터록이 다시 바뀌었을 수 있으므로 재평가
        if (want != computeWantEnabled()) {
            updateEnableState();
        }
    }).detach();
}

void MotorDriverNode::controlLoop(const ros::TimerEvent&)
{
    // 긴급정지 → 매 주기 0속도 강제 (servo OFF 전이 완료 전에도 즉시 정지 보장)
    if (estop_engaged_.load()) {
        for (const auto& id : drive_motor_ids_) {
            controller_->set_target_velocity(id, 0.0f);
        }
        ROS_WARN_THROTTLE(2.0, "E-STOP engaged: drive motors held at zero.");
    }
    // 명령 타임아웃 → 안전 정지
    else if (motors_enabled_.load() &&
        (ros::Time::now() - last_cmd_time_) > ros::Duration(cmd_timeout_sec_)) {
        ROS_WARN_THROTTLE(5.0, "Command timeout. Holding drive motors at zero.");
        for (const auto& id : drive_motor_ids_) {
            controller_->set_target_velocity(id, 0.0f);
        }
    }

    // 실측 속도 발행 /motor/velocity  [m1_rpm, m2_rpm]
    std_msgs::Float32MultiArray velocity_msg;
    velocity_msg.data.reserve(drive_motor_ids_.size());
    for (size_t i = 0; i < drive_motor_ids_.size(); ++i) {
        // 방향 부호를 피드백에도 적용 → 명령과 같은 부호 규약 (base_controller odom 일관성)
        velocity_msg.data.push_back(motor_dir_[i] * controller_->get_feedback_velocity(drive_motor_ids_[i]));
    }
    velocity_pub_.publish(velocity_msg);

    // 상태 발행 /motor/status  모터별 [전압V, 전류raw, statusword, error_code] (순서 = drive_motor_ids)
    std_msgs::Float32MultiArray status_msg;
    status_msg.data.reserve(drive_motor_ids_.size() * 4);
    for (const auto& id : drive_motor_ids_) {
        MotorSnapshot snap = controller_->get_status(id);
        status_msg.data.push_back(static_cast<float>(snap.voltage));
        status_msg.data.push_back(snap.current_a);
        status_msg.data.push_back(static_cast<float>(snap.status_flags));
        status_msg.data.push_back(static_cast<float>(snap.error_code));
    }
    status_pub_.publish(status_msg);

    // 드라이브 에러코드 알람 확인/발행 (fault reset 대상)
    bool has_drive_fault = false;
    for (const auto& id : drive_motor_ids_) {
        auto alarm_text = controller_->get_alarm(id);
        if (alarm_text) {
            has_drive_fault = true;
            std_msgs::String alarm_msg;
            alarm_msg.data = *alarm_text;
            alarm_pub_.publish(alarm_msg);
        }
    }

    // 피드백(TPDO2) 타임아웃 알람 — CAN 통신 두절 등. 통신 문제이므로 fault reset 대상 아님.
    bool feedback_timeout = false;
    for (const auto& id : drive_motor_ids_) {
        if (controller_->get_feedback_age_sec(id) > feedback_timeout_sec_) {
            feedback_timeout = true;
            ROS_WARN_THROTTLE(2.0, "Motor %d feedback timeout (> %.2fs).", id, feedback_timeout_sec_);
            std_msgs::String alarm_msg;
            alarm_msg.data = "[" + std::to_string(id) + "] MOTOR_FEEDBACK_TIMEOUT";
            alarm_pub_.publish(alarm_msg);
        }
    }

    is_error_.store(has_drive_fault || feedback_timeout);

    // 에러 유무 발행 /motor/error
    std_msgs::Bool error_msg;
    error_msg.data = has_drive_fault || feedback_timeout;
    error_pub_.publish(error_msg);

    // 드라이브 fault 자동 리셋 (피드백 타임아웃은 리셋으로 해결 안 되므로 제외).
    //   reset_motor() 는 CAN 상태전이(수 초 블로킹)를 수반하므로 타이머(=발행 루프)를
    //   막지 않도록 별도 스레드에서 수행하고, fault_reset_interval_sec_ 로 재시도를
    //   rate-limit 한다. (그러지 않으면 지속 fault 시 매 주기 블로킹되어 모니터링 토픽이 멎음)
    const bool want_reset =
        has_drive_fault && !estop_engaged_.load() && computeWantEnabled() &&
        !transition_in_progress_.load() &&
        (ros::Time::now() - last_reset_attempt_).toSec() > fault_reset_interval_sec_;
    if (want_reset && !transition_in_progress_.exchange(true)) {
        last_reset_attempt_ = ros::Time::now();
        std::thread([this]() {
            const bool ok = controller_->reset_motor();
            motors_enabled_.store(ok);       // 리셋 후 실제 enable 여부 반영
            if (ok) {
                ROS_INFO("Motor fault reset successful.");
                is_error_.store(false);
            } else {
                ROS_ERROR_THROTTLE(2.0, "Motor fault reset failed.");
            }
            transition_in_progress_.store(false);
        }).detach();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_driver_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    MotorDriverNode node(nh, pnh);

    // 콜백(구독)과 타이머가 서로 블로킹되지 않도록 멀티스레드 스피너 사용
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
