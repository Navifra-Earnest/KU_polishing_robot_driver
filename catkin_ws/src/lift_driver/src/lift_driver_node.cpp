#include "lift_driver/lift_driver_node.hpp"

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <sstream>

namespace {
// MDROBOT PID
constexpr uint8_t PID_ALARM_RESET  = 12;    // 제어기 알람상태 해제 (183,TMID,ID,12,1,x,CHK)
constexpr uint8_t PID_POSI_RESET   = 13;    // 현재위치를 0 으로 리셋 (원점화)
constexpr uint8_t PID_REQ_PID_DATA = 4;
constexpr uint8_t PID_VEL_CMD      = 130;   // 속도명령 (rpm, INT 2byte, 부호=방향)
constexpr uint8_t PID_TAR_POSI_VEL = 176;   // 위치제어 최대속도 (rpm, WORD 2byte)
constexpr uint8_t PID_MONITOR      = 196;   // rpm/current/status/position
constexpr uint8_t PID_POSI_DATA    = 197;
constexpr uint8_t PID_POSI_CMD     = 243;   // 절대 목표위치 (LONG 4byte, LSB first)
constexpr uint8_t PID_INC_POSI_CMD = 244;   // 상대 위치증분 (LONG 4byte, LSB first)

int16_t le16(const uint8_t* b) { return static_cast<int16_t>(b[0] | (b[1] << 8)); }
int32_t le32(const uint8_t* b) {
    return static_cast<int32_t>(static_cast<uint32_t>(b[0]) | (static_cast<uint32_t>(b[1]) << 8) |
                                (static_cast<uint32_t>(b[2]) << 16) | (static_cast<uint32_t>(b[3]) << 24));
}

// PID_CTRL_STATUS(34) BIT0 = ALARM(알람 유/무). 개별 원인은 BIT1~7 (decodeStatus 참조).
constexpr uint8_t STATUS_ALARM_BIT = 0x01;
}  // namespace

LiftDriver::LiftDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh)
{
    pnh_.param<std::string>("port", port_, "/dev/ttyUSB0");
    pnh_.param<int>("baud", baud_, 19200);
    int rmid = 183, host = 172, cid = 1;
    pnh_.param<int>("motor_rmid", rmid, 183);
    pnh_.param<int>("host_mid", host, 172);
    pnh_.param<int>("controller_id", cid, 1);
    motor_rmid_ = static_cast<uint8_t>(rmid);
    host_mid_ = static_cast<uint8_t>(host);
    ctrl_id_ = static_cast<uint8_t>(cid);
    pnh_.param<int>("up_speed_rpm", up_speed_rpm_, 0);
    pnh_.param<double>("poll_rate", poll_rate_, 10.0);
    pnh_.param<double>("timeout_sec", timeout_sec_, 1.0);

    pnh_.param<bool>("enable_position_control", enable_position_control_, true);
    pnh_.param<int>("posi_ctrl_vel_rpm", posi_ctrl_vel_rpm_, 0);
    pnh_.param<int>("home_speed_rpm", home_speed_rpm_, 0);
    pnh_.param<int>("home_stall_counts", home_stall_counts_, 5);
    pnh_.param<double>("home_stall_sec", home_stall_sec_, 1.5);
    pnh_.param<double>("home_min_sec", home_min_sec_, 1.0);
    pnh_.param<double>("home_timeout_sec", home_timeout_sec_, 40.0);
    pnh_.param<int>("in_position_tol", in_position_tol_, 20);
    pnh_.param<bool>("auto_home_on_start", auto_home_on_start_, false);
    pnh_.param<double>("post_home_speed_scale", post_home_speed_scale_, 2.0);
    if (post_home_speed_scale_ < 1.0) post_home_speed_scale_ = 1.0;   // 감속 배율은 허용 안 함(안전)

    pnh_.param<bool>("use_limit_switch_for_home", use_limit_switch_for_home_, true);
    pnh_.param<int>("limit_di_descend_mask", limit_di_descend_mask_, 0x04);
    pnh_.param<bool>("auto_release_on_stop", auto_release_on_stop_, true);
    pnh_.param<double>("release_grace_sec", release_grace_sec_, 2.0);
    pnh_.param<double>("release_stop_sec", release_stop_sec_, 1.0);

    pnh_.param<bool>("auto_reset_on_alarm", auto_reset_on_alarm_, true);
    pnh_.param<double>("alarm_reset_interval_sec", alarm_reset_interval_sec_, 2.0);
    pnh_.param<int>("alarm_reset_max_attempts", alarm_reset_max_attempts_, 5);
    pnh_.param<int>("alarm_resume_max_cycles", alarm_resume_max_cycles_, 3);
    if (alarm_reset_interval_sec_ < 0.1) alarm_reset_interval_sec_ = 0.1;
    if (alarm_reset_max_attempts_ < 0) alarm_reset_max_attempts_ = 0;   // 0 = 무제한
    if (alarm_resume_max_cycles_ < 0) alarm_resume_max_cycles_ = 0;     // 0 = 무제한

    if (up_speed_rpm_ == 0) {
        ROS_WARN("lift_driver: up_speed_rpm not set; 'up'/'down' will not move. "
                 "Set it in config/lift_driver.yaml (sign = up direction). velocity_cmd still works.");
    }

    cmd_sub_ = nh_.subscribe("/lift/command", 10, &LiftDriver::commandCallback, this);
    vel_cmd_sub_ = nh_.subscribe("/lift/velocity_cmd", 10, &LiftDriver::velocityCmdCallback, this);
    reset_sub_ = nh_.subscribe("/lift/reset", 1, &LiftDriver::resetCallback, this);
    position_pub_ = nh_.advertise<std_msgs::Int32>("/lift/position", 10);
    status_pub_ = nh_.advertise<std_msgs::String>("/lift/status", 10);
    error_pub_ = nh_.advertise<std_msgs::Bool>("/lift/error", 10);
    alarm_pub_ = nh_.advertise<std_msgs::String>("/lift/alarm", 10);

    if (enable_position_control_) {
        position_cmd_sub_ = nh_.subscribe("/lift/position_cmd", 10, &LiftDriver::positionCmdCallback, this);
        inc_position_cmd_sub_ = nh_.subscribe("/lift/inc_position_cmd", 10, &LiftDriver::incPositionCmdCallback, this);
        home_sub_ = nh_.subscribe("/lift/home", 1, &LiftDriver::homeCallback, this);
        homed_pub_ = nh_.advertise<std_msgs::Bool>("/lift/homed", 1, true);  // latched
    }

    serial_ = std::make_unique<SerialInterface>();
    if (!serial_->open(port_, baud_)) {
        ROS_ERROR("lift_driver: failed to open serial %s @ %d. Check device/permission (dialout).",
                  port_.c_str(), baud_);
    }

    // 위치제어 기준속도 (미설정 시 |up_speed_rpm| 사용; 둘 다 0이면 드라이브 기본 2000rpm)
    // 홈잉 전 = 기준속도, 홈잉 완료(homed) 후 = ×post_home_speed_scale (updateMotion 에서 재설정)
    if (enable_position_control_) {
        posi_base_vel_rpm_ = posi_ctrl_vel_rpm_ > 0 ? posi_ctrl_vel_rpm_ : std::abs(up_speed_rpm_);
        if (posi_base_vel_rpm_ > 0) {
            sendPositionVel(static_cast<uint16_t>(posi_base_vel_rpm_));
            ROS_INFO("lift_driver: position-control speed = %d rpm (homed 후 ×%.2f = %d rpm)",
                     posi_base_vel_rpm_, post_home_speed_scale_,
                     static_cast<int>(std::lround(posi_base_vel_rpm_ * post_home_speed_scale_)));
        } else {
            ROS_WARN("lift_driver: position-control speed unset -> drive default (2000rpm) applies to /lift/position_cmd. "
                     "Set posi_ctrl_vel_rpm or up_speed_rpm.");
        }
        std_msgs::Bool hb; hb.data = homed_; homed_pub_.publish(hb);
        pending_home_ = auto_home_on_start_;
    }

    last_rx_ = ros::Time(0);
    last_alarm_reset_ = ros::Time(0);
    poll_timer_ = nh_.createTimer(ros::Duration(1.0 / poll_rate_), &LiftDriver::pollTimer, this);
    ROS_INFO("lift_driver started: port=%s @ %d 8N1, ctrl_id=%d, up_speed=%d rpm, position_ctrl=%s "
             "(MDROBOT RS485, speed+limit)",
             port_.c_str(), baud_, ctrl_id_, up_speed_rpm_, enable_position_control_ ? "on" : "off");
    const std::string max_attempts_str =
        alarm_reset_max_attempts_ == 0 ? "unlimited" : std::to_string(alarm_reset_max_attempts_);
    ROS_INFO("lift_driver: alarm auto-reset=%s (interval=%.1fs, max_attempts=%s), manual reset = /lift/reset",
             auto_reset_on_alarm_ ? "on" : "off", alarm_reset_interval_sec_, max_attempts_str.c_str());
}

LiftDriver::~LiftDriver()
{
    if (serial_) { sendVelocity(0); serial_->close(); }  // 종료 시 정지
}

// 프레임: RMID(183) TMID(host) ID PID DataNum DATA... CHK, CHK=(~sum)+1
void LiftDriver::sendFrame(uint8_t pid, const std::vector<uint8_t>& data)
{
    if (!serial_ || !serial_->is_open()) return;
    std::vector<uint8_t> f;
    f.reserve(6 + data.size());
    f.push_back(motor_rmid_);
    f.push_back(host_mid_);
    f.push_back(ctrl_id_);
    f.push_back(pid);
    f.push_back(static_cast<uint8_t>(data.size()));
    for (uint8_t b : data) f.push_back(b);
    uint8_t sum = 0;
    for (uint8_t b : f) sum = static_cast<uint8_t>(sum + b);
    f.push_back(static_cast<uint8_t>(-static_cast<int>(sum)));  // (~sum)+1
    serial_->write(f.data(), f.size());
}

// PID_VEL_CMD(130): 2byte INT rpm (low byte first)
void LiftDriver::sendVelocity(int16_t rpm)
{
    // 지령 추적: 부호/시각이 바뀌면 정지판정 기준을 리셋 (releaseIfBlocked 에서 사용)
    if (rpm != last_vel_cmd_) {
        last_vel_cmd_ = rpm;
        vel_cmd_time_ = ros::Time::now();
        vel_ref_valid_ = false;
    }
    uint16_t r = static_cast<uint16_t>(rpm);
    sendFrame(PID_VEL_CMD, {static_cast<uint8_t>(r & 0xFF), static_cast<uint8_t>((r >> 8) & 0xFF)});
}

// DIR 비트(CTRL #6) = 하강 허용 입력. 0 이면 하부 리미트에 닿아 더 내려갈 수 없다.
// (실기 확정: 바닥 DIR=0 / 중간 DIR=1 / 하강 도달 순간 1→0. 상부 리미트는 이 입력에 없음)
bool LiftDriver::atLowerLimit(uint8_t di, bool fresh) const
{
    if (!fresh) return false;   // stale 데이터로 판정하지 않음
    return (di & static_cast<uint8_t>(limit_di_descend_mask_)) == 0;
}

// 수동 속도명령이 리미트/구속으로 멈췄으면 속도 0 을 보내 지령을 해제한다.
//   · 하강: DIR 비트로 즉시 판정 가능
//   · 상승: 드라이브 입력에 상부 리미트가 없어 '정지 감지'로만 알 수 있음
// 지령을 유지하면 드라이브가 CTRL_FAIL 을 내므로(상부 리미트 실측), 여기서 미리 끊는다.
void LiftDriver::releaseIfBlocked(int32_t pos, bool fresh, uint8_t di)
{
    if (!auto_release_on_stop_ || mode_ != Mode::MANUAL || last_vel_cmd_ == 0 || !fresh) return;

    const ros::Time now = ros::Time::now();

    if (last_vel_cmd_ < 0 && atLowerLimit(di, fresh)) {
        ROS_INFO("lift_driver: lower limit reached (DIR=0) - releasing down command (pos=%d).", pos);
        sendVelocity(0);
        return;
    }

    if ((now - vel_cmd_time_).toSec() < release_grace_sec_) return;   // 스핀업 여유

    if (!vel_ref_valid_ || std::abs(pos - vel_ref_pos_) > home_stall_counts_) {
        vel_ref_pos_ = pos;
        vel_ref_time_ = now;
        vel_ref_valid_ = true;
        return;
    }
    if ((now - vel_ref_time_).toSec() >= release_stop_sec_) {
        ROS_WARN("lift_driver: motion stopped at pos=%d while commanding %d rpm "
                 "(%s) - releasing command to avoid CTRL_FAIL.",
                 pos, last_vel_cmd_,
                 last_vel_cmd_ > 0 ? "upper limit or obstruction (no drive limit input on this axis)"
                                   : "obstruction");
        sendVelocity(0);
    }
}

// PID_POSI_CMD(243): 4byte LONG 절대 목표위치 (LSB first)
void LiftDriver::sendPosition(int32_t counts)
{
    uint32_t v = static_cast<uint32_t>(counts);
    sendFrame(PID_POSI_CMD, {static_cast<uint8_t>(v & 0xFF), static_cast<uint8_t>((v >> 8) & 0xFF),
                             static_cast<uint8_t>((v >> 16) & 0xFF), static_cast<uint8_t>((v >> 24) & 0xFF)});
}

// PID_INC_POSI_CMD(244): 4byte LONG 상대 이동량 (LSB first)
void LiftDriver::sendIncPosition(int32_t counts)
{
    uint32_t v = static_cast<uint32_t>(counts);
    sendFrame(PID_INC_POSI_CMD, {static_cast<uint8_t>(v & 0xFF), static_cast<uint8_t>((v >> 8) & 0xFF),
                                 static_cast<uint8_t>((v >> 16) & 0xFF), static_cast<uint8_t>((v >> 24) & 0xFF)});
}

// PID_TAR_POSI_VEL(176): 2byte WORD 위치제어 최대속도(rpm)
void LiftDriver::sendPositionVel(uint16_t rpm)
{
    sendFrame(PID_TAR_POSI_VEL, {static_cast<uint8_t>(rpm & 0xFF), static_cast<uint8_t>((rpm >> 8) & 0xFF)});
}

// PID_POSI_RESET(13): 현재위치를 0 으로 (DataNum=1, dummy 1byte)
void LiftDriver::sendPositionReset()
{
    sendFrame(PID_POSI_RESET, {0});
}

// PID_ALARM_RESET(12): 제어기 알람상태 해제 (DataNum=1, dummy 1byte)
void LiftDriver::sendAlarmReset()
{
    sendFrame(PID_ALARM_RESET, {0});
}

void LiftDriver::requestMonitor()
{
    sendFrame(PID_REQ_PID_DATA, {PID_MONITOR});
}

void LiftDriver::commandCallback(const std_msgs::String::ConstPtr& msg)
{
    const std::string& c = msg->data;
    if (c != "up" && c != "down" && c != "stop") {
        ROS_WARN_THROTTLE(2.0, "lift_driver: unknown command '%s' (use up/down/stop)", c.c_str());
        return;
    }
    mode_ = Mode::MANUAL;   // 수동속도 명령은 위치이동/홈잉을 취소
    cancelResume();         // 알람 해제 후 자동재개도 폐기(사용자 명령이 우선)
    if (c == "up") {
        sendVelocity(manualUpSpeed());       // 리미트에서 드라이브가 자동 정지 (homed면 2배)
        ROS_INFO("lift_driver: command=up (%d rpm)", manualUpSpeed());
    } else if (c == "down") {
        sendVelocity(static_cast<int16_t>(-manualUpSpeed()));
        ROS_INFO("lift_driver: command=down (%d rpm)", -manualUpSpeed());
    } else {
        sendVelocity(0);
        ROS_INFO("lift_driver: command=stop");
    }
}

void LiftDriver::velocityCmdCallback(const std_msgs::Int16::ConstPtr& msg)
{
    mode_ = Mode::MANUAL;   // 수동속도 명령은 위치이동/홈잉을 취소
    cancelResume();
    sendVelocity(msg->data);
    ROS_INFO("lift_driver: velocity_cmd=%d rpm", msg->data);
}

void LiftDriver::positionCmdCallback(const std_msgs::Int32::ConstPtr& msg)
{
    if (!homed_) {
        // 원점 미확립 상태의 절대위치 명령은 거부(무시). 상대이동은 /lift/inc_position_cmd 사용.
        ROS_WARN_THROTTLE(2.0, "lift_driver: not homed - absolute position_cmd IGNORED. "
                               "Publish /lift/home (true) once per power cycle first "
                               "(or use /lift/inc_position_cmd for relative moves).");
        return;
    }
    if (alarm_active_) {
        ROS_WARN("lift_driver: alarm active (%s) - position_cmd IGNORED. "
                 "Clear it first (/lift/reset) or wait for auto-reset.", decodeStatus(alarm_status_).c_str());
        return;
    }
    cancelResume();
    target_pos_ = msg->data;
    mode_ = Mode::POSITION;
    sendPosition(target_pos_);   // 드라이브가 위치제어를 자율 수행; 완료판정은 pollTimer
    ROS_INFO("lift_driver: position command -> target=%d counts", target_pos_);
}

void LiftDriver::incPositionCmdCallback(const std_msgs::Int32::ConstPtr& msg)
{
    if (alarm_active_) {
        ROS_WARN("lift_driver: alarm active (%s) - inc_position_cmd IGNORED. "
                 "Clear it first (/lift/reset) or wait for auto-reset.", decodeStatus(alarm_status_).c_str());
        return;
    }
    cancelResume();
    int32_t cur;
    { std::lock_guard<std::mutex> lock(data_mutex_); cur = position_; }
    target_pos_ = cur + msg->data;   // 완료판정용 예상 목표
    mode_ = Mode::POSITION;
    sendIncPosition(msg->data);
    ROS_INFO("lift_driver: inc position command -> %+d counts (expect target=%d)", msg->data, target_pos_);
}

void LiftDriver::homeCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (!msg->data) {   // 홈잉 중단
        pending_home_ = false;
        cancelResume();
        if (mode_ == Mode::HOMING) {
            mode_ = Mode::MANUAL;
            sendVelocity(0);
            ROS_WARN("lift_driver: homing aborted by /lift/home=false.");
        }
        return;
    }
    if (alarm_active_) {
        ROS_WARN("lift_driver: alarm active (%s) - homing request IGNORED. "
                 "Clear it first (/lift/reset) or wait for auto-reset.", decodeStatus(alarm_status_).c_str());
        return;
    }
    const int mag = home_speed_rpm_ > 0 ? home_speed_rpm_ : std::abs(up_speed_rpm_);
    if (mag == 0) {
        ROS_ERROR("lift_driver: homing needs a speed - set home_speed_rpm or up_speed_rpm.");
        return;
    }
    const int up_dir = (up_speed_rpm_ >= 0) ? 1 : -1;   // '상승' 부호
    home_dir_speed_ = static_cast<int16_t>(-up_dir * mag);   // 홈잉은 하강(상승 반대)
    mode_ = Mode::HOMING;
    home_start_ = ros::Time::now();
    { std::lock_guard<std::mutex> lock(data_mutex_); home_start_pos_ = position_; }
    home_log_time_ = home_start_;
    home_log_pos_ = home_start_pos_;
    home_alarm_seen_ = false;
    stall_tracking_ = false;
    pending_home_ = false;
    cancelResume();
    ROS_INFO("lift_driver: homing started (descend @ %d rpm, from pos=%d, expect lower limit auto-stop).",
             home_dir_speed_, home_start_pos_);
}

// /lift/reset (true) — 수동 알람 리셋. 자동 재시도 한계에 걸려 포기한 상태도 여기서 재무장된다.
void LiftDriver::resetCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (!msg->data) return;   // false 는 no-op (오발행 방지)

    alarm_reset_attempts_ = 0;
    alarm_reset_exhausted_ = false;
    resume_cycles_ = 0;   // 사용자 개입 → 자동 재개 예산도 초기화
    if (!alarm_active_) {
        // 알람이 없어도 요청대로 1회 전송한다(래치된 잔여 알람 정리용).
        ROS_INFO("lift_driver: /lift/reset - no active alarm; sending PID_ALARM_RESET anyway.");
        sendAlarmReset();
        last_alarm_reset_ = ros::Time::now();
        return;
    }
    ROS_INFO("lift_driver: /lift/reset - manual alarm reset (status=%s).", decodeStatus(alarm_status_).c_str());
    requestAlarmReset(true);
}

// 알람 감지 → 즉시정지 → (자동/수동) PID_ALARM_RESET → 해제되면 원래 동작 재개.
// ⚠️ pollTimer 에서 updateMotion 보다 먼저 호출해야 한다. HOMING 모드의 updateMotion 은
//    매 tick 하강 속도를 재전송하므로, 알람 tick 에 먼저 MANUAL 로 내려놓지 않으면
//    리셋 직후 같은 원인으로 다시 알람이 나는 루프에 빠진다.
void LiftDriver::updateAlarm(uint8_t st, bool fresh, int32_t pos)
{
    // stale 데이터로는 알람 판정/리셋을 하지 않는다 (통신 두절은 /lift/status 의 NO_DATA 로 표시).
    if (!fresh) return;

    const bool alarm = (st & STATUS_ALARM_BIT) != 0;

    if (alarm && !alarm_active_) {   // 알람 상승엣지
        alarm_active_ = true;
        alarm_status_ = st;
        alarm_reset_attempts_ = 0;
        alarm_reset_exhausted_ = false;
        last_alarm_reset_ = ros::Time(0);   // 첫 리셋은 이번 tick 에 즉시
        sendVelocity(0);                    // 알람 tick 에 즉시 정지

        const char* note;
        if (mode_ == Mode::HOMING) {
            // 홈잉 중 알람 = 홈잉 실패. CTRL_FAIL(속도제어 실패)은 리미트 도달 신호가 아니라
            // 실제 고장이므로, 정지 지점을 하부 리미트로 오판해 원점을 잡으면 절대위치가 전부 틀어진다.
            // → 원점을 잡지 않고 중단한다(homed 유지). 알람 리셋은 계속 수행.
            mode_ = Mode::MANUAL;
            home_alarm_seen_ = true;
            note = " during homing - homing ABORTED, origin NOT set.";
        } else {
            if (mode_ != Mode::MANUAL) {    // 진행 중이던 동작을 기억(해제 후 재개)
                resume_mode_ = mode_;
                resume_target_ = target_pos_;
            }
            mode_ = Mode::MANUAL;
            note = (resume_mode_ == Mode::POSITION) ? " position move will resume after clear." : "";
        }
        ROS_ERROR("lift_driver: ALARM detected (status=0x%02X %s) - stopped.%s",
                  st, decodeStatus(st).c_str(), note);
        if (home_alarm_seen_) {
            ROS_ERROR("lift_driver: homing failed at pos=%d (moved %+d from start=%d) - NOT homed. "
                      "Clear the cause, then re-run /lift/home.",
                      pos, pos - home_start_pos_, home_start_pos_);
        }
    } else if (alarm) {
        alarm_status_ = st;   // 원인 비트가 늘어날 수 있으므로 갱신
    }

    if (!alarm && alarm_active_) {   // 알람 하강엣지 = 해제됨
        alarm_active_ = false;
        alarm_status_ = st;
        ROS_INFO("lift_driver: alarm cleared (after %d reset attempt(s)).", alarm_reset_attempts_);
        alarm_reset_attempts_ = 0;
        alarm_reset_exhausted_ = false;
        resumeAfterAlarm(pos);
        return;
    }

    if (alarm_active_ && auto_reset_on_alarm_ && !alarm_reset_exhausted_) {
        const ros::Time now = ros::Time::now();
        if (last_alarm_reset_.isZero() ||
            (now - last_alarm_reset_).toSec() >= alarm_reset_interval_sec_) {
            requestAlarmReset(false);
        }
    }
}

// PID_ALARM_RESET 전송. 자동(manual=false)은 alarm_reset_max_attempts_ 로 제한한다
// (한계 초과 시 포기 → 원인 제거 후 /lift/reset 으로 재무장).
void LiftDriver::requestAlarmReset(bool manual)
{
    if (!manual && alarm_reset_max_attempts_ > 0 && alarm_reset_attempts_ >= alarm_reset_max_attempts_) {
        alarm_reset_exhausted_ = true;
        ROS_ERROR("lift_driver: alarm reset failed %d time(s) (status=%s) - auto-reset given up. "
                  "Fix the cause, then: rostopic pub -1 /lift/reset std_msgs/Bool \"data: true\"",
                  alarm_reset_attempts_, decodeStatus(alarm_status_).c_str());
        return;
    }
    sendVelocity(0);      // 리셋 직후 잔여 속도명령으로 재기동되지 않도록
    sendAlarmReset();
    last_alarm_reset_ = ros::Time::now();
    ++alarm_reset_attempts_;
    ROS_WARN("lift_driver: PID_ALARM_RESET sent (%s, attempt %d, status=%s).",
             manual ? "manual" : "auto", alarm_reset_attempts_, decodeStatus(alarm_status_).c_str());
}

// 알람 직전 동작 재개. 원점 미확립 상태에서는 절대명령을 못 쓰므로 상대이동으로 같은 목표에 복귀.
void LiftDriver::resumeAfterAlarm(int32_t pos)
{
    if (resume_mode_ == Mode::MANUAL) return;   // 재개할 것 없음

    // 재개 → 같은 원인으로 재알람 → 재개 … 무한사이클 방지
    if (alarm_resume_max_cycles_ > 0 && resume_cycles_ >= alarm_resume_max_cycles_) {
        ROS_ERROR("lift_driver: alarm recurred %d time(s) after resume - NOT resuming %s. "
                  "Fix the cause, then re-issue the command.",
                  resume_cycles_, resume_mode_ == Mode::HOMING ? "homing" : "position move");
        resume_mode_ = Mode::MANUAL;
        return;
    }
    ++resume_cycles_;

    if (resume_mode_ == Mode::HOMING) {
        resume_mode_ = Mode::MANUAL;
        mode_ = Mode::HOMING;
        home_start_ = ros::Time::now();   // 타임아웃 창 재시작 (재개 횟수는 alarm_resume_max_cycles 로 제한)
        home_start_pos_ = pos;            // 진단 기준점도 재개 지점으로
        home_log_time_ = home_start_;
        home_log_pos_ = pos;
        home_alarm_seen_ = false;
        stall_tracking_ = false;
        ROS_WARN("lift_driver: resuming homing after alarm clear (descend @ %d rpm).", home_dir_speed_);
        return;
    }

    resume_mode_ = Mode::MANUAL;
    target_pos_ = resume_target_;
    mode_ = Mode::POSITION;
    if (homed_) sendPosition(target_pos_);
    else        sendIncPosition(target_pos_ - pos);
    ROS_WARN("lift_driver: resuming position move after alarm clear (target=%d, pos=%d).", target_pos_, pos);
}

void LiftDriver::cancelResume()
{
    resume_cycles_ = 0;   // 사용자 개입 = 재개 사이클 카운터 초기화
    if (resume_mode_ == Mode::MANUAL) return;
    ROS_INFO("lift_driver: pending post-alarm resume (%s) cancelled by new command.",
             resume_mode_ == Mode::HOMING ? "HOMING" : "POSITION");
    resume_mode_ = Mode::MANUAL;
}

void LiftDriver::updateMotion(int32_t pos, bool fresh, uint8_t di)
{
    const ros::Time now = ros::Time::now();

    if (mode_ == Mode::HOMING) {
        // 진행상황 로그(5초 주기). "안 움직여서 멈춤"과 "통신두절로 멈춤"을 사후에 구분하기 위해
        // 위치·구간이동량과 fresh/stale 여부를 함께 남긴다.
        const double log_dt = (now - home_log_time_).toSec();
        if (log_dt >= 5.0) {
            if (fresh) {
                ROS_INFO("lift_driver: homing: pos=%d moved=%+d in %.1fs (fresh)",
                         pos, pos - home_log_pos_, log_dt);
            } else {
                ROS_WARN("lift_driver: homing: NO RS485 reply for %.1fs (stall detection paused, last pos=%d)",
                         log_dt, pos);
            }
            home_log_time_ = now;
            home_log_pos_ = pos;
        }

        if ((now - home_start_).toSec() > home_timeout_sec_) {
            sendVelocity(0);
            mode_ = Mode::MANUAL;
            resume_cycles_ = 0;   // 동작 종료 → 재개 사이클 카운터 초기화
            ROS_ERROR("lift_driver: homing timeout (%.0fs) - aborted at pos=%d (moved %+d from start=%d, "
                      "data=%s), NOT homed.",
                      home_timeout_sec_, pos, pos - home_start_pos_, home_start_pos_,
                      fresh ? "fresh" : "STALE");
            return;
        }
        // 하부 리미트 신호(DIR=0)가 최우선 판정 — 정지감지 1.5초 추론보다 정확하고 즉시 확정된다.
        // (중간에서 고장·구속으로 멈춘 것을 리미트로 오판해 엉뚱한 원점을 잡는 위험이 사라짐)
        if (use_limit_switch_for_home_ && atLowerLimit(di, fresh)) {
            sendVelocity(0);
            ROS_INFO("lift_driver: homing: lower limit switch reached (DIR=0) after %.1fs, "
                     "start=%d end=%d (moved %+d).",
                     (now - home_start_).toSec(), home_start_pos_, pos, pos - home_start_pos_);
            sendPositionReset();
            homed_ = true;
            mode_ = Mode::MANUAL;
            resume_cycles_ = 0;
            if (posi_base_vel_rpm_ > 0 && post_home_speed_scale_ > 1.0) {
                int scaled = static_cast<int>(std::lround(posi_base_vel_rpm_ * post_home_speed_scale_));
                if (scaled > 32767) scaled = 32767;
                sendPositionVel(static_cast<uint16_t>(scaled));
            }
            std_msgs::Bool hb; hb.data = true; homed_pub_.publish(hb);
            return;
        }

        sendVelocity(home_dir_speed_);   // 하강 유지 (리미트에서 드라이브 자동정지)
        if (!fresh) {
            // 위치데이터 stale → 정지감지 신뢰불가. 오검출 방지 위해 추적 리셋(타임아웃은 계속 감시)
            stall_tracking_ = false;
            return;
        }
        // 위치변화 정지 감지: pos 가 임계 이내로 머무는 시간이 home_stall_sec 이상이면 리미트 도달로 판정
        if (!stall_tracking_ || std::abs(pos - stall_ref_pos_) > home_stall_counts_) {
            stall_ref_pos_ = pos;
            stall_since_ = now;
            stall_tracking_ = true;
        } else if ((now - home_start_).toSec() >= home_min_sec_ &&
                   (now - stall_since_).toSec() >= home_stall_sec_) {
            sendVelocity(0);
            // 이동량/소요시간을 남긴다: moved≈0 이면 "이미 하부 리미트"였다는 뜻이고,
            // 전행정에 못 미치는 값이면 도중 정지를 리미트로 오판(원점이 엉뚱한 높이에 잡힘)한 것이다.
            ROS_INFO("lift_driver: homing stop detected: took %.1fs, start=%d end=%d (moved %+d), di=%s.",
                     (now - home_start_).toSec(), home_start_pos_, pos, pos - home_start_pos_,
                     decodeDi(di_byte_).c_str());
            sendPositionReset();   // 현재(하부 리미트) 위치를 원점 0 으로
            homed_ = true;
            mode_ = Mode::MANUAL;
            resume_cycles_ = 0;    // 정상 완료 → 재개 사이클 카운터 초기화
            // 홈잉 후 위치제어 속도를 배율 적용값으로 상향 (수동 up/down 은 manualUpSpeed 에서 처리)
            if (posi_base_vel_rpm_ > 0 && post_home_speed_scale_ > 1.0) {
                int scaled = static_cast<int>(std::lround(posi_base_vel_rpm_ * post_home_speed_scale_));
                if (scaled > 32767) scaled = 32767;
                sendPositionVel(static_cast<uint16_t>(scaled));
                ROS_INFO("lift_driver: homed - position reset to 0; speed scaled ×%.2f (position=%d rpm, manual up/down=%d rpm).",
                         post_home_speed_scale_, scaled, manualUpSpeed());
            } else {
                ROS_INFO("lift_driver: homed - lower limit reached, position reset to 0.");
            }
            std_msgs::Bool hb; hb.data = true; homed_pub_.publish(hb);
        }
    } else if (mode_ == Mode::POSITION) {
        if (fresh && std::abs(target_pos_ - pos) <= in_position_tol_) {
            mode_ = Mode::MANUAL;   // 목표 도달 (stale 데이터로는 판정 안 함)
            resume_cycles_ = 0;     // 정상 완료 → 재개 사이클 카운터 초기화
            ROS_INFO("lift_driver: reached target (target=%d, pos=%d).", target_pos_, pos);
        }
        // 위치제어는 드라이브가 자율 수행하므로 재전송하지 않음
    }
}

const char* LiftDriver::modeStr() const
{
    switch (mode_) {
        case Mode::HOMING:   return "HOMING";
        case Mode::POSITION: return "POSITION";
        default:             return "MANUAL";
    }
}

// 수동 up/down 속도: 홈잉 완료(homed) 후에는 post_home_speed_scale 배 (int16 포화)
int16_t LiftDriver::manualUpSpeed() const
{
    const double scale = homed_ ? post_home_speed_scale_ : 1.0;
    long v = std::lround(up_speed_rpm_ * scale);
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    return static_cast<int16_t>(v);
}

void LiftDriver::readAndParse()
{
    uint8_t tmp[256];
    int n = serial_->read(tmp, sizeof(tmp), 30);
    if (n > 0) rx_buf_.insert(rx_buf_.end(), tmp, tmp + n);

    // 프레임 스캔: [host_mid, 183, id, pid, num, data.., chk], sum(all)%256==0
    size_t i = 0;
    while (rx_buf_.size() >= i + 6) {
        if (rx_buf_[i] != host_mid_ || rx_buf_[i + 1] != motor_rmid_) { ++i; continue; }
        uint8_t num = rx_buf_[i + 4];
        size_t flen = 5 + num + 1;
        if (rx_buf_.size() < i + flen) break;
        uint8_t sum = 0;
        for (size_t k = 0; k < flen; ++k) sum = static_cast<uint8_t>(sum + rx_buf_[i + k]);
        if (sum == 0) {
            handleFrame(rx_buf_[i + 3], &rx_buf_[i + 5], num);
            i += flen;
        } else {
            ++i;
        }
    }
    if (i > 0) rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + i);
    if (rx_buf_.size() > 512) rx_buf_.clear();
}

void LiftDriver::handleFrame(uint8_t pid, const uint8_t* d, uint8_t n)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (pid == PID_MONITOR && n >= 12) {
        rpm_ = le16(d);
        status_byte_ = d[6];
        position_ = le32(d + 7);
        di_byte_ = d[11];      // D12: DI 입력 (리미트스위치 겸용 #6 DIR / #8 START_STOP 포함)
        got_data_ = true;
        last_rx_ = ros::Time::now();
    } else if (pid == PID_POSI_DATA && n >= 4) {
        position_ = le32(d);
        got_data_ = true;
        last_rx_ = ros::Time::now();
    }
}

std::string LiftDriver::decodeStatus(uint8_t st) const
{
    // PID_CTRL_STATUS(34) 비트. BIT0 = 알람 유/무(종합), BIT1~7 = 원인.
    std::string s;
    if (st & 0x01) s += "ALARM ";
    if (st & 0x02) s += "CTRL_FAIL ";
    if (st & 0x04) s += "OVER_VOLT ";
    if (st & 0x08) s += "OVER_TEMP ";
    if (st & 0x10) s += "OVER_LOAD ";
    if (st & 0x20) s += "HALL_FAIL ";
    if (st & 0x40) s += "INV_VEL ";
    if (st & 0x80) s += "STALL ";
    if (s.empty()) s = "OK";
    return s;
}

// PID_DI(48) 비트. 매뉴얼 §2.3: 통신구동 시 #6(DIR)=CW(-) 방향 허용, #8(START_STOP)=CCW(+) 방향 허용
// 이며, 리프트처럼 기구 구속된 축에서는 이 두 핀에 리미트스위치(NC)를 물린다.
// → 해당 비트가 0 이면 그 방향 끝단 리미트에 닿은 것(= 그 방향으로 더 못 감).
// ⚠️ 어느 비트가 상/하 리미트인지는 실기에서 확정해야 한다(하강 부호가 CW/CCW 중 무엇인지에 달림).
std::string LiftDriver::decodeDi(uint8_t di) const
{
    std::ostringstream ss;
    ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(di) << std::dec
       << "(DIR=" << ((di & 0x04) ? 1 : 0)              // #6  CW(-) 방향 허용 / 리미트
       << ",START_STOP=" << ((di & 0x10) ? 1 : 0)       // #8  CCW(+) 방향 허용 / 리미트
       << ")";
    return ss.str();
}

void LiftDriver::pollTimer(const ros::TimerEvent&)
{
    requestMonitor();
    readAndParse();

    int32_t pos; uint8_t st; uint8_t di; bool fresh;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pos = position_;
        st = status_byte_;
        di = di_byte_;
        fresh = got_data_ && (ros::Time::now() - last_rx_).toSec() < timeout_sec_;
    }

    // 알람 처리(감지·정지·리셋·재개)는 반드시 updateMotion 보다 먼저.
    updateAlarm(st, fresh, pos);

    // 자동 홈잉(옵션): 통신이 살아난 뒤 1회 시작 (알람 중에는 보류)
    if (pending_home_ && fresh && !alarm_active_) {
        std_msgs::BoolPtr hm(new std_msgs::Bool());
        hm->data = true;
        homeCallback(hm);   // pending_home_ 은 여기서 해제됨
    }

    updateMotion(pos, fresh, di);   // 홈잉/위치이동 상태머신
    releaseIfBlocked(pos, fresh, di);   // 리미트/구속으로 멈춘 수동 지령 해제 (CTRL_FAIL 예방)

    std_msgs::Int32 pmsg; pmsg.data = pos; position_pub_.publish(pmsg);

    const std::string status_text = decodeStatus(st);

    std_msgs::String smsg;
    std::ostringstream ss;
    ss << (fresh ? "connected" : "NO_DATA") << " mode=" << modeStr() << " homed=" << (homed_ ? 1 : 0)
       << " pos=" << pos;
    if (mode_ == Mode::POSITION) ss << " target=" << target_pos_;
    ss << " status=" << status_text << " di=" << decodeDi(di);
    if (alarm_active_) ss << " alarm=1 reset_tries=" << alarm_reset_attempts_
                          << (alarm_reset_exhausted_ ? " reset=GAVE_UP" : "");
    if (resume_mode_ != Mode::MANUAL) ss << " resume_pending=" << (resume_mode_ == Mode::HOMING ? "HOMING" : "POSITION");
    smsg.data = ss.str();
    status_pub_.publish(smsg);

    std_msgs::Bool emsg; emsg.data = alarm_active_; error_pub_.publish(emsg);
    if (alarm_active_) {
        std_msgs::String amsg;
        amsg.data = status_text;
        alarm_pub_.publish(amsg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lift_driver");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    LiftDriver node(nh, pnh);
    ros::spin();
    return 0;
}
