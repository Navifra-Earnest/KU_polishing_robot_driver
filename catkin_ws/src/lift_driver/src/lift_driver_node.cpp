#include "lift_driver/lift_driver_node.hpp"

#include <cmath>
#include <cstdlib>
#include <sstream>

namespace {
// MDROBOT PID
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

    if (up_speed_rpm_ == 0) {
        ROS_WARN("lift_driver: up_speed_rpm not set; 'up'/'down' will not move. "
                 "Set it in config/lift_driver.yaml (sign = up direction). velocity_cmd still works.");
    }

    cmd_sub_ = nh_.subscribe("/lift/command", 10, &LiftDriver::commandCallback, this);
    vel_cmd_sub_ = nh_.subscribe("/lift/velocity_cmd", 10, &LiftDriver::velocityCmdCallback, this);
    estop_sub_ = nh_.subscribe("/estop", 10, &LiftDriver::estopCallback, this);  // 모터와 공유
    position_pub_ = nh_.advertise<std_msgs::Int32>("/lift/position", 10);
    status_pub_ = nh_.advertise<std_msgs::String>("/lift/status", 10);

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
    poll_timer_ = nh_.createTimer(ros::Duration(1.0 / poll_rate_), &LiftDriver::pollTimer, this);
    ROS_INFO("lift_driver started: port=%s @ %d 8N1, ctrl_id=%d, up_speed=%d rpm, position_ctrl=%s "
             "(MDROBOT RS485, speed+limit)",
             port_.c_str(), baud_, ctrl_id_, up_speed_rpm_, enable_position_control_ ? "on" : "off");
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
    uint16_t r = static_cast<uint16_t>(rpm);
    sendFrame(PID_VEL_CMD, {static_cast<uint8_t>(r & 0xFF), static_cast<uint8_t>((r >> 8) & 0xFF)});
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

void LiftDriver::requestMonitor()
{
    sendFrame(PID_REQ_PID_DATA, {PID_MONITOR});
}

void LiftDriver::estopCallback(const std_msgs::Bool::ConstPtr& msg)
{
    const bool prev = estop_engaged_.exchange(msg->data);
    if (msg->data) {
        mode_ = Mode::MANUAL;   // 진행 중 위치이동/홈잉 취소
        pending_home_ = false;
        sendVelocity(0);   // 즉시 정지
        if (!prev) ROS_WARN("lift_driver: E-STOP engaged (/estop=true) -> lift stopped, commands ignored.");
    } else if (prev) {
        ROS_INFO("lift_driver: E-STOP released (/estop=false).");
    }
}

void LiftDriver::commandCallback(const std_msgs::String::ConstPtr& msg)
{
    if (estop_engaged_.load()) {
        sendVelocity(0);
        ROS_WARN_THROTTLE(2.0, "lift_driver: E-STOP engaged, ignoring '%s'.", msg->data.c_str());
        return;
    }
    const std::string& c = msg->data;
    if (c == "up") {
        mode_ = Mode::MANUAL;   // 수동속도 명령은 위치이동/홈잉을 취소
        sendVelocity(manualUpSpeed());       // 리미트에서 드라이브가 자동 정지 (homed면 2배)
    } else if (c == "down") {
        mode_ = Mode::MANUAL;
        sendVelocity(static_cast<int16_t>(-manualUpSpeed()));
    } else if (c == "stop") {
        mode_ = Mode::MANUAL;
        sendVelocity(0);
    } else {
        ROS_WARN_THROTTLE(2.0, "lift_driver: unknown command '%s' (use up/down/stop)", c.c_str());
    }
}

void LiftDriver::velocityCmdCallback(const std_msgs::Int16::ConstPtr& msg)
{
    if (estop_engaged_.load()) {
        sendVelocity(0);
        return;
    }
    mode_ = Mode::MANUAL;   // 수동속도 명령은 위치이동/홈잉을 취소
    sendVelocity(msg->data);
}

void LiftDriver::positionCmdCallback(const std_msgs::Int32::ConstPtr& msg)
{
    if (estop_engaged_.load()) {
        ROS_WARN_THROTTLE(2.0, "lift_driver: E-STOP engaged, ignoring position_cmd.");
        return;
    }
    if (!homed_) {
        ROS_WARN_THROTTLE(2.0, "lift_driver: not homed - target is relative to power-on origin. "
                               "Publish /lift/home (true) once per power cycle for repeatable absolute positioning.");
    }
    target_pos_ = msg->data;
    mode_ = Mode::POSITION;
    sendPosition(target_pos_);   // 드라이브가 위치제어를 자율 수행; 완료판정은 pollTimer
    ROS_INFO("lift_driver: position command -> target=%d counts", target_pos_);
}

void LiftDriver::incPositionCmdCallback(const std_msgs::Int32::ConstPtr& msg)
{
    if (estop_engaged_.load()) {
        ROS_WARN_THROTTLE(2.0, "lift_driver: E-STOP engaged, ignoring inc_position_cmd.");
        return;
    }
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
        if (mode_ == Mode::HOMING) {
            mode_ = Mode::MANUAL;
            sendVelocity(0);
            ROS_WARN("lift_driver: homing aborted by /lift/home=false.");
        }
        return;
    }
    if (estop_engaged_.load()) {
        ROS_WARN("lift_driver: E-STOP engaged, cannot start homing.");
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
    stall_tracking_ = false;
    pending_home_ = false;
    ROS_INFO("lift_driver: homing started (descend @ %d rpm, expect lower limit auto-stop).", home_dir_speed_);
}

void LiftDriver::updateMotion(int32_t pos, bool fresh)
{
    if (estop_engaged_.load()) return;   // estopCallback 이 이미 정지/MANUAL 처리
    const ros::Time now = ros::Time::now();

    if (mode_ == Mode::HOMING) {
        if ((now - home_start_).toSec() > home_timeout_sec_) {
            sendVelocity(0);
            mode_ = Mode::MANUAL;
            ROS_ERROR("lift_driver: homing timeout (%.0fs) - aborted, NOT homed.", home_timeout_sec_);
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
            sendPositionReset();   // 현재(하부 리미트) 위치를 원점 0 으로
            homed_ = true;
            mode_ = Mode::MANUAL;
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
    // PID_CTRL_STATUS 비트
    std::string s;
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

void LiftDriver::pollTimer(const ros::TimerEvent&)
{
    requestMonitor();
    readAndParse();

    int32_t pos; uint8_t st; bool fresh;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pos = position_;
        st = status_byte_;
        fresh = got_data_ && (ros::Time::now() - last_rx_).toSec() < timeout_sec_;
    }

    // 자동 홈잉(옵션): 통신이 살아난 뒤 1회 시작
    if (pending_home_ && fresh && !estop_engaged_.load()) {
        std_msgs::BoolPtr hm(new std_msgs::Bool());
        hm->data = true;
        homeCallback(hm);   // pending_home_ 은 여기서 해제됨
    }

    updateMotion(pos, fresh);   // 홈잉/위치이동 상태머신

    std_msgs::Int32 pmsg; pmsg.data = pos; position_pub_.publish(pmsg);

    std_msgs::String smsg;
    std::ostringstream ss;
    ss << (fresh ? "connected" : "NO_DATA") << " mode=" << modeStr() << " homed=" << (homed_ ? 1 : 0)
       << " pos=" << pos;
    if (mode_ == Mode::POSITION) ss << " target=" << target_pos_;
    ss << " status=" << decodeStatus(st);
    smsg.data = ss.str();
    status_pub_.publish(smsg);
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
