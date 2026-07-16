#include "lift_driver/lift_driver_node.hpp"

#include <sstream>

namespace {
// MDROBOT PID
constexpr uint8_t PID_REQ_PID_DATA = 4;
constexpr uint8_t PID_VEL_CMD      = 130;   // 속도명령 (rpm, INT 2byte, 부호=방향)
constexpr uint8_t PID_MONITOR      = 196;   // rpm/current/status/position
constexpr uint8_t PID_POSI_DATA    = 197;

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

    if (up_speed_rpm_ == 0) {
        ROS_WARN("lift_driver: up_speed_rpm not set; 'up'/'down' will not move. "
                 "Set it in config/lift_driver.yaml (sign = up direction). velocity_cmd still works.");
    }

    cmd_sub_ = nh_.subscribe("/lift/command", 10, &LiftDriver::commandCallback, this);
    vel_cmd_sub_ = nh_.subscribe("/lift/velocity_cmd", 10, &LiftDriver::velocityCmdCallback, this);
    estop_sub_ = nh_.subscribe("/estop", 10, &LiftDriver::estopCallback, this);  // 모터와 공유
    position_pub_ = nh_.advertise<std_msgs::Int32>("/lift/position", 10);
    status_pub_ = nh_.advertise<std_msgs::String>("/lift/status", 10);

    serial_ = std::make_unique<SerialInterface>();
    if (!serial_->open(port_, baud_)) {
        ROS_ERROR("lift_driver: failed to open serial %s @ %d. Check device/permission (dialout).",
                  port_.c_str(), baud_);
    }

    last_rx_ = ros::Time(0);
    poll_timer_ = nh_.createTimer(ros::Duration(1.0 / poll_rate_), &LiftDriver::pollTimer, this);
    ROS_INFO("lift_driver started: port=%s @ %d 8N1, ctrl_id=%d, up_speed=%d rpm (MDROBOT RS485, speed+limit)",
             port_.c_str(), baud_, ctrl_id_, up_speed_rpm_);
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

void LiftDriver::requestMonitor()
{
    sendFrame(PID_REQ_PID_DATA, {PID_MONITOR});
}

void LiftDriver::estopCallback(const std_msgs::Bool::ConstPtr& msg)
{
    const bool prev = estop_engaged_.exchange(msg->data);
    if (msg->data) {
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
        sendVelocity(static_cast<int16_t>(up_speed_rpm_));       // 리미트에서 드라이브가 자동 정지
    } else if (c == "down") {
        sendVelocity(static_cast<int16_t>(-up_speed_rpm_));
    } else if (c == "stop") {
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
    sendVelocity(msg->data);
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

    std_msgs::Int32 pmsg; pmsg.data = pos; position_pub_.publish(pmsg);

    std_msgs::String smsg;
    std::ostringstream ss;
    ss << (fresh ? "connected" : "NO_DATA") << " pos=" << pos << " status=" << decodeStatus(st);
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
