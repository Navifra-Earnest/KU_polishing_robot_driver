#include "motor_driver/bms_driver_node.hpp"

#include <algorithm>

namespace {
// big-endian 헬퍼 (Daly 는 MSB first)
inline uint16_t be16(const uint8_t* b) { return (uint16_t(b[0]) << 8) | b[1]; }
inline uint32_t be32(const uint8_t* b) {
    return (uint32_t(b[0]) << 24) | (uint32_t(b[1]) << 16) | (uint32_t(b[2]) << 8) | b[3];
}
}  // namespace

BmsDriver::BmsDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh)
{
    pnh_.param<std::string>("can_device", can_device_, "can1");
    pnh_.param<std::string>("frame_id", frame_id_, "bms");
    pnh_.param<double>("poll_rate", poll_rate_, 2.0);
    pnh_.param<double>("publish_rate", publish_rate_, 2.0);
    pnh_.param<double>("timeout_sec", timeout_sec_, 3.0);
    pnh_.param<double>("design_capacity", design_capacity_, 0.0);
    int bms_addr = 0x01, host_addr = 0x40;
    pnh_.param<int>("bms_addr", bms_addr, 0x01);
    pnh_.param<int>("host_addr", host_addr, 0x40);
    bms_addr_ = static_cast<uint8_t>(bms_addr);
    host_addr_ = static_cast<uint8_t>(host_addr);

    battery_pub_ = nh_.advertise<sensor_msgs::BatteryState>("/bms/state", 10);
    soc_pub_ = nh_.advertise<std_msgs::Float32>("/bms/soc", 10);  // 0~100 % (편의용)

    can_ = std::make_unique<CanInterface>();
    can_->register_callback([this](const can_frame& f) { this->canCallback(f); });
    if (!can_->open(can_device_)) {
        ROS_ERROR("bms_driver: failed to open CAN device %s (is it up at 250K?)", can_device_.c_str());
    }

    last_rx_ = ros::Time(0);
    poll_timer_ = nh_.createTimer(ros::Duration(1.0 / poll_rate_), &BmsDriver::pollTimer, this);
    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &BmsDriver::publishTimer, this);

    ROS_INFO("bms_driver started: dev=%s, poll=%.1fHz, publishing /bms/state (Daly CAN 250K, extended ID)",
             can_device_.c_str(), poll_rate_);
}

BmsDriver::~BmsDriver()
{
    if (can_) can_->close();
}

// 요청 프레임: 0x18 <DataID> <BMS:0x01> <PC:0x40>, 데이터 8B reserved(0)
void BmsDriver::sendRequest(uint8_t data_id)
{
    uint32_t id = (uint32_t(0x18) << 24) | (uint32_t(data_id) << 16) |
                  (uint32_t(bms_addr_) << 8) | uint32_t(host_addr_);
    if (can_) can_->write(id | CAN_EFF_FLAG, 0, 0, 0, 0, 0, 0, 0, 0);
}

void BmsDriver::pollTimer(const ros::TimerEvent&)
{
    // 배터리 모니터링에 필요한 항목 폴링
    sendRequest(0x90);  // 전압/전류/SOC
    sendRequest(0x92);  // 온도
    sendRequest(0x93);  // 충방전 상태/잔여용량
    sendRequest(0x94);  // 충전기/부하 연결
    sendRequest(0x98);  // 고장/알람
}

void BmsDriver::canCallback(const can_frame& frame)
{
    const uint32_t id = frame.can_id & CAN_EFF_MASK;
    // BMS 응답만: 하위 16비트가 <PC:0x40><BMS:0x01> 이어야 함
    if ((id & 0xFFFF) != ((uint32_t(host_addr_) << 8) | bms_addr_)) return;
    const uint8_t data_id = (id >> 16) & 0xFF;
    if (frame.can_dlc < 8) return;
    const uint8_t* b = frame.data;

    std::lock_guard<std::mutex> lock(data_mutex_);
    switch (data_id) {
        case 0x90:
            voltage_ = be16(b) * 0.1f;                          // 누적 총전압 0.1V
            current_ = (int(be16(b + 4)) - 30000) * 0.1f;       // 30000 offset, 0.1A
            soc_ = be16(b + 6) * 0.1f;                          // 0.1%
            got_data_ = true;
            last_rx_ = ros::Time::now();
            break;
        case 0x92:
            max_temp_ = float(int(b[0]) - 40);                  // 40 offset ℃
            min_temp_ = float(int(b[2]) - 40);
            break;
        case 0x93:
            cd_state_ = b[0];                                   // 0정지/1충전/2방전
            remain_capacity_ah_ = be32(b + 4) / 1000.0f;        // mAh -> Ah
            break;
        case 0x94:
            num_cells_ = b[0];
            num_temps_ = b[1];
            charger_status_ = b[2];                             // 0/1
            load_status_ = b[3];
            break;
        case 0x98: {
            bool f = false;
            for (int i = 0; i < 7; ++i) f = f || (b[i] != 0);   // byte0~6 알람비트
            has_fault_ = f;
            break;
        }
        default:
            break;
    }
}

void BmsDriver::publishTimer(const ros::TimerEvent&)
{
    sensor_msgs::BatteryState msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;

    std::lock_guard<std::mutex> lock(data_mutex_);
    const bool fresh = got_data_ && (ros::Time::now() - last_rx_).toSec() < timeout_sec_;

    msg.voltage = voltage_;
    msg.current = current_;                       // +충전/-방전 (하드웨어에서 극성 검증)
    msg.temperature = max_temp_;
    msg.charge = remain_capacity_ah_;             // Ah
    msg.capacity = remain_capacity_ah_;           // 현재 용량(추정)
    msg.design_capacity = static_cast<float>(design_capacity_);
    msg.percentage = std::max(0.0f, std::min(soc_ / 100.0f, 1.0f));  // 0~1
    msg.present = fresh;

    if (!fresh) {
        msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    } else {
        if (charger_status_ == 1 || cd_state_ == 1)
            msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
        else if (cd_state_ == 2)
            msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        else if (soc_ >= 99.5f)
            msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL;
        else
            msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;

        msg.power_supply_health = has_fault_
            ? sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
            : sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    }
    msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

    battery_pub_.publish(msg);

    // 편의용 SOC (0~100 %) — BatteryState.percentage(0~1) 와 별개
    std_msgs::Float32 soc_msg;
    soc_msg.data = soc_;
    soc_pub_.publish(soc_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bms_driver");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    BmsDriver node(nh, pnh);
    ros::spin();
    return 0;
}
