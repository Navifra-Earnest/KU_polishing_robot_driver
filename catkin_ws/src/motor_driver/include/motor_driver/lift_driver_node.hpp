#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

#include "motor_driver/serial_interface.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// ============================================================================
// 리프트 드라이버 (MDROBOT MD DC 모터드라이버, RS485/RS232, SAE J1587 기반)
//   프레임: RMID(183) TMID(host) ID PID DataNum DATA(low-byte first) CHK,  CHK=(~sum)+1
//   시리얼: 8N1, 19200bps. FTDI USB-RS485 는 DTR+RTS assert 필요(serial_interface 처리).
//
//   제어 방식(현장 확인): INPUT_TYPE=ANALOG(속도) + 리미트스위치 사용.
//     → 상승/하강 = 속도명령(PID_VEL_CMD, 130), 드라이브가 상/하 리미트에서 자동 정지.
//     → 위치는 홀/엔코더 카운트로 모니터링(증분).
//
//   ROS 인터페이스
//     입력  /lift/command       std_msgs/String  "up"/"down"/"stop"
//     입력  /lift/velocity_cmd  std_msgs/Int16   직접 속도지령[rpm] (부호=방향)  → PID_VEL_CMD(130)
//     출력  /lift/position      std_msgs/Int32   현재 위치(홀 카운트, 증분)
//     출력  /lift/status        std_msgs/String  연결/위치/상태비트
// ============================================================================
class LiftDriver {
public:
    LiftDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~LiftDriver();

private:
    void commandCallback(const std_msgs::String::ConstPtr& msg);
    void velocityCmdCallback(const std_msgs::Int16::ConstPtr& msg);
    void estopCallback(const std_msgs::Bool::ConstPtr& msg);  // /estop 긴급정지 (모터와 공유)
    void pollTimer(const ros::TimerEvent&);

    void sendFrame(uint8_t pid, const std::vector<uint8_t>& data);
    void sendVelocity(int16_t rpm);        // PID_VEL_CMD(130)
    void requestMonitor();                 // PID_REQ_PID_DATA(4) → PID_MONITOR(196)
    void readAndParse();
    void handleFrame(uint8_t pid, const uint8_t* data, uint8_t n);
    std::string decodeStatus(uint8_t st) const;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber vel_cmd_sub_;
    ros::Subscriber estop_sub_;
    ros::Publisher position_pub_;
    ros::Publisher status_pub_;
    ros::Timer poll_timer_;
    std::unique_ptr<SerialInterface> serial_;

    // 파라미터
    std::string port_{"/dev/ttyUSB0"};
    int baud_{19200};
    uint8_t motor_rmid_{183};
    uint8_t host_mid_{172};
    uint8_t ctrl_id_{1};
    int up_speed_rpm_{0};      // 상승 속도[rpm], 부호가 상승 방향 (하강은 -up_speed_rpm)
    double poll_rate_{10.0};
    double timeout_sec_{1.0};

    // 상태
    std::mutex data_mutex_;
    bool got_data_{false};
    int32_t position_{0};
    uint8_t status_byte_{0};
    int16_t rpm_{0};
    ros::Time last_rx_;
    std::atomic<bool> estop_engaged_{false};  // /estop=true 면 상승/하강 무시하고 정지

    std::vector<uint8_t> rx_buf_;
};
