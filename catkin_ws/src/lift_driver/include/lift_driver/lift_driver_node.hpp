#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

#include "lift_driver/serial_interface.hpp"

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
//   위치제어(추가): 드라이브가 RS485 위치명령을 지원.
//     → 절대이동 PID_POSI_CMD(243), 상대이동 PID_INC_POSI_CMD(244),
//       위치제어 최대속도 PID_TAR_POSI_VEL(176), 원점리셋 PID_POSI_RESET(13).
//     ⚠️ 위치는 전원 켠 시점 기준 홀카운트(증분)라 재부팅 시 원점 소실.
//        → 절대 재현성을 원하면 전원 사이클마다 /lift/home 로 1회 홈잉(하부 리미트→원점 0).
//        (진짜 절대엔코더는 별도 HW: PID_POS_SEN_TYPE=MENA_RS485/CAN, 현재 미장착)
//
//   ROS 인터페이스
//     입력  /lift/command        std_msgs/String  "up"/"down"/"stop"
//     입력  /lift/velocity_cmd   std_msgs/Int16   직접 속도지령[rpm] (부호=방향)  → PID_VEL_CMD(130)
//     입력  /lift/position_cmd   std_msgs/Int32   절대 목표위치[홀카운트]          → PID_POSI_CMD(243)
//     입력  /lift/inc_position_cmd std_msgs/Int32 상대 이동량[홀카운트]           → PID_INC_POSI_CMD(244)
//     입력  /lift/home           std_msgs/Bool    true=홈잉시작, false=중단
//     출력  /lift/position       std_msgs/Int32   현재 위치(홀 카운트, 증분)
//     출력  /lift/status         std_msgs/String  연결/모드/위치/상태비트
//     출력  /lift/homed          std_msgs/Bool    원점 확립 여부
//   임의 명령(velocity/up/down/stop) 또는 /estop 은 진행 중인 위치이동/홈잉을 취소하고 수동모드로 전환.
// ============================================================================
class LiftDriver {
public:
    LiftDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~LiftDriver();

    enum class Mode { MANUAL, POSITION, HOMING };

private:
    void commandCallback(const std_msgs::String::ConstPtr& msg);
    void velocityCmdCallback(const std_msgs::Int16::ConstPtr& msg);
    void positionCmdCallback(const std_msgs::Int32::ConstPtr& msg);      // 절대 위치이동
    void incPositionCmdCallback(const std_msgs::Int32::ConstPtr& msg);   // 상대 위치이동
    void homeCallback(const std_msgs::Bool::ConstPtr& msg);             // 홈잉 시작/중단
    void estopCallback(const std_msgs::Bool::ConstPtr& msg);  // /estop 긴급정지 (모터와 공유)
    void pollTimer(const ros::TimerEvent&);
    void updateMotion(int32_t pos, bool fresh);  // 홈잉/위치이동 상태머신 (pollTimer 에서 호출)

    void sendFrame(uint8_t pid, const std::vector<uint8_t>& data);
    void sendVelocity(int16_t rpm);        // PID_VEL_CMD(130)
    void sendPosition(int32_t counts);     // PID_POSI_CMD(243) 절대
    void sendIncPosition(int32_t counts);  // PID_INC_POSI_CMD(244) 상대
    void sendPositionVel(uint16_t rpm);    // PID_TAR_POSI_VEL(176) 위치제어 최대속도
    void sendPositionReset();              // PID_POSI_RESET(13) 현재위치→0
    void requestMonitor();                 // PID_REQ_PID_DATA(4) → PID_MONITOR(196)
    void readAndParse();
    void handleFrame(uint8_t pid, const uint8_t* data, uint8_t n);
    std::string decodeStatus(uint8_t st) const;
    const char* modeStr() const;
    int16_t manualUpSpeed() const;         // up_speed_rpm × (homed면 post_home_speed_scale)

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber vel_cmd_sub_;
    ros::Subscriber position_cmd_sub_;
    ros::Subscriber inc_position_cmd_sub_;
    ros::Subscriber home_sub_;
    ros::Subscriber estop_sub_;
    ros::Publisher position_pub_;
    ros::Publisher status_pub_;
    ros::Publisher homed_pub_;
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

    // 위치제어 파라미터
    bool enable_position_control_{true};
    int posi_ctrl_vel_rpm_{0};   // 위치제어 최대속도[rpm]. 0이면 |up_speed_rpm| 사용, 그것도 0이면 드라이브 기본(2000)
    int home_speed_rpm_{0};      // 홈잉(하강) 속도크기[rpm]. 0이면 |up_speed_rpm| 사용
    int home_stall_counts_{5};   // 이 카운트 이하로 위치변화가 없으면 '정지'로 간주
    double home_stall_sec_{1.5}; // 위 정지상태가 이 시간 이상 지속되면 홈 도달로 판정
    double home_min_sec_{1.0};   // 홈잉 시작 후 최소 이 시간이 지나야 완료 허용(스핀업 오검출 방지)
    double home_timeout_sec_{40.0};
    int in_position_tol_{20};    // |target-pos| 이 값 이하면 위치이동 완료로 간주
    bool auto_home_on_start_{false};
    double post_home_speed_scale_{2.0};  // 홈잉 완료(homed) 후 수동 up/down + 위치제어 속도 배율
    int posi_base_vel_rpm_{0};   // 위치제어 기준속도[rpm](홈잉 전). homed 후 ×post_home_speed_scale 적용

    // 상태
    std::mutex data_mutex_;
    bool got_data_{false};
    int32_t position_{0};
    uint8_t status_byte_{0};
    int16_t rpm_{0};
    ros::Time last_rx_;
    std::atomic<bool> estop_engaged_{false};  // /estop=true 면 상승/하강 무시하고 정지

    // 위치제어/홈잉 상태 (단일 스레드 spin, 콜백·타이머 동일 스레드라 락 불필요)
    Mode mode_{Mode::MANUAL};
    bool homed_{false};
    int32_t target_pos_{0};
    int16_t home_dir_speed_{0};   // 홈잉 하강 속도(부호 포함)
    ros::Time home_start_;
    ros::Time stall_since_;
    int32_t stall_ref_pos_{0};
    bool stall_tracking_{false};
    bool pending_home_{false};

    std::vector<uint8_t> rx_buf_;
};
