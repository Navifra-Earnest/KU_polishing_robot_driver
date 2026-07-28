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
//   알람(에러) 처리: PID_MONITOR(196) D7 = PID_CTRL_STATUS 비트. BIT0 = ALARM(알람 유/무).
//     → 알람 감지 시 즉시 정지(속도 0 + MANUAL)하고 PID_ALARM_RESET(12) 로 해제를 시도한다.
//       (자동 재시도 = auto_reset_on_alarm, 간격/횟수 제한. 수동은 /lift/reset)
//       알람이 해제되면 알람 직전에 진행 중이던 위치이동/홈잉을 자동 재개한다.
//
//   ROS 인터페이스
//     입력  /lift/command        std_msgs/String  "up"/"down"/"stop"
//     입력  /lift/velocity_cmd   std_msgs/Int16   직접 속도지령[rpm] (부호=방향)  → PID_VEL_CMD(130)
//     입력  /lift/position_cmd   std_msgs/Int32   절대 목표위치[홀카운트]          → PID_POSI_CMD(243)
//     입력  /lift/inc_position_cmd std_msgs/Int32 상대 이동량[홀카운트]           → PID_INC_POSI_CMD(244)
//     입력  /lift/home           std_msgs/Bool    true=홈잉시작, false=중단
//     입력  /lift/reset          std_msgs/Bool    true=알람 리셋(수동)            → PID_ALARM_RESET(12)
//     출력  /lift/position       std_msgs/Int32   현재 위치(홀 카운트, 증분)
//     출력  /lift/status         std_msgs/String  연결/모드/위치/상태비트
//     출력  /lift/homed          std_msgs/Bool    원점 확립 여부
//     출력  /lift/error          std_msgs/Bool    알람 발생 여부(true=알람)
//     출력  /lift/alarm          std_msgs/String  알람 내용(디코딩된 상태비트)
//   임의 명령(velocity/up/down/stop)은 진행 중인 위치이동/홈잉을 취소하고 수동모드로 전환.
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
    void resetCallback(const std_msgs::Bool::ConstPtr& msg);            // 알람 리셋(수동)
    void pollTimer(const ros::TimerEvent&);
    void updateMotion(int32_t pos, bool fresh, uint8_t di);  // 홈잉/위치이동 상태머신 (pollTimer 에서 호출)
    // 수동 속도명령이 리미트/구속으로 정지했으면 속도 0 을 보내 명령을 해제한다.
    // (막힌 축에 지령을 계속 유지하면 드라이브가 CTRL_FAIL 을 낸다 — 상부 리미트에서 실측)
    void releaseIfBlocked(int32_t pos, bool fresh, uint8_t di);
    bool atLowerLimit(uint8_t di, bool fresh) const;   // DIR 비트 0 = 하부 리미트 도달
    // 알람 감지/리셋/재개. updateMotion 보다 먼저 호출되어야 한다(알람 중 속도 재전송 방지).
    void updateAlarm(uint8_t st, bool fresh, int32_t pos);
    void requestAlarmReset(bool manual);   // PID_ALARM_RESET 전송 + 재시도 카운터 관리
    void resumeAfterAlarm(int32_t pos);    // 알람 직전 위치이동/홈잉 재개
    void cancelResume();                   // 새 명령이 오면 보류중인 재개를 폐기

    void sendFrame(uint8_t pid, const std::vector<uint8_t>& data);
    void sendVelocity(int16_t rpm);        // PID_VEL_CMD(130)
    void sendPosition(int32_t counts);     // PID_POSI_CMD(243) 절대
    void sendIncPosition(int32_t counts);  // PID_INC_POSI_CMD(244) 상대
    void sendPositionVel(uint16_t rpm);    // PID_TAR_POSI_VEL(176) 위치제어 최대속도
    void sendPositionReset();              // PID_POSI_RESET(13) 현재위치→0
    void sendAlarmReset();                 // PID_ALARM_RESET(12) 알람상태 해제
    void requestMonitor();                 // PID_REQ_PID_DATA(4) → PID_MONITOR(196)
    void readAndParse();
    void handleFrame(uint8_t pid, const uint8_t* data, uint8_t n);
    std::string decodeStatus(uint8_t st) const;
    std::string decodeDi(uint8_t di) const;   // PID_MONITOR D12 = PID_DI(48). 리미트스위치 상태 포함
    const char* modeStr() const;
    int16_t manualUpSpeed() const;         // up_speed_rpm × (homed면 post_home_speed_scale)

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber vel_cmd_sub_;
    ros::Subscriber position_cmd_sub_;
    ros::Subscriber inc_position_cmd_sub_;
    ros::Subscriber home_sub_;
    ros::Subscriber reset_sub_;
    ros::Publisher position_pub_;
    ros::Publisher status_pub_;
    ros::Publisher homed_pub_;
    ros::Publisher error_pub_;
    ros::Publisher alarm_pub_;
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

    // ── 리미트스위치 기반 판정 (실기 확정 2026-07-28) ────────────────────────
    // 하부 리미트만 드라이브 입력(CTRL #6 DIR)에 물려 있다 → DI bit0x04 == 0 이면 하부 리미트.
    // 상부 리미트는 드라이브 입력에 없다 → 기구 끝단에 밀려 정지하며, 명령을 계속 유지하면
    // 드라이브가 CTRL_FAIL(속도제어 실패)을 낸다. 그래서 상부는 '정지 감지'로만 알 수 있다.
    bool use_limit_switch_for_home_{true};  // 홈잉 완료를 DIR 비트로 판정(false=정지감지 추론만)
    int limit_di_descend_mask_{0x04};       // 하강 허용/하부 리미트 DI 비트 (CTRL #6 DIR)
    bool auto_release_on_stop_{true};       // 리미트/구속으로 정지 시 속도명령 자동 해제(CTRL_FAIL 예방)
    double release_grace_sec_{2.0};         // 명령 후 이 시간은 정지판정 안 함(스핀업 여유)
    double release_stop_sec_{1.0};          // 이 시간 이상 위치변화 없으면 정지로 보고 속도 0 전송

    // 알람 리셋 파라미터
    bool auto_reset_on_alarm_{true};       // 알람 감지 시 PID_ALARM_RESET 자동 전송
    double alarm_reset_interval_sec_{2.0}; // 자동 리셋 재시도 간격[s]
    int alarm_reset_max_attempts_{5};      // 연속 자동 리셋 시도 한계(0=무제한). 초과 시 포기(수동 대기)
    // 알람 해제 후 자동 재개 횟수 한계(0=무제한). 재개 → 같은 원인으로 재알람 → 재개 …
    // 사이클이 무한 반복되는 것을 막는다. 정상 완료/새 명령/수동 리셋 시 카운터는 0 으로 복귀.
    int alarm_resume_max_cycles_{3};

    // 상태
    std::mutex data_mutex_;
    bool got_data_{false};
    int32_t position_{0};
    uint8_t status_byte_{0};
    // PID_MONITOR D12 = DI 입력(PID_DI, 48). ⚠️ 이 리프트에서 CTRL 커넥터 #6(DIR)·#8(START/STOP)이
    // 리미트스위치(NC) 입력을 겸한다(매뉴얼 §2.3): 해당 방향 구동은 그 입력이 ON 이어야 가능.
    // 즉 이 비트가 리미트 도달의 "진짜" 신호다 (정지감지 추론보다 정확). 방향↔비트 대응은 실기 확정 필요.
    uint8_t di_byte_{0};
    int16_t rpm_{0};
    ros::Time last_rx_;

    // 위치제어/홈잉 상태 (단일 스레드 spin, 콜백·타이머 동일 스레드라 락 불필요)
    Mode mode_{Mode::MANUAL};
    bool homed_{false};
    int32_t target_pos_{0};
    int16_t home_dir_speed_{0};   // 홈잉 하강 속도(부호 포함)
    ros::Time home_start_;
    int32_t home_start_pos_{0};   // 홈잉 시작 시점 위치 (중단/완료 시 이동량 진단용)
    // 홈잉 중 드라이브 알람이 있었는지. 이 리프트는 리미트 도달 시에도 ALARM(CTRL_FAIL)을 올리므로
    // (현장 확인 2026-07-28) 홈잉 중 알람은 "하부 리미트 도달"로 보고 정지감지가 원점을 확정하게 둔다.
    bool home_alarm_seen_{false};
    ros::Time home_log_time_;     // 홈잉 진행상황 로그 마지막 시각
    int32_t home_log_pos_{0};     // 위 시각의 위치 (구간 이동량 계산용)
    ros::Time stall_since_;
    int32_t stall_ref_pos_{0};
    bool stall_tracking_{false};
    bool pending_home_{false};

    // 수동 속도명령 추적 (정지 시 자동 해제용)
    int16_t last_vel_cmd_{0};      // 마지막으로 보낸 속도지령[rpm] (0=해제된 상태)
    ros::Time vel_cmd_time_;       // 그 지령을 보낸 시각 (스핀업 여유 판정)
    int32_t vel_ref_pos_{0};       // 정지판정 기준 위치
    ros::Time vel_ref_time_;       // 위 위치가 갱신된 시각
    bool vel_ref_valid_{false};

    // 알람 상태 (단일 스레드 spin 이라 락 불필요)
    bool alarm_active_{false};          // 마지막 fresh 데이터 기준 알람 여부
    uint8_t alarm_status_{0};           // 알람 발생 시의 상태바이트(로그/발행용)
    int alarm_reset_attempts_{0};       // 현재 알람에 대한 자동 리셋 시도 횟수
    bool alarm_reset_exhausted_{false}; // 자동 재시도 한계 초과 → 수동 /lift/reset 대기
    ros::Time last_alarm_reset_;
    // 알람 직전 진행 중이던 동작(알람 해제 후 재개용). MANUAL = 재개할 것 없음.
    Mode resume_mode_{Mode::MANUAL};
    int32_t resume_target_{0};
    int resume_cycles_{0};              // 연속 자동 재개 횟수 (완료·새명령·수동리셋 시 0)

    std::vector<uint8_t> rx_buf_;
};
