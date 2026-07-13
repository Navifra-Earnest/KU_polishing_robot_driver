#pragma once

#include "motor_driver/can_interface.hpp"
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>
#include <map>
#include <thread>
#include <optional>
#include <atomic>
#include <string>
#include <chrono>

// ============================================================================
// Kinco CANopen (CiA 402) 구동모터 컨트롤러 - 속도 제어 전용 (모터 2개)
//   원본(navicomm_hyundai) 대비 리프트/턴테이블(차상모터) 위치제어·호밍 로직 제거.
// ============================================================================

// CAN PDO 데이터 구조체 정의
#pragma pack(1)
struct RPDO1_Data {
    uint16_t controlword;
    int8_t modes_of_operation;
};
struct RPDO2_Vel_Data {
    int32_t target_velocity;
};
struct TPDO1_Data {
    uint16_t statusword;
    int8_t mode_of_operation_display;
};
struct TPDO2_Data {
    int32_t velocity_actual;
    int32_t position_actual;
};
struct TPDO3_Data {
    uint16_t dc_link_voltage;
    int16_t current_actual;
    uint16_t error_code;
};
#pragma pack()

// CANopen NMT Commands
constexpr uint8_t NMT_START_REMOTE_NODE = 0x01;
constexpr uint8_t NMT_STOP_REMOTE_NODE = 0x02;
constexpr uint8_t NMT_ENTER_PRE_OPERATIONAL = 0x80;
constexpr uint8_t NMT_RESET_NODE = 0x81;
constexpr uint8_t NMT_RESET_COMMUNICATION = 0x82;

// CANopen Function Codes
constexpr uint16_t NMT_ID = 0x000;
constexpr uint16_t SYNC_ID = 0x080;
constexpr uint16_t SDO_TX_ID_BASE = 0x580;
constexpr uint16_t SDO_RX_ID_BASE = 0x600;
constexpr uint16_t TPDO1_ID_BASE = 0x180;
constexpr uint16_t RPDO1_ID_BASE = 0x200;
constexpr uint16_t TPDO2_ID_BASE = 0x280;
constexpr uint16_t RPDO2_ID_BASE = 0x300;
constexpr uint16_t TPDO3_ID_BASE = 0x380;

// CiA 402 상태 제어를 위한 Controlword 값
namespace Controlword {
    constexpr uint16_t SHUTDOWN = 0x0006;
    constexpr uint16_t SWITCH_ON = 0x0007;
    constexpr uint16_t ENABLE_OPERATION = 0x000F;
    constexpr uint16_t START_MOTION = 0x001F;
    constexpr uint16_t FAULT_RESET = 0x0080;
}

// CiA 402 - State
enum class CiA402State {
    NOT_READY_TO_SWITCH_ON = 0,
    SWITCH_ON_DISABLED = 1,
    READY_TO_SWITCH_ON = 2,
    SWITCHED_ON = 3,
    OPERATION_ENABLED = 4,
    QUICK_STOP_ACTIVE = 5,
    FAULT_REACTION_ACTIVE = 6,
    FAULT = 7
};

// 모터 상태/알람 스냅샷 (ROS 커스텀 메시지 의존성 제거 → 표준 타입으로 발행)
struct MotorSnapshot {
    float current_a{0.0f};      // current_actual 원시값 (스케일 미적용)
    uint16_t status_flags{0};   // CiA402 statusword
    uint16_t voltage{0};        // DC 링크 전압 [V] (get_status 에서 0.1V→V 변환)
    uint16_t error_code{0};     // Kinco 에러 코드 (0=정상)
};

class MotorController {
public:
    MotorController();
    ~MotorController();

    void init(const std::string& can_device, const std::vector<uint8_t>& node_ids);

    // 속도 제어
    void set_target_velocity(uint8_t node_id, float rpm);
    float get_feedback_velocity(uint8_t node_id) const;
    // 마지막 속도 피드백 수신 후 경과 시간 [s] (피드백 없으면 큰 값 반환)
    double get_feedback_age_sec(uint8_t node_id) const;

    MotorSnapshot get_status(uint8_t node_id) const;
    // 알람 발생 시 사람이 읽을 수 있는 메시지를 반환 (없으면 nullopt)
    std::optional<std::string> get_alarm(uint8_t node_id);

    void enable_motor();
    void disable_motor();
    bool reset_motor();

private:
    struct MotorState {
        uint16_t voltage{0};
        uint16_t error_code{0};
        MotorSnapshot status;
        uint16_t last_alarm_code{0};
        std::atomic<float> atomic_target_velocity_rpm{0.0f};
        float feedback_velocity_rpm{0.0f};
        int32_t feedback_position{0};
        // 마지막으로 속도 피드백(TPDO2)을 수신한 시각 (통신 타임아웃 감지용)
        std::chrono::steady_clock::time_point last_feedback_time{};
    };

    void can_callback(const can_frame& frame);
    void send_velocity_command(uint8_t node_id, float rpm);

    void change_motor_state(uint8_t node_id, uint16_t controlword, uint8_t mode);
    CiA402State get_cia402_state(uint16_t statusword);
    bool wait_for_state(uint8_t node_id, CiA402State target_state, uint32_t timeout_ms);

    void setup_pdo_mapping(uint8_t node_id, uint16_t comm_param_index, uint16_t map_param_index, uint32_t cob_id_base, const std::vector<uint32_t>& mapped_objects);
    void sync_loop();

    std::string get_kinco_error_message(uint16_t error_code);

    std::unique_ptr<CanInterface> can_if_;
    mutable std::mutex status_mutex_;
    std::vector<uint8_t> node_ids_;
    std::map<uint8_t, MotorState> motor_states_;
    std::atomic<bool> is_running_{false};
    std::thread sync_thread_;
};
