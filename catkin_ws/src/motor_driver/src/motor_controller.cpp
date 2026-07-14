// KINCO DRIVER - 구동모터(속도 제어) 전용
#include "motor_driver/motor_controller.hpp"
#include <iostream>
#include <cstring>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <future>
#include <tuple>

using namespace std::chrono_literals;

MotorController::MotorController() {}

MotorController::~MotorController() {
    is_running_ = false;
    if (sync_thread_.joinable()) {
        sync_thread_.join();
    }
    if (can_if_) {
        disable_motor();
    }
}

void MotorController::init(const std::string& can_device, const std::vector<uint8_t>& node_ids) {
    node_ids_ = node_ids;
    for (const auto& id : node_ids_) {
        motor_states_.emplace(std::piecewise_construct,
            std::forward_as_tuple(id),
            std::forward_as_tuple());
    }

    can_if_ = std::make_unique<CanInterface>();
    can_if_->register_callback(
        [this](const can_frame& frame) { this->can_callback(frame); });

    if (!can_if_->open(can_device)) {
        std::cerr << "Fatal: Failed to open CAN device " << can_device << std::endl;
        return;
    }

    for (const auto& node_id : node_ids_) {
        std::cout << "Initializing motor with Node ID: " << static_cast<int>(node_id) << std::endl;

        // 1. NMT Reset Communication
        can_if_->write(NMT_ID, {NMT_RESET_COMMUNICATION, node_id});
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // --- PDO Mapping (모든 구동모터는 속도 제어) ---
        // RPDO1: Controlword, Mode of operation
        setup_pdo_mapping(node_id, 0x1400, 0x1600, RPDO1_ID_BASE, {0x60400010, 0x60600008});
        // RPDO2: Target Velocity (0x60FF)
        setup_pdo_mapping(node_id, 0x1401, 0x1601, RPDO2_ID_BASE, {0x60FF0020});
        // TPDO1: Statusword, Mode of operation display
        setup_pdo_mapping(node_id, 0x1800, 0x1A00, TPDO1_ID_BASE, {0x60410010, 0x60610008});
        // TPDO2: Velocity actual, Position actual
        setup_pdo_mapping(node_id, 0x1801, 0x1A01, TPDO2_ID_BASE, {0x606C0020, 0x60640020});
        // TPDO3: DC link voltage, Current actual, Error code
        setup_pdo_mapping(node_id, 0x1802, 0x1A02, TPDO3_ID_BASE, {0x60790010, 0x60780010, 0x26010010, 0x26020010});

        // Start Node
        can_if_->write(NMT_ID, {NMT_START_REMOTE_NODE, node_id});
        std::this_thread::sleep_for(100ms);

        std::cout << "Motor with Node ID " << static_cast<int>(node_id) << " initialized." << std::endl;
    }

    is_running_ = true;

    // 피드백 타임아웃 기준 시각 초기화 (SYNC 시작 직전) → 첫 피드백 전 오탐 방지
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        auto now = std::chrono::steady_clock::now();
        for (const auto& id : node_ids_) {
            motor_states_.at(id).last_feedback_time = now;
        }
    }

    sync_thread_ = std::thread(&MotorController::sync_loop, this);
}

double MotorController::get_feedback_age_sec(uint8_t node_id) const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (motor_states_.count(node_id)) {
        std::chrono::duration<double> age =
            std::chrono::steady_clock::now() - motor_states_.at(node_id).last_feedback_time;
        return age.count();
    }
    return 1e9;  // 해당 모터 상태 없음 → 타임아웃으로 간주
}

void MotorController::setup_pdo_mapping(uint8_t node_id, uint16_t comm_param_index, uint16_t map_param_index, uint32_t cob_id_base, const std::vector<uint32_t>& mapped_objects)
{
    const auto SDO_RX_ID = SDO_RX_ID_BASE + node_id;
    const std::chrono::milliseconds delay(20);

    uint16_t comm_idx_low = comm_param_index & 0xFF;
    uint16_t comm_idx_high = (comm_param_index >> 8) & 0xFF;
    uint16_t map_idx_low = map_param_index & 0xFF;
    uint16_t map_idx_high = (map_param_index >> 8) & 0xFF;

    // 1. Disable PDO
    uint32_t cob_id_disabled = 0x80000000 | (cob_id_base + node_id);
    can_if_->write(SDO_RX_ID, 0x23, comm_idx_low, comm_idx_high, 1, cob_id_disabled & 0xFF, (cob_id_disabled >> 8) & 0xFF, (cob_id_disabled >> 16) & 0xFF, (cob_id_disabled >> 24) & 0xFF);
    std::this_thread::sleep_for(delay);
    // 2. Set transmission type to 1 (SYNC)
    can_if_->write(SDO_RX_ID, 0x2F, comm_idx_low, comm_idx_high, 2, 1, 0, 0, 0);
    std::this_thread::sleep_for(delay);
    // 3. Clear existing mapping
    can_if_->write(SDO_RX_ID, 0x2F, map_idx_low, map_idx_high, 0, 0, 0, 0, 0);
    std::this_thread::sleep_for(delay);
    // 4. Map new objects
    uint8_t sub_index = 1;
    for (const uint32_t& obj : mapped_objects) {
        can_if_->write(SDO_RX_ID, 0x23, map_idx_low, map_idx_high, sub_index++, obj & 0xFF, (obj >> 8) & 0xFF, (obj >> 16) & 0xFF, (obj >> 24) & 0xFF);
        std::this_thread::sleep_for(delay);
    }
    // 5. Set number of mapped objects
    can_if_->write(SDO_RX_ID, 0x2F, map_idx_low, map_idx_high, 0, static_cast<uint8_t>(mapped_objects.size()), 0, 0, 0);
    std::this_thread::sleep_for(delay);
    // 6. Enable PDO
    uint32_t cob_id_enabled = cob_id_base + node_id;
    can_if_->write(SDO_RX_ID, 0x23, comm_idx_low, comm_idx_high, 1, cob_id_enabled & 0xFF, (cob_id_enabled >> 8) & 0xFF, (cob_id_enabled >> 16) & 0xFF, (cob_id_enabled >> 24) & 0xFF);
    std::this_thread::sleep_for(delay);
}

void MotorController::sync_loop() {
    while (is_running_) {
        if (can_if_) {
            for (const auto& node_id : node_ids_) {
                float current_target_rpm = motor_states_.at(node_id).atomic_target_velocity_rpm.load();
                send_velocity_command(node_id, current_target_rpm);
            }
            can_if_->write(SYNC_ID, nullptr, 0);
        }
        std::this_thread::sleep_for(20ms);
    }
}

// ================= Velocity Control =================
void MotorController::set_target_velocity(uint8_t node_id, float rpm) {
    if (motor_states_.count(node_id)) {
        // 모터2는 좌/우 대칭 장착으로 방향 반전
        float target_rpm = (node_id == 2) ? rpm * (-1.0f) : rpm;
        motor_states_.at(node_id).atomic_target_velocity_rpm.store(target_rpm);
    }
}

void MotorController::send_velocity_command(uint8_t node_id, float rpm) {
    int32_t target_pps = static_cast<int32_t>(rpm * 2730.66f);  // 512 * 10000 / 1875
    RPDO2_Vel_Data vel_pdo_data;
    vel_pdo_data.target_velocity = target_pps;
    if (can_if_) {
        can_if_->write(RPDO2_ID_BASE + node_id, reinterpret_cast<uint8_t*>(&vel_pdo_data), sizeof(vel_pdo_data));
    }
}

float MotorController::get_feedback_velocity(uint8_t node_id) const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (motor_states_.count(node_id)) {
        // 내부 pps -> RPM. 모터2는 반전.
        if (node_id == 2) {
            return motor_states_.at(node_id).feedback_velocity_rpm / -2730.66f;
        }
        return motor_states_.at(node_id).feedback_velocity_rpm / 2730.66f;
    }
    return 0.0f;
}
// ====================================================

bool MotorController::reset_motor() {
    bool all_success = true;

    for (const auto& node_id : node_ids_) {
        std::cout << "Resetting motor with Node ID " << static_cast<int>(node_id) << "..." << std::endl;
        change_motor_state(node_id, Controlword::FAULT_RESET, 0);
        std::this_thread::sleep_for(100ms);
        change_motor_state(node_id, Controlword::FAULT_RESET, 0);
    }
    enable_motor();

    for (const auto& node_id : node_ids_) {
        if (!wait_for_state(node_id, CiA402State::OPERATION_ENABLED, 2000)) {
            std::cerr << "Motor " << static_cast<int>(node_id)
                      << " failed to reach OPERATION_ENABLED." << std::endl;
            all_success = false;
        }
    }
    return all_success;
}

void MotorController::change_motor_state(uint8_t node_id, uint16_t controlword, uint8_t mode) {
    RPDO1_Data pdo_data;
    pdo_data.controlword = controlword;
    pdo_data.modes_of_operation = mode;

    if (can_if_) {
        can_if_->write(RPDO1_ID_BASE + node_id, reinterpret_cast<uint8_t*>(&pdo_data), sizeof(pdo_data));
    }
}

CiA402State MotorController::get_cia402_state(uint16_t statusword) {
    bool ready_to_switch_on = (statusword & 0x01) != 0;
    bool switched_on = (statusword & 0x02) != 0;
    bool operation_enabled = (statusword & 0x04) != 0;
    bool fault = (statusword & 0x08) != 0;
    bool quick_stop = (statusword & 0x20) != 0;
    bool switch_on_disabled = (statusword & 0x40) != 0;

    if (fault) {
        return CiA402State::FAULT;
    }
    if (switch_on_disabled) {
        return CiA402State::SWITCH_ON_DISABLED;
    }
    if (!ready_to_switch_on && !switched_on && !operation_enabled) {
        return CiA402State::NOT_READY_TO_SWITCH_ON;
    }
    if (ready_to_switch_on && !switched_on && !operation_enabled) {
        return CiA402State::READY_TO_SWITCH_ON;
    }
    if (ready_to_switch_on && switched_on && !operation_enabled) {
        return CiA402State::SWITCHED_ON;
    }
    if (ready_to_switch_on && switched_on && operation_enabled) {
        return CiA402State::OPERATION_ENABLED;
    }
    if (!quick_stop && operation_enabled) {
        return CiA402State::QUICK_STOP_ACTIVE;
    }

    return CiA402State::NOT_READY_TO_SWITCH_ON;
}

bool MotorController::wait_for_state(uint8_t node_id, CiA402State target_state, uint32_t timeout_ms) {
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() - start_time < timeout) {
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            if (motor_states_.count(node_id)) {
                uint16_t statusword = motor_states_.at(node_id).status.status_flags;
                CiA402State current_state = get_cia402_state(statusword);

                if (current_state == target_state) {
                    return true;
                }
                if (current_state == CiA402State::FAULT) {
                    std::cerr << "Motor Node ID " << static_cast<int>(node_id)
                              << " entered FAULT state (statusword: 0x"
                              << std::hex << statusword << std::dec << ")" << std::endl;
                    return false;
                }
            }
        }
        std::this_thread::sleep_for(10ms);
    }

    std::lock_guard<std::mutex> lock(status_mutex_);
    if (motor_states_.count(node_id)) {
        uint16_t statusword = motor_states_.at(node_id).status.status_flags;
        CiA402State current_state = get_cia402_state(statusword);
        std::cerr << "Timeout waiting for state transition. Node ID: " << static_cast<int>(node_id)
                  << ", Current state: " << static_cast<int>(current_state)
                  << ", Target state: " << static_cast<int>(target_state)
                  << ", Statusword: 0x" << std::hex << statusword << std::dec << std::endl;
    }

    return false;
}

bool MotorController::enable_motor() {
    std::vector<std::future<bool>> futures;

    for (const auto& node_id : node_ids_) {
        futures.push_back(std::async(std::launch::async, [this, node_id]() -> bool {
            const uint8_t mode = 3;  // Velocity Mode

            std::cout << "Enabling Motor with Node ID " << static_cast<int>(node_id) << "..." << std::endl;

            const auto SDO_RX_ID = SDO_RX_ID_BASE + node_id;

            // 1. Set Operation Mode (Velocity, 3)
            can_if_->write(SDO_RX_ID, 0x2F, 0x60, 0x60, 0, mode, 0, 0, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            // 2. Fault reset if needed
            CiA402State initial_state;
            {
                std::lock_guard<std::mutex> lock(status_mutex_);
                initial_state = get_cia402_state(motor_states_.at(node_id).status.status_flags);
            }
            if (initial_state == CiA402State::FAULT) {
                change_motor_state(node_id, Controlword::FAULT_RESET, 0);
                std::this_thread::sleep_for(100ms);
            }

            // Step 1: SHUTDOWN
            std::this_thread::sleep_for(100ms);
            change_motor_state(node_id, Controlword::SHUTDOWN, mode);
            std::this_thread::sleep_for(100ms);
            change_motor_state(node_id, Controlword::SHUTDOWN, mode);
            if (!wait_for_state(node_id, CiA402State::READY_TO_SWITCH_ON, 2000)) {
                std::cerr << "Failed to reach READY_TO_SWITCH_ON state for Node ID " << static_cast<int>(node_id) << std::endl;
                return false;
            }

            // Step 2: SWITCH_ON
            std::this_thread::sleep_for(100ms);
            change_motor_state(node_id, Controlword::SWITCH_ON, mode);
            std::this_thread::sleep_for(100ms);
            change_motor_state(node_id, Controlword::SWITCH_ON, mode);
            if (!wait_for_state(node_id, CiA402State::SWITCHED_ON, 2000)) {
                std::cerr << "Failed to reach SWITCHED_ON state for Node ID " << static_cast<int>(node_id) << std::endl;
                return false;
            }

            // Step 3: ENABLE_OPERATION
            std::this_thread::sleep_for(100ms);
            change_motor_state(node_id, Controlword::ENABLE_OPERATION, mode);
            std::this_thread::sleep_for(100ms);
            change_motor_state(node_id, Controlword::ENABLE_OPERATION, mode);
            if (!wait_for_state(node_id, CiA402State::OPERATION_ENABLED, 2000)) {
                std::cerr << "Failed to reach OPERATION_ENABLED state for Node ID " << static_cast<int>(node_id) << std::endl;
                return false;
            }

            std::cout << "Motor with Node ID " << static_cast<int>(node_id) << " enabled (Servo ON)." << std::endl;
            return true;
        }));
    }

    // 모든 모터가 OPERATION_ENABLED 에 도달해야 성공. 하나라도 실패하면 false.
    bool all_enabled = true;
    for (auto& f : futures) {
        if (!f.get()) all_enabled = false;
    }
    return all_enabled;
}

void MotorController::disable_motor() {
    std::cout << "Disabling all motors..." << std::endl;
    for (const auto& node_id : node_ids_) {
        send_velocity_command(node_id, 0);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    for (const auto& node_id : node_ids_) {
        const uint8_t mode = 3;
        change_motor_state(node_id, Controlword::SHUTDOWN, mode);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        std::cout << "Motor with Node ID " << static_cast<int>(node_id) << " disabled (Servo OFF)." << std::endl;
    }
}

std::string MotorController::get_kinco_error_message(uint16_t error_code) {
    // Based on Kinco Low-Voltage User Manual 2020 (p.94 ~ p.99)
    switch (error_code) {
        case 0x7380: return "Encoder ABZ signal incorrect";
        case 0x7381: return "Encoder UVW signal incorrect";
        case 0x4210: return "Controller Temperature";
        case 0x3210: return "Overvoltage";
        case 0x3220: return "Undervoltage";
        case 0x2320: return "Short circuit of driver output";
        case 0x7110: return "Driver brake resistor is abnormal";
        case 0x8611: return "Following error";
        case 0x5112: return "Low logic voltage";
        case 0x2350: return "Motor or controller IIt (Overload)";
        case 0x8A80: return "Over input frequency";
        case 0x4310: return "Motor temperature";
        case 0x7122: return "Motor excitation";
        case 0x6310: return "EEPROM data";
        case 0x5210: return "Current sensor error";
        case 0x6010: return "Watchdog error";
        case 0x6011: return "Wrong interrupt";
        case 0x7400: return "MCU ID error";
        case 0x6320: return "Motor configuration error";
        case 0x5443: return "External enable error";
        case 0x5442: return "Positive limit error";
        case 0x5441: return "Negative limit error";
        case 0x6012: return "SPI internal error";
        case 0x8A81: return "Close loop direction error";
        case 0x7306: return "Encoder count wrong / Master counting error";
        case 0x0000: return "No error";
        default:     return "Unknown error code";
    }
}

std::optional<std::string> MotorController::get_alarm(uint8_t node_id) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (motor_states_.count(node_id)) {
        auto& state = motor_states_.at(node_id);

        if (state.error_code != 0) {
            state.last_alarm_code = state.error_code;
            std::stringstream ss;
            ss << "[" << std::to_string(node_id) << "] " << get_kinco_error_message(state.error_code)
               << " (0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << state.error_code << ")";
            return ss.str();
        }
        else if (state.error_code == 0 && state.last_alarm_code != 0) {
            state.last_alarm_code = 0;
            // 에러 해소됨
            return std::nullopt;
        }
    }
    return std::nullopt;
}

void MotorController::can_callback(const can_frame& frame) {
    uint8_t node_id = frame.can_id & 0x7F;
    uint16_t function_code = frame.can_id & 0xFF80;

    if (motor_states_.find(node_id) == motor_states_.end()) {
        return;
    }

    std::lock_guard<std::mutex> lock(status_mutex_);

    if (function_code == SDO_TX_ID_BASE) {
        if (frame.data[0] == 0x80) {
            std::cerr << "SDO Write Error received for Node ID " << static_cast<int>(node_id) << std::endl;
        }
    }
    else if (function_code == TPDO1_ID_BASE) {
        if (frame.can_dlc >= sizeof(TPDO1_Data)) {
            TPDO1_Data pdo_data;
            memcpy(&pdo_data, frame.data, sizeof(pdo_data));
            motor_states_[node_id].status.status_flags = pdo_data.statusword;
        }
    }
    else if (function_code == TPDO2_ID_BASE) {
        if (frame.can_dlc >= sizeof(TPDO2_Data)) {
            TPDO2_Data pdo_data;
            memcpy(&pdo_data, frame.data, sizeof(pdo_data));
            motor_states_[node_id].feedback_velocity_rpm = static_cast<float>(pdo_data.velocity_actual);
            motor_states_[node_id].feedback_position = pdo_data.position_actual;
            motor_states_[node_id].last_feedback_time = std::chrono::steady_clock::now();
        }
    }
    else if (function_code == TPDO3_ID_BASE) {
        if (frame.can_dlc >= sizeof(TPDO3_Data)) {
            TPDO3_Data pdo_data;
            memcpy(&pdo_data, frame.data, sizeof(pdo_data));
            motor_states_[node_id].voltage = pdo_data.dc_link_voltage;
            motor_states_[node_id].status.current_a = static_cast<float>(pdo_data.current_actual);
            motor_states_[node_id].status.voltage = pdo_data.dc_link_voltage;
            motor_states_[node_id].error_code = pdo_data.error_code;
        }
    }
}

MotorSnapshot MotorController::get_status(uint8_t node_id) const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (motor_states_.count(node_id)) {
        auto& state = motor_states_.at(node_id);
        MotorSnapshot snap;
        snap.status_flags = state.status.status_flags;
        snap.current_a = state.status.current_a;
        snap.voltage = state.voltage / 10;  // 0.1V -> V
        snap.error_code = state.error_code;
        return snap;
    }
    return MotorSnapshot();
}
