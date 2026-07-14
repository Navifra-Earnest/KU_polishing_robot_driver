#pragma once

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <linux/can.h>

#include "motor_driver/can_interface.hpp"

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

// ============================================================================
// Daly (达锂) BMS CAN 드라이버 (ROS1 Noetic)
//   프로토콜: CAN 250K, 29비트 확장 ID. 요청-응답(폴링) 방식.
//     요청(호스트→BMS): 0x18 <DataID> <BMS:0x01> <PC:0x40>   (data 8B reserved)
//     응답(BMS→호스트): 0x18 <DataID> <0x40> <0x01>          (data 8B, big-endian)
//   폴링 DataID:
//     0x90 총전압(0.1V)/전류(30000offset,0.1A)/SOC(0.1%)
//     0x92 최고·최저 온도(40 offset,℃)
//     0x93 충방전상태(0정지/1충전/2방전)/잔여용량(mAh)
//     0x94 충전기·부하 연결상태, 셀/온도 개수
//     0x98 고장/알람 비트
//   출력: sensor_msgs/BatteryState (/bms/state)
//
//   ⚠️ BMS 는 250K 라 모터 CAN(500K)과 다른 버스/비트레이트여야 한다.
//      bms_can_device 를 250K 로 올린 인터페이스로 지정할 것.
// ============================================================================
class BmsDriver {
public:
    BmsDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~BmsDriver();

private:
    void canCallback(const can_frame& frame);
    void pollTimer(const ros::TimerEvent&);
    void publishTimer(const ros::TimerEvent&);
    void sendRequest(uint8_t data_id);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher battery_pub_;
    ros::Timer poll_timer_;
    ros::Timer publish_timer_;
    std::unique_ptr<CanInterface> can_;

    // 파라미터
    std::string can_device_{"can1"};
    std::string frame_id_{"bms"};
    double poll_rate_{2.0};        // [Hz] 요청 주기
    double publish_rate_{2.0};     // [Hz] /bms/state 발행 주기
    double timeout_sec_{3.0};      // 응답 없으면 present=false
    double design_capacity_{0.0};  // [Ah] 0=미지정
    uint8_t bms_addr_{0x01};
    uint8_t host_addr_{0x40};

    // 최신 파싱값 (data_mutex_ 보호)
    std::mutex data_mutex_;
    bool got_data_{false};
    float voltage_{0.0f};              // [V]
    float current_{0.0f};              // [A] (+충전/-방전)
    float soc_{0.0f};                  // [%]
    float remain_capacity_ah_{0.0f};   // [Ah]
    uint8_t cd_state_{0};              // 0 정지, 1 충전, 2 방전
    uint8_t charger_status_{0};        // 0 미연결, 1 연결
    uint8_t load_status_{0};
    uint8_t num_cells_{0};
    uint8_t num_temps_{0};
    float max_temp_{0.0f};             // [℃]
    float min_temp_{0.0f};             // [℃]
    bool has_fault_{false};
    ros::Time last_rx_;
};
