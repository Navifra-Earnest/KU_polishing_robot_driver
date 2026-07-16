#pragma once

#include <linux/can.h>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

// SocketCAN(RAW) 래퍼. 특정 어댑터에 종속되지 않으며, can0 등 표준
// SocketCAN 네트워크 디바이스를 그대로 사용한다.
class CanInterface {
public:
    CanInterface();
    ~CanInterface();

    bool open(const std::string& can_device);
    void close();

    void write(const can_frame& msg);
    void write(uint32_t can_id, const uint8_t* data, uint8_t dlc);
    void write(uint32_t can_id, const std::vector<uint8_t>& data);
    void write(uint32_t can_id, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7, uint8_t b8);
    void register_callback(const std::function<void(const can_frame&)>& callback);

private:
    void read_loop();

    int can_fd_{-1};
    std::thread read_thread_;
    std::mutex write_mutex_;
    std::function<void(const can_frame&)> callback_{nullptr};
    std::atomic<bool> is_running_{false};
};
