#include "can_interface/can_interface.hpp"

#include <fcntl.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

CanInterface::CanInterface() {}

CanInterface::~CanInterface() {
    if (is_running_) {
        close();
    }
}

bool CanInterface::open(const std::string& can_device) {
    can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd_ < 0) {
        perror("Socket");
        return false;
    }
    struct ifreq ifr;
    strncpy(ifr.ifr_name, can_device.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        ::close(can_fd_);
        can_fd_ = -1;
        return false;
    }
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind");
        ::close(can_fd_);
        can_fd_ = -1;
        return false;
    }
    is_running_ = true;
    read_thread_ = std::thread(&CanInterface::read_loop, this);
    std::cout << "CAN interface opened on " << can_device << std::endl;
    return true;
}

void CanInterface::close() {
    is_running_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
    if (can_fd_ >= 0) {
        ::close(can_fd_);
        can_fd_ = -1;
        std::cout << "CAN interface closed." << std::endl;
    }
}

void CanInterface::write(const can_frame& msg) {
    std::lock_guard<std::mutex> lock(write_mutex_);
    if (can_fd_ < 0) return;
    if (::write(can_fd_, &msg, sizeof(can_frame)) != sizeof(can_frame)) {
        perror("CAN write error");
    }
}

void CanInterface::write(uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    if (dlc > 8) return;
    can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = dlc;
    if (data) {
        memcpy(frame.data, data, dlc);
    }
    write(frame);
}

void CanInterface::write(uint32_t can_id, const std::vector<uint8_t>& data) {
    if (data.size() > 8) return;
    can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = data.size();
    memcpy(frame.data, data.data(), data.size());
    write(frame);
}

void CanInterface::write(uint32_t can_id, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7, uint8_t b8) {
    can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = 8;
    frame.data[0] = b1;
    frame.data[1] = b2;
    frame.data[2] = b3;
    frame.data[3] = b4;
    frame.data[4] = b5;
    frame.data[5] = b6;
    frame.data[6] = b7;
    frame.data[7] = b8;
    write(frame);
}

void CanInterface::register_callback(const std::function<void(const can_frame&)>& callback) {
    callback_ = callback;
}

void CanInterface::read_loop() {
    while (is_running_) {
        if (can_fd_ < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        can_frame frame;
        ssize_t nbytes = read(can_fd_, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (nbytes == sizeof(struct can_frame) && callback_) {
            callback_(frame);
        }
    }
}
