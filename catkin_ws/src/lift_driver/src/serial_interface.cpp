#include "lift_driver/serial_interface.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/ioctl.h>

#include <cstring>
#include <iostream>

namespace {
speed_t to_speed(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:     return B0;
    }
}
}  // namespace

SerialInterface::~SerialInterface() { close(); }

bool SerialInterface::open(const std::string& device, int baud) {
    speed_t spd = to_speed(baud);
    if (spd == B0) {
        std::cerr << "SerialInterface: unsupported baud " << baud << std::endl;
        return false;
    }
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "SerialInterface: failed to open " << device << std::endl;
        return false;
    }
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd_, &tio) != 0) {
        std::cerr << "SerialInterface: tcgetattr failed" << std::endl;
        ::close(fd_); fd_ = -1; return false;
    }
    cfmakeraw(&tio);                 // raw 모드
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);
    tio.c_cflag |= (CLOCAL | CREAD); // 로컬 연결, 수신 활성
    tio.c_cflag &= ~PARENB;          // no parity
    tio.c_cflag &= ~CSTOPB;          // 1 stop bit
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;              // 8 data bits
    tio.c_cflag &= ~CRTSCTS;         // no HW flow control
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        std::cerr << "SerialInterface: tcsetattr failed" << std::endl;
        ::close(fd_); fd_ = -1; return false;
    }
    tcflush(fd_, TCIOFLUSH);

    // DTR+RTS 를 asserted 상태로 둔다. FTDI USB-RS485 어댑터는 이 라인이 켜져야
    // 트랜시버가 동작한다(Windows 는 자동 assert, Linux 기본은 deassert → 무응답).
    int mbits = TIOCM_DTR | TIOCM_RTS;
    ioctl(fd_, TIOCMBIS, &mbits);

    std::cout << "Serial opened: " << device << " @ " << baud << " 8N1 (DTR+RTS asserted)" << std::endl;
    return true;
}

void SerialInterface::close() {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool SerialInterface::write(const uint8_t* data, size_t n) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (fd_ < 0) return false;
    size_t sent = 0;
    while (sent < n) {
        ssize_t w = ::write(fd_, data + sent, n - sent);
        if (w < 0) return false;
        sent += static_cast<size_t>(w);
    }
    return true;
}

int SerialInterface::read(uint8_t* buf, size_t n, int timeout_ms) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (fd_ < 0) return -1;
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    int r = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (r <= 0) return r;  // 0=timeout, <0=error
    ssize_t got = ::read(fd_, buf, n);
    return static_cast<int>(got);
}
