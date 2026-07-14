#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <mutex>

// 간단한 SocketCAN 스타일 시리얼(termios) 래퍼.
//   8 data bits, no parity, 1 stop bit (8N1) 고정, 보드레이트만 지정.
//   RS485/RS232 어댑터(예: /dev/ttyS0=COM1) 를 그대로 사용.
class SerialInterface {
public:
    SerialInterface() = default;
    ~SerialInterface();

    bool open(const std::string& device, int baud);  // baud: 9600/19200/... (지원되는 termios 속도)
    void close();
    bool is_open() const { return fd_ >= 0; }

    // 전체 버퍼 송신 (성공 시 true)
    bool write(const uint8_t* data, size_t n);
    // 최대 n바이트를 timeout_ms 안에 읽어 buf 에 저장, 읽은 바이트 수 반환(0=타임아웃, <0=에러)
    int read(uint8_t* buf, size_t n, int timeout_ms);

private:
    int fd_{-1};
    std::mutex io_mutex_;
};
