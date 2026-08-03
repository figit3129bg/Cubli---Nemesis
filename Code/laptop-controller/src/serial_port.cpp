// serial_port.cpp
#include "serial_port.h"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <unistd.h>

SerialPort::SerialPort(const std::string& device, int baud) {
    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        throw std::runtime_error("Failed to open " + device + ": " + strerror(errno));
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        throw std::runtime_error("tcgetattr failed on " + device);
    }

    speed_t speed = baudToSpeed(baud);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    cfmakeraw(&tty);       // no line discipline - we do our own line buffering below
    tty.c_cc[VMIN]  = 1;   // block until at least 1 byte is available
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        throw std::runtime_error("tcsetattr failed on " + device);
    }
}

SerialPort::~SerialPort() {
    if (fd_ >= 0) close(fd_);
}

std::string SerialPort::readLine() {
    while (true) {
        auto nl = buffer_.find('\n');
        if (nl != std::string::npos) {
            std::string line = buffer_.substr(0, nl);
            buffer_.erase(0, nl + 1);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            return line;
        }

        char chunk[256];
        ssize_t n = read(fd_, chunk, sizeof(chunk));
        if (n <= 0) {
            throw std::runtime_error("Serial read failed - port closed or unplugged");
        }
        buffer_.append(chunk, n);
    }
}

speed_t SerialPort::baudToSpeed(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:
            throw std::runtime_error("Unsupported baud rate: " + std::to_string(baud));
    }
}
