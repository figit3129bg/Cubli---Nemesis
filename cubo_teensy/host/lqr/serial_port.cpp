#include "serial_port.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace {

speed_t toSpeed(int baudRate) {
    switch (baudRate) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:     return B115200;
    }
}

} // namespace

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string &device, int baudRate) {
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    const speed_t speed = toSpeed(baudRate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
}

bool SerialPort::readLine(std::string &line) {
    if (fd_ < 0) {
        return false;
    }

    for (;;) {
        const auto pos = buffer_.find('\n');
        if (pos != std::string::npos) {
            line = buffer_.substr(0, pos);
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            buffer_.erase(0, pos + 1);
            return true;
        }

        char chunk[256];
        const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
        if (n <= 0) {
            return false;
        }
        buffer_.append(chunk, static_cast<size_t>(n));
    }
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}
