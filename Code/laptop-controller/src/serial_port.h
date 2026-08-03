// serial_port.h
#pragma once
#include <string>
#include <termios.h>

// Minimal blocking line-reader over a POSIX serial device (e.g. /dev/ttyUSB0).
class SerialPort {
public:
    SerialPort(const std::string& device, int baud);
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Blocks until a full '\n'-terminated line has been read.
    std::string readLine();

private:
    static speed_t baudToSpeed(int baud);

    int fd_ = -1;
    std::string buffer_;
};
