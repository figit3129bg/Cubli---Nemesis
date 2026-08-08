#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <string>

// Minimal blocking line-reader over a POSIX tty (e.g. /dev/ttyUSB0).
// Linux/WSL only -- uses termios.
class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort &) = delete;
    SerialPort &operator=(const SerialPort &) = delete;

    bool open(const std::string &device, int baudRate);
    // Blocks until a '\n'-terminated line arrives (trailing '\r' stripped)
    // or the port errors/closes, in which case it returns false.
    bool readLine(std::string &line);
    void close();

private:
    int fd_ = -1;
    std::string buffer_;
};

#endif // SERIAL_PORT_H
