#pragma once
#include <string>

// Minimal blocking-read serial port wrapper for WSL/Linux.
// Talks to the ESP32 over the USB-CDC serial device (e.g. /dev/ttyUSB0).
class SerialPort
{
public:
    ~SerialPort();

    // device e.g. "/dev/ttyUSB0", baud e.g. 115200
    bool open(const std::string &device, int baud);
    void close();

    // Reads one line (up to '\n'), blocking. Strips the trailing newline.
    // Returns false if the port is closed or a read error occurs.
    bool readLine(std::string &line);

private:
    int fd_ = -1;
};