#include "serial_port.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

static speed_t baudToSpeed(int baud)
{
    switch (baud)
    {
        case 9600:   return B9600;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:     return B115200; // fallback, matches your firmware's Serial.begin()
    }
}

bool SerialPort::open(const std::string &device, int baud)
{
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0)
    {
        return false; // check: does this device exist? permissions? (see note below)
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0)
    {
        close();
        return false;
    }

    cfsetospeed(&tty, baudToSpeed(baud));
    cfsetispeed(&tty, baudToSpeed(baud));

    tty.c_cflag &= ~PARENB;   // no parity
    tty.c_cflag &= ~CSTOPB;   // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;       // 8 data bits
    tty.c_cflag &= ~CRTSCTS;  // no hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;   // raw mode, not line-buffered by the OS
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN]  = 1; // block until at least 1 byte is available
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
        close();
        return false;
    }

    return true;
}

void SerialPort::close()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::readLine(std::string &line)
{
    if (fd_ < 0) return false;

    line.clear();
    char c;
    while (true)
    {
        ssize_t n = ::read(fd_, &c, 1);
        if (n <= 0)
        {
            return false; // port closed / error / device unplugged
        }
        if (c == '\n')
        {
            return true;
        }
        if (c != '\r') // ESP32's println() sends \r\n, drop the \r
        {
            line += c;
        }
    }
}

SerialPort::~SerialPort()
{
    close();
}