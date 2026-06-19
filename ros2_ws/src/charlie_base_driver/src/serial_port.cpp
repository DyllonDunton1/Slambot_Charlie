#include "charlie_base_driver/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace charlie_base_driver {

SerialPort::SerialPort()
: fd_(-1)
{
}

SerialPort::~SerialPort()
{
    close_port();
}

bool SerialPort::open_port(const std::string & port_name, int baud_rate)
{
    close_port();

    fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd_ < 0) {
        return false;
    }

    termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
        close_port();
        return false;
    }

    cfmakeraw(&tty);

    const int baud = baud_to_termios(baud_rate);
    if (baud < 0) {
        close_port();
        return false;
    }

    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag |= static_cast<unsigned int>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<unsigned int>(~PARENB);
    tty.c_cflag &= static_cast<unsigned int>(~CSTOPB);
    tty.c_cflag &= static_cast<unsigned int>(~CSIZE);
    tty.c_cflag |= CS8;
    tty.c_cflag &= static_cast<unsigned int>(~CRTSCTS);

    tty.c_iflag &= static_cast<unsigned int>(~(IXON | IXOFF | IXANY));
    tty.c_lflag = 0;
    tty.c_oflag = 0;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        close_port();
        return false;
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
}

void SerialPort::close_port()
{
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }

    read_buffer_.clear();
}

bool SerialPort::is_open() const
{
    return fd_ >= 0;
}

bool SerialPort::write_string(const std::string & data)
{
    if (fd_ < 0) {
        return false;
    }

    const char * buffer = data.c_str();
    std::size_t remaining = data.size();

    while (remaining > 0) {
        const ssize_t written = write(fd_, buffer, remaining);

        if (written < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            return false;
        }

        buffer += written;
        remaining -= static_cast<std::size_t>(written);
    }

    return true;
}

bool SerialPort::read_line(std::string & line)
{
    line.clear();

    if (fd_ < 0) {
        return false;
    }

    char buffer[256];

    while (true) {
        const ssize_t bytes_read = read(fd_, buffer, sizeof(buffer));

        if (bytes_read > 0) {
            read_buffer_.append(buffer, static_cast<std::size_t>(bytes_read));
        }
        else if (bytes_read < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            return false;
        }
        else {
            break;
        }
    }

    const std::size_t newline_pos = read_buffer_.find('\n');

    if (newline_pos == std::string::npos) {
        return false;
    }

    line = read_buffer_.substr(0, newline_pos);
    read_buffer_.erase(0, newline_pos + 1);

    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }

    return true;
}

int SerialPort::baud_to_termios(int baud_rate) const
{
    switch (baud_rate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return -1;
    }
}

}  // namespace charlie_base_driver