#pragma once

#include <string>

namespace charlie_base_driver {

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    bool open_port(const std::string & port_name, int baud_rate);
    void close_port();

    bool is_open() const;

    bool write_string(const std::string & data);
    bool read_line(std::string & line);

private:
    int baud_to_termios(int baud_rate) const;

    int fd_;
    std::string read_buffer_;
};

}  // namespace charlie_base_driver