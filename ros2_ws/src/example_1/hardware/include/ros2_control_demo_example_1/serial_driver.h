#ifndef RRBOT_ROS2_SYSTEM_SERIAL_DRIVER_H
#define RRBOT_ROS2_SYSTEM_SERIAL_DRIVER_H

#include <libserial/SerialPort.h>
#include <iostream>
#include <string>
#include <utility>

class SerialDriver
{
public:
    SerialDriver();
    SerialDriver(const std::string& port_name, int baudrate);

    bool configure(const std::string& port_name = "/dev/ttyACM0",
                   int baudrate = 115200,
                   int timeout_ms = 20);

    std::pair<int, double> getServoState(int servo_index);
    void writeLine(const std::string& line);
    std::string readLine();
    void close();
    bool connected() const;

private:
    LibSerial::SerialPort _serial_conn;
    std::string _port_name{};
    int _baud_rate{115200};
    int _timeout_ms{1000};
    bool _connected{false};

    LibSerial::BaudRate convertBaudrate(int baudrate);
};

#endif  // RRBOT_ROS2_SYSTEM_SERIAL_DRIVER_H
