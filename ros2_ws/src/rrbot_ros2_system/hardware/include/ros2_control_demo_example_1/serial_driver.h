
#ifndef RRBOT_ROS2_SYSTEM_SERIAL_DRIVER_H
#define RRBOT_ROS2_SYSTEM_SERIAL_DRIVER_H

#include <libserial/SerialPort.h>

#include <iostream>
#include <string>
#include <utility>

using namespace LibSerial;

class SerialDriver
{
public:
    SerialDriver();
    SerialDriver(std::string port_name, int baudrate);
    void configure(std::string port_name="/dev/ttyACM0", int baudrate=115200, int timeout_ms=20);
    LibSerial::BaudRate convertBaudrate(int baudrate);
    std::string readLine();
    void writeLine(std::string line);
    std::pair<int, double> getServoState(size_t nservo);
    bool connected();
    void close();
    
private:
    SerialPort  _serial_conn;
    int         _timeout_ms;
    bool        _connected;
    std::string _port_name;
    uint        _baud_rate;
};

#endif