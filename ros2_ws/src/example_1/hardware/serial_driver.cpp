#include "ros2_control_demo_example_1/serial_driver.h"
#include <sstream>
#include <iostream>
#include "rclcpp/rclcpp.hpp" 

SerialDriver::SerialDriver()
    : _port_name{}, _baud_rate{115200}, _timeout_ms{1000}, _connected{false}
{}

SerialDriver::SerialDriver(const std::string& port_name, int baudrate)
    : _port_name(port_name), _baud_rate(baudrate), _timeout_ms{1000}, _connected{false}
{}

bool SerialDriver::configure(const std::string& port_name, int baudrate, int timeout_ms)
{
    _port_name = port_name;
    _baud_rate = baudrate;
    _timeout_ms = timeout_ms;

    try
    {
        if (!_serial_conn.IsOpen())
        {
            _serial_conn.Open(_port_name);
        }
        _serial_conn.SetBaudRate(convertBaudrate(_baud_rate));
        _connected = _serial_conn.IsOpen();
    }
    catch (const LibSerial::OpenFailed&)
    {
        std::cerr << "The serial port did not open correctly." << std::endl;
        _connected = false;
    }

    return _connected;
}

LibSerial::BaudRate SerialDriver::convertBaudrate(int baudrate)
{
    switch (baudrate)
    {
        case 1200: return LibSerial::BaudRate::BAUD_1200;
        case 1800: return LibSerial::BaudRate::BAUD_1800;
        case 2400: return LibSerial::BaudRate::BAUD_2400;
        case 4800: return LibSerial::BaudRate::BAUD_4800;
        case 9600: return LibSerial::BaudRate::BAUD_9600;
        case 19200: return LibSerial::BaudRate::BAUD_19200;
        case 38400: return LibSerial::BaudRate::BAUD_38400;
        case 57600: return LibSerial::BaudRate::BAUD_57600;
        case 115200: return LibSerial::BaudRate::BAUD_115200;
        case 230400: return LibSerial::BaudRate::BAUD_230400;
        default:
            std::cout << "Error! Baud rate " << baudrate << " not supported! Defaulting to 57600.\n";
            return LibSerial::BaudRate::BAUD_57600;
    }
}

std::string SerialDriver::readLine()
{
    if (!_serial_conn.IsOpen())
    {
        throw LibSerial::NotOpen("Serial port not open on readLine()");
    }

    std::string msg;
    char next_char;

    while (true)
    {
        _serial_conn.ReadByte(next_char, _timeout_ms);
        if (next_char == '\n') break;
        msg += next_char;
    }

    return msg;
}

std::pair<int, double> SerialDriver::getServoState(int servo_index)
{
    writeLine("e " + std::to_string(servo_index));
    const std::string response = readLine();

    std::istringstream iss(response);
    std::string token1, token2;
    iss >> token1 >> token2;

    try
    {
        const int id = std::stoi(token1);
        const double radians = std::stod(token2);
        return {id, radians};
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
            "Invalid servo feedback format: '%s'", response.c_str()
        );
        return {servo_index, 0.0};
    }
}

void SerialDriver::writeLine(const std::string& line)
{
    if (!_serial_conn.IsOpen())
    {
        throw LibSerial::NotOpen("Serial port not open on writeLine()");
    }

    _serial_conn.Write(line);
    _serial_conn.DrainWriteBuffer();
}

bool SerialDriver::connected() const
{
    return _serial_conn.IsOpen();
}

void SerialDriver::close()
{
    if (_serial_conn.IsOpen())
    {
        _serial_conn.Close();
    }
    _connected = false;
}
