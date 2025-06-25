#include <rrbot_ros2_system/serial_driver.h>


SerialDriver::SerialDriver()
{
    // _serial_conn.Open("/dev/ttyACM0");
    // _serial_conn.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
}

SerialDriver::SerialDriver(std::string port_name, int baudrate)
    : _port_name(port_name), _baud_rate(baudrate)
{
    // _serial_conn.Open(port_name);
    // _serial_conn.SetBaudRate(convertBaudrate(baudrate));
}

void SerialDriver::configure(std::string port_name, int baudrate, int timeout_ms)
{
    try
    {
        _serial_conn.Open(port_name);
        _connected = true;
        _serial_conn.SetBaudRate(convertBaudrate(baudrate));
        _timeout_ms = timeout_ms;
    }
    catch(const LibSerial::OpenFailed&)
    {
        std::cerr << "The serial port did not open correctly." << std::endl;
        _connected = false;
    }
}

LibSerial::BaudRate SerialDriver::convertBaudrate(int baudrate)
{
  // Just handle some common baud rates
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
      std::cout << "Error! Baud rate " << baudrate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

std::string SerialDriver::readLine()
{
    std::string msg{};
    char next_char{};
    while(1)
    {
        _serial_conn.ReadByte(next_char, 0);
        if(next_char == '\n') return msg;
        msg += next_char;
    }

}

std::pair<int, double> SerialDriver::getServoState(uint8_t nservo)
{
    std::string msg{"e "};
    msg += std::to_string(nservo) + "\n";
    writeLine(msg);

    msg = readLine();
    size_t last = 0; size_t next = 0;
    next = msg.find(" ", last);
    int servo = std::stoi(msg.substr(last, next-last));
    last = next + 1;
    next = msg.find(" ", last);
    double angle = std::stod(msg.substr(last, next-last));
    
    return std::make_pair(servo, angle);
}

void SerialDriver::writeLine(std::string line)
{
    _serial_conn.Write(line);
    _serial_conn.DrainWriteBuffer();
}

bool SerialDriver::connected()
{
    return _connected;
}

void SerialDriver::close()
{
    _serial_conn.Close();
}