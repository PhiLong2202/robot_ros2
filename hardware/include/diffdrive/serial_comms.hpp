#ifndef DIFFDRIVE_SERIAL_COMMS_HPP
#define DIFFDRIVE_SERIAL_COMMS_HPP

// #include <cstring>
#include <sstream>
 //#include <cstdlib>
//#include <libserial/SerialPort.h>
#include <libserial/SerialPort.h>
#include <iostream>
#include <string>
#include <math.h> 

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
    // Just handle some common baud rates
    switch (baud_rate)
    {
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    default:
        std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 9600" << std::endl;
        return LibSerial::BaudRate::BAUD_9600;
    }
}

class SerialComms
{

public:

    SerialComms() = default;

    void connect(const std::string& serial_device, int32_t baud_rate, int32_t timeout_ms)
    {
        timeout_ms_ = timeout_ms;
        serial_conn_.Open(serial_device);
        serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    }

    void disconnect()
    {
        serial_conn_.Close();
    }

    bool connected() const
    {
        return serial_conn_.IsOpen();
    }


    void read_enc_values(int& wheel_l, int& wheel_r)
    {
        std::string response = "";
        // Responses end with \r\n so we will read up to (and including) the \n.
        try
        {
          // Responses end with \r\n so we will read up to (and including) the \n.
          serial_conn_.ReadLine(response, '\n', 1000);
        }
        catch (const LibSerial::ReadTimeout&)
        {
           std::cerr << "The ReadByte() call has timed out." << std::endl ;
        }
        std::string split = ",";
        size_t sub = response.find(",");
        std::string token_1 = response.substr(0, sub);
        std::string token_2 = response.substr(sub + split.length());
        wheel_l += std::stoi(token_1.c_str());
        wheel_r += std::stoi(token_2.c_str());
        
    }
    void set_motor_values(int wheel_r_rpm, int wheel_l_rpm)
    {
        char txbuffer[4];
        txbuffer[0] = 0x02;
        txbuffer[1] = wheel_r_rpm;
        txbuffer[2] = wheel_l_rpm;
        txbuffer[3] = 0x03;
        serial_conn_.FlushIOBuffers(); // Just in case
        for( int i=0; i<4; i++ )
        {
            serial_conn_.WriteByte(txbuffer[i]);
        }
    }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // DIFFDRIVE_SERIAL_COMMS_HPP
