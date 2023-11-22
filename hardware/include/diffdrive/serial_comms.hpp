#ifndef DIFFDRIVE_SERIAL_COMMS_HPP
#define DIFFDRIVE_SERIAL_COMMS_HPP

// #include <cstring>
#include <sstream>
 //#include <cstdlib>
//#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <iostream>


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
        return serial_conn_.good();
    }


    void read_enc_values(double& wheel_l, double& wheel_r)
    {
        char rxbuffer[4];
        // Responses end with \r\n so we will read up to (and including) the \n.
        serial_conn_.read(rxbuffer, 4);
        if(static_cast<int>(rxbuffer[0]) == 0)
        {
            wheel_l += 3*static_cast<int>(rxbuffer[1]);
        }else
        {
            wheel_l += -3*static_cast<int>(rxbuffer[1]);
        }
        if(static_cast<int>(rxbuffer[2]) == 0)
        {
            wheel_r += 3*static_cast<int>(rxbuffer[3]);
        }else
        {
            wheel_r += -3*static_cast<int>(rxbuffer[3]);
        }
        
    }
    void set_motor_values(int wheel_r_rpm, int wheel_l_rpm)
    {
        char txbuffer[2];
        
        txbuffer[0] = wheel_l_rpm;
        txbuffer[1] = wheel_r_rpm;
        serial_conn_.FlushIOBuffers(); // Just in case
        serial_conn_.write(txbuffer,2);
    }

private:
    LibSerial::SerialStream serial_conn_;
    int timeout_ms_;
};

#endif // DIFFDRIVE_SERIAL_COMMS_HPP
