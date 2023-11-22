#ifndef DIFFDRIVE_WHEEL_HPP
#define DIFFDRIVE_WHEEL_HPP

#include <string>
#include <cmath>


class Wheel
{
public:

    std::string name = "";
    double cmd = 0;
    double pos = 0;
    double vel = 0; // rad/s
    double enc = 0;

    Wheel() = default;

    Wheel(const std::string& wheel_name)
    {
        setup(wheel_name);
    }

    void setup(const std::string& wheel_name)
    {
        name = wheel_name;
    }


};


#endif // DIFFDRIVE_WHEEL_HPP
