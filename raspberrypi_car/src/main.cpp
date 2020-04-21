#include <iostream>
#include <chrono>
#include <thread>

#include "car_device.h"
#include "smart_car.h"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    if(!SmartCar::Instance().init())
    {
        std::cerr << "car init fail" << std::endl;
        return -1;
    }

    int status = 0;
    for(int i = 0; i < 100; ++i)
    {
        std::cout << SmartCar::Instance().get_infrared_sensor(SENSOR_LEFT_INFRARED);
        std::this_thread::sleep_for(20ms);
    }
    std::cout << std::endl;
}