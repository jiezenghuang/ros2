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

    for(int i = 0; i < 100; ++i)
    {
        std::cout << SmartCar::Instance().get_distance << std::endl;
        std::this_thread::sleep_for(200ms);
    }
}