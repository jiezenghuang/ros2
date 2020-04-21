#include <iostream>

#include "smart_car.h"
#include "state_machine.hpp"

int main(int argc, char** argv)
{
    if(!SmartCar::Instance().init())
    {
        std::cerr << "car init fail" << std::endl;
        return -1;
    }
}