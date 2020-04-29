#include <iostream>
#include <thread>
#include <chrono>

#include "raspberrypi_car/include/car_device.h"
#include "raspberrypi_car/include/smart_car.h"

using namespace std;
using namespace std::chrono_literals;

int main(int argc, char** argv)
{   
    if(SmartCar::Instance().init())
    { 
    	do
    	{
        	cout << "Distance: " << SmartCar::Instance().get_distance() << endl;
        	cout << "Left: " << SmartCar::Instance().get_infrared_sensor(SENSOR_LEFT_INFRARED) << endl;
        	cout << "Right:" << SmartCar::Instance().get_infrared_sensor(SENSOR_RIGHT_INFRARED) << endl;
        	std::this_thread::sleep_for(100ms);
    	} while (true);
    }
    
    return 0;
}
