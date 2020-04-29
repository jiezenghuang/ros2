#include <iostream>
#include <thread>
#include <chrono>

#include "car_device.h"
#include "smart_car.h"

using namespace std;
using namespace std::chrono_literals;

int main(int argc, char** argv)
{   
    if(SmartCar::Instance().init())
    {
		string cmd; 
		float speed;
    	do
    	{
			cout << "Please input command:";
			cin >> cmd;
			cin >> speed;
			cout << "Command: " << cmd << ", Speed: " << speed << endl;
			if(cmd == "go")
				SmartCar::Instance().go(speed);

			if(cmd == "back")
				SmartCar::Instance().back(speed);

			if(cmd == "turn_left")
				SmartCar::Instance().turn_left(speed);

			if(cmd == "turn_right")
				SmartCar::Instance().turn_right(speed);
			
			if(cmd == "spin_left")
				SmartCar::Instance().spin_left(speed);

			if(cmd == "spin_right")
				SmartCar::Instance().spin_right(speed);
				
        	cout << "Distance: " << SmartCar::Instance().get_distance() << endl;
        	cout << "Left: " << SmartCar::Instance().get_infrared_sensor(SENSOR_LEFT_INFRARED) << endl;
        	cout << "Right:" << SmartCar::Instance().get_infrared_sensor(SENSOR_RIGHT_INFRARED) << endl;

        	std::this_thread::sleep_for(3s);

			SmartCar::Instance().stop();
    	} while (true);
    }
    
    return 0;
}
