#include <iostream>
#include <thread>

#include "car_device.h"
#include "smart_car.h"
#include "logger.hpp"

#ifdef __cplusplus
extern "C"
{
#endif
    #include <wiringPi.h>
    #include <softPwm.h>
#ifdef __cplusplus
}
#endif

using namespace std;

const int SERVO_PWM_RANGE = 200;
const int SERVO_PWM_L = 5;
const int SERVO_PWM_H = 25;
const int MOTOR_PWM_RANGE = 100;

const int CONTROL_DELAY = 20;

const float DEFAULT_SERVO_ANGLE = 90;

const int DEVICE_POWER_OR_BEE = 10;

const int DEVICE_US_ECHO = 30;
const int DEVICE_US_TRIGGER = 31;

const int DEVICE_RIGHT_MOTOR_PWM = 23;
const int DEVICE_RIGHT_MOTOR_GO = 24;
const int DEVICE_RIGHT_MOTOR_BACK = 25;

const int DEVICE_LEFT_MOTOR_PWM = 27;
const int DEVICE_LEFT_MOTOR_GO = 28;
const int DEVICE_LEFT_MOTOR_BACK = 29;

SmartCar::SmartCar()
{}

SmartCar::~SmartCar()
{}

SmartCar::SmartCar(const SmartCar&)
{}
	
SmartCar& SmartCar::operator=(const SmartCar&) 
{ 
    return SmartCar::Instance(); 
}

bool SmartCar::init()
{
    if(-1 == wiringPiSetup())
        return false;

    pinMode(DEVICE_LED_GREEN, OUTPUT);
    pinMode(DEVICE_LED_RED, OUTPUT);
    pinMode(DEVICE_LED_BLUE, OUTPUT);

    pinMode(DEVICE_US_H_SERVO, OUTPUT);
    pinMode(DEVICE_CAMERA_H_SERVO, OUTPUT);
    pinMode(DEVICE_CAMERA_V_SERVO, OUTPUT);

    pinMode(DEVICE_POWER_OR_BEE, INPUT);

    pinMode(DEVICE_FAN, OUTPUT);

    pinMode(SENSOR_LEFT_INFRARED, INPUT);
    pinMode(SENSOR_RIGHT_INFRARED, INPUT);

    pinMode(SENSOR_LEFT_LIGHT, INPUT);
    pinMode(SENSOR_RIGHT_LIGHT, INPUT);

    pinMode(SENSOR_LEFT_TRACK_1, INPUT);
    pinMode(SENSOR_LEFT_TRACK_2, INPUT);
    pinMode(SENSOR_RIGHT_TRACK_1, INPUT);
    pinMode(SENSOR_RIGHT_TRACK_2, INPUT);

    pinMode(DEVICE_US_ECHO, INPUT);
    pinMode(DEVICE_US_TRIGGER, OUTPUT);

    pinMode(DEVICE_RIGHT_MOTOR_GO, OUTPUT);
    pinMode(DEVICE_RIGHT_MOTOR_BACK, OUTPUT);

    pinMode(DEVICE_LEFT_MOTOR_GO, OUTPUT);
    pinMode(DEVICE_LEFT_MOTOR_BACK, OUTPUT);

    softPwmCreate(DEVICE_RIGHT_MOTOR_PWM, 0, MOTOR_PWM_RANGE);
    softPwmCreate(DEVICE_LEFT_MOTOR_PWM, 0, MOTOR_PWM_RANGE);

    softPwmCreate(DEVICE_US_H_SERVO, 0, SERVO_PWM_RANGE);
    softPwmCreate(DEVICE_CAMERA_H_SERVO, 0, SERVO_PWM_RANGE);
    softPwmCreate(DEVICE_CAMERA_V_SERVO, 0, SERVO_PWM_RANGE);

    digitalWrite(DEVICE_LED_RED, LOW);
    digitalWrite(DEVICE_LED_GREEN, LOW);
    digitalWrite(DEVICE_LED_BLUE, LOW);

    digitalWrite(DEVICE_FAN, HIGH);

    return true;
}

void SmartCar::set_led(int id, int val)
{
    if(id == DEVICE_LED_RED || id == DEVICE_LED_GREEN || id == DEVICE_LED_BLUE)
    {
        Logger::get_logger().debug("set led %d to %d", id, val);
        digitalWrite(id, val == 0 ? LOW : HIGH);
    }
}

void SmartCar::set_fan(int val)
{
    Logger::get_logger().debug("set fan %d", val);
    val == 0 ? digitalWrite(DEVICE_FAN, HIGH) : digitalWrite(DEVICE_FAN, LOW);
}

void SmartCar::set_servo_angle(int id, float angle)
{
    if(angle < 0 || angle > 180)
        return;
    
    if(id == DEVICE_US_H_SERVO || id == DEVICE_CAMERA_H_SERVO || id == DEVICE_CAMERA_V_SERVO)
    {
        Logger::get_logger().debug("set servo %d angle to %f", id, angle);
        int pluse = angle * (SERVO_PWM_H - SERVO_PWM_L) / 180  + SERVO_PWM_L;  
        softPwmWrite(id, pluse);      
    }
}

float SmartCar::get_distance()
{    
    digitalWrite(DEVICE_US_TRIGGER, LOW);
    delayMicroseconds(2);
    digitalWrite(DEVICE_US_TRIGGER, HIGH);
    delayMicroseconds(20);
    digitalWrite(DEVICE_US_TRIGGER, LOW);

    auto start = system_clock::now();
    auto end = start;
    const int max_respone_time = 30000;
    while(digitalRead(DEVICE_US_ECHO) != HIGH)
    {
        end = system_clock::now();
        auto duration = duration_cast<microseconds>(end - start);
        if(duration.count() > max_respone_time)
        {
            return -1;
        }
    }

    start = system_clock::now();
    while(digitalRead(DEVICE_US_ECHO) != LOW)
    {
        end = system_clock::now();
        auto duration = duration_cast<microseconds>(end - start);
        if(duration.count() > max_respone_time)
        {
            return max_respone_time * 0.34 / 2;
        }
    }

    end = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    return float(duration.count()) * 0.34 / 2;
}

int SmartCar::get_infrared_sensor(int id)
{
    if(id == SENSOR_LEFT_INFRARED || id == SENSOR_RIGHT_INFRARED)
        return digitalRead(id);
    else
        return -1;
}

int SmartCar::get_light_sensor(int id)
{
    if(id == SENSOR_LEFT_LIGHT || id == SENSOR_RIGHT_LIGHT)
        return digitalRead(id);
    else
        return -1;
}

int SmartCar::get_track_sensor(int id)
{
    if(id == SENSOR_LEFT_TRACK_1 || id == SENSOR_LEFT_TRACK_2
            || id == SENSOR_RIGHT_TRACK_1 || id == SENSOR_RIGHT_TRACK_2)
        return digitalRead(id);
    else
        return -1;
}

void SmartCar::go(float speed)
{
    digitalWrite(DEVICE_RIGHT_MOTOR_GO, HIGH);
    digitalWrite(DEVICE_RIGHT_MOTOR_BACK, LOW);
    softPwmWrite(DEVICE_RIGHT_MOTOR_PWM, MOTOR_PWM_RANGE * speed);

    digitalWrite(DEVICE_LEFT_MOTOR_GO, HIGH);
    digitalWrite(DEVICE_LEFT_MOTOR_BACK, LOW);
    softPwmWrite(DEVICE_LEFT_MOTOR_PWM, MOTOR_PWM_RANGE * speed);   

    Logger::get_logger().debug("car go, left speed %d, right speed %d", MOTOR_PWM_RANGE * speed, MOTOR_PWM_RANGE * speed); 
}

void SmartCar::back(float speed)
{
    digitalWrite(DEVICE_RIGHT_MOTOR_GO, LOW);
    digitalWrite(DEVICE_RIGHT_MOTOR_BACK, HIGH);
    softPwmWrite(DEVICE_RIGHT_MOTOR_PWM, MOTOR_PWM_RANGE * speed);

    digitalWrite(DEVICE_LEFT_MOTOR_GO, LOW);
    digitalWrite(DEVICE_LEFT_MOTOR_BACK, HIGH);
    softPwmWrite(DEVICE_LEFT_MOTOR_PWM, MOTOR_PWM_RANGE * speed);

    Logger::get_logger().debug("car back, left speed %d, right speed %d", MOTOR_PWM_RANGE * speed, MOTOR_PWM_RANGE * speed); 
}

void SmartCar::stop()
{
    digitalWrite(DEVICE_RIGHT_MOTOR_GO, LOW);
    digitalWrite(DEVICE_RIGHT_MOTOR_BACK, LOW);
    softPwmWrite(DEVICE_RIGHT_MOTOR_PWM, 0);

    digitalWrite(DEVICE_LEFT_MOTOR_GO, LOW);
    digitalWrite(DEVICE_LEFT_MOTOR_BACK, LOW);
    softPwmWrite(DEVICE_LEFT_MOTOR_PWM, 0);

    Logger::get_logger().debug("car stop"); 
}

void SmartCar::turn_left(float speed)
{
    digitalWrite(DEVICE_RIGHT_MOTOR_GO, HIGH);
    digitalWrite(DEVICE_RIGHT_MOTOR_BACK, LOW);
    softPwmWrite(DEVICE_RIGHT_MOTOR_PWM, MOTOR_PWM_RANGE * speed);

    digitalWrite(DEVICE_LEFT_MOTOR_GO, HIGH);
    digitalWrite(DEVICE_LEFT_MOTOR_BACK, LOW);
    softPwmWrite(DEVICE_LEFT_MOTOR_PWM, MOTOR_PWM_RANGE * speed * (1 - SpeedDelta));

    Logger::get_logger().debug("car turn left, left speed %d, right speed %d", MOTOR_PWM_RANGE * speed * (1 - SpeedDelta), MOTOR_PWM_RANGE * speed); 
}

void SmartCar::turn_right(float speed)
{
    digitalWrite(DEVICE_RIGHT_MOTOR_GO, HIGH);
    digitalWrite(DEVICE_RIGHT_MOTOR_BACK, LOW);
    softPwmWrite(DEVICE_RIGHT_MOTOR_PWM, MOTOR_PWM_RANGE * speed * (1 - SpeedDelta));

    digitalWrite(DEVICE_LEFT_MOTOR_GO, HIGH);
    digitalWrite(DEVICE_LEFT_MOTOR_BACK, LOW);
    softPwmWrite(DEVICE_LEFT_MOTOR_PWM, MOTOR_PWM_RANGE * speed);

    Logger::get_logger().debug("car turn right, left speed %d, right speed %d", MOTOR_PWM_RANGE * speed, MOTOR_PWM_RANGE * speed * (1 - SpeedDelta)); 
}

void SmartCar::spin_left(float speed)
{
    digitalWrite(DEVICE_RIGHT_MOTOR_GO, HIGH);
    digitalWrite(DEVICE_RIGHT_MOTOR_BACK, LOW);
    softPwmWrite(DEVICE_RIGHT_MOTOR_PWM, MOTOR_PWM_RANGE * speed);

    digitalWrite(DEVICE_LEFT_MOTOR_GO, LOW);
    digitalWrite(DEVICE_LEFT_MOTOR_BACK, HIGH);
    softPwmWrite(DEVICE_LEFT_MOTOR_PWM, MOTOR_PWM_RANGE * speed);

    Logger::get_logger().debug("car spin left, left speed %d, right speed %d", MOTOR_PWM_RANGE * speed, MOTOR_PWM_RANGE * speed); 
}

void SmartCar::spin_right(float speed)
{
    digitalWrite(DEVICE_RIGHT_MOTOR_GO, LOW);
    digitalWrite(DEVICE_RIGHT_MOTOR_BACK, HIGH);
    softPwmWrite(DEVICE_RIGHT_MOTOR_PWM, MOTOR_PWM_RANGE * speed);

    digitalWrite(DEVICE_LEFT_MOTOR_GO, HIGH);
    digitalWrite(DEVICE_LEFT_MOTOR_BACK, LOW);
    softPwmWrite(DEVICE_LEFT_MOTOR_PWM, MOTOR_PWM_RANGE * speed);

    Logger::get_logger().debug("car spin right, left speed %d, right speed %d", MOTOR_PWM_RANGE * speed, MOTOR_PWM_RANGE * speed); 
}

void SmartCar::shutdown()
{
    stop();

    softPwmWrite(DEVICE_US_H_SERVO, 0);
    softPwmWrite(DEVICE_CAMERA_H_SERVO, 0);
    softPwmWrite(DEVICE_CAMERA_V_SERVO, 0);

    digitalWrite(DEVICE_LED_RED, LOW);
    digitalWrite(DEVICE_LED_GREEN, LOW);
    digitalWrite(DEVICE_LED_BLUE, LOW);

    digitalWrite(DEVICE_FAN, HIGH);

    Logger::get_logger().debug("car shutdown");
}