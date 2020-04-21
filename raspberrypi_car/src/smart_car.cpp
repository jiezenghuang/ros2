#include <iostream>
#include <thread>
#include <chrono>

#include "car_device.h"
#include "smart_car.h"

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
using namespace chrono;

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
    : is_init(false), speed(50), is_lighting(false), 
    us_servo_angle(DEFAULT_SERVO_ANGLE), ch_servo_angle(DEFAULT_SERVO_ANGLE), cv_servo_angle(DEFAULT_SERVO_ANGLE)
{}

SmartCar::~SmartCar()
{
    is_init = false;
}

SmartCar::SmartCar(const SmartCar&)
{}
	
SmartCar& SmartCar::operator=(const SmartCar&) 
{ 
    return SmartCar::Instance(); 
}

bool SmartCar::init()
{
    if(-1 == wiringPiSetup())
    {
        cerr << "smart car init fail" << endl;
        is_init = false;
        return false;
    }

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

    is_init = true;
    cout << "smart car init sucess, current car speed is " << speed << endl;

    return true;
}

void led_light(bool red, bool green, bool blue)
{
    red ? digitalWrite(DEVICE_LED_RED, HIGH) : digitalWrite(DEVICE_LED_RED, LOW);
    green ? digitalWrite(DEVICE_LED_GREEN, HIGH) : digitalWrite(DEVICE_LED_GREEN, LOW);
    blue ? digitalWrite(DEVICE_LED_BLUE, HIGH) : digitalWrite(DEVICE_LED_BLUE, LOW);
}

void SmartCar::color_led()
{
    while(is_lighting)
    {
        led_light(true, false, false);
        delay(1000);
        led_light(false, true, false);
        delay(1000);
        led_light(false, false, true);
        delay(1000);
        led_light(true, true, false);
        delay(1000);
        led_light(false, true, true);
        delay(1000);
        led_light(true, false, true);
        delay(1000);
    }
    cout << "color light set to off" << endl;
}

void SmartCar::set_led(int id, int val)
{
    if(id == DEVICE_LED_RED || id == DEVICE_LED_GREEN || id == DEVICE_LED_BLUE)
    {
        digitalWrite(id, val == 0 ? LOW : HIGH);
    }
}

void SmartCar::set_color_led(bool on)
{
    is_lighting = on;
    if(is_lighting)
    {
        cout << "color let set to on" << endl;
        thread light_thread(&SmartCar::color_led, this);
        light_thread.detach();
    }
}

void SmartCar::set_fan(int val)
{
    val == 0 ? digitalWrite(DEVICE_FAN, HIGH) : digitalWrite(DEVICE_FAN, LOW);
}

void SmartCar::set_servo_angle(int id, float angle)
{
    if(angle < 0 || angle > 180)
        return;
    
    if(id == DEVICE_US_H_SERVO || id == DEVICE_CAMERA_H_SERVO || id == DEVICE_CAMERA_V_SERVO)
    {
        int pluse = angle * (SERVO_PWM_H - SERVO_PWM_L) / 180  + SERVO_PWM_L;  
        softPwmWrite(id, pluse);      
        // int pluse_width = (1.0 + angle / 45) * 500;
        // digitalWrite(id, HIGH);
        // delayMicroseconds(pluse_width);
        // digitalWrite(id, LOW);
        // delayMicroseconds(20000 - pluse_width);

        switch (id)
        {
            case DEVICE_US_H_SERVO:
                us_servo_angle = angle;                
                break;
            case DEVICE_CAMERA_H_SERVO:
                ch_servo_angle = angle;
                break;
            case DEVICE_CAMERA_V_SERVO:
                cv_servo_angle = angle;
            default:
                break;
        }
    }
}

float SmartCar::get_servo_angle(int id)
{
    float angle = -1;
    switch (id)
    {
    case DEVICE_US_H_SERVO:
        angle = us_servo_angle;
        break;
    case DEVICE_CAMERA_H_SERVO:
        angle = ch_servo_angle;
        break;
    case DEVICE_CAMERA_V_SERVO:
        angle = cv_servo_angle;
        break;
    default:
        break;
    }
    return angle;
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
    while(digitalRead(DEVICE_US_ECHO) != HIGH)
    {
        end = system_clock::now();
        auto duration = duration_cast<microseconds>(end - start);
        if(duration.count() > 30000)
        {
            return -1;
        }

    }

    start = system_clock::now();
    while(digitalRead(DEVICE_US_ECHO) != LOW)
    {
        end = system_clock::now();
        auto duration = duration_cast<microseconds>(end - start);
        if(duration.count() > 30000)
        {
            return -1;
        }
    }

    end = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    return float(duration.count()) * 0.34 / 2;
}

int SmartCar::get_infrared_sensor(int id)
{
    if(id == SENSOR_LEFT_INFRARED || id == SENSOR_LEFT_INFRARED)
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

void SmartCar::go(float rate)
{
    if(is_init)
    {
        speed = MOTOR_PWM_RANGE * rate;
        digitalWrite(DEVICE_RIGHT_MOTOR_GO, HIGH);
        digitalWrite(DEVICE_RIGHT_MOTOR_BACK, LOW);
        softPwmWrite(DEVICE_RIGHT_MOTOR_PWM, speed);

        digitalWrite(DEVICE_LEFT_MOTOR_GO, HIGH);
        digitalWrite(DEVICE_LEFT_MOTOR_BACK, LOW);
        softPwmWrite(DEVICE_LEFT_MOTOR_PWM, speed);    
        delay(CONTROL_DELAY);
    }
}

void SmartCar::back(float rate)
{
    if(is_init)
    {
        speed = MOTOR_PWM_RANGE * rate;
        digitalWrite(DEVICE_RIGHT_MOTOR_GO, LOW);
        digitalWrite(DEVICE_RIGHT_MOTOR_BACK, HIGH);
        softPwmWrite(DEVICE_RIGHT_MOTOR_PWM, speed);

        digitalWrite(DEVICE_LEFT_MOTOR_GO, LOW);
        digitalWrite(DEVICE_LEFT_MOTOR_BACK, HIGH);
        softPwmWrite(DEVICE_LEFT_MOTOR_PWM, speed);
        delay(CONTROL_DELAY);
    }
}

void SmartCar::stop()
{
    if(is_init)
    {
        digitalWrite(DEVICE_RIGHT_MOTOR_GO, LOW);
        digitalWrite(DEVICE_RIGHT_MOTOR_BACK, LOW);
        softPwmWrite(right_motor_pwm, 0);

        digitalWrite(DEVICE_LEFT_MOTOR_GO, LOW);
        digitalWrite(DEVICE_LEFT_MOTOR_BACK, LOW);
        softPwmWrite(right_motor_pwm, 0);
        delay(CONTROL_DELAY);
    }
}

void SmartCar::turn_left()
{
    if(is_init)
    {
        digitalWrite(DEVICE_RIGHT_MOTOR_GO, HIGH);
        digitalWrite(DEVICE_RIGHT_MOTOR_BACK, LOW);
        softPwmWrite(DEVICE_RIGHT_MOTOR_PWM, speed / 2);

        digitalWrite(DEVICE_LEFT_MOTOR_GO, LOW);
        digitalWrite(DEVICE_LEFT_MOTOR_BACK, LOW);
        softPwmWrite(DEVICE_LEFT_MOTOR_PWM, 0);
        delay(CONTROL_DELAY);
    }
}

void SmartCar::turn_right()
{
    if(is_init)
    {
        digitalWrite(DEVICE_RIGHT_MOTOR_GO, LOW);
        digitalWrite(DEVICE_RIGHT_MOTOR_BACK, LOW);
        softPwmWrite(DEVICE_RIGHT_MOTOR_PWM, 0);

        digitalWrite(DEVICE_LEFT_MOTOR_GO, HIGH);
        digitalWrite(DEVICE_LEFT_MOTOR_BACK, LOW);
        softPwmWrite(DEVICE_LEFT_MOTOR_PWM, speed / 2);
        delay(CONTROL_DELAY);
    }
}

void SmartCar::spin_left()
{
    if(is_init)
    {
        digitalWrite(DEVICE_RIGHT_MOTOR_GO, HIGH);
        digitalWrite(DEVICE_RIGHT_MOTOR_BACK, LOW);
        softPwmWrite(DEVICE_RIGHT_MOTOR_PWM, speed / 2);

        digitalWrite(DEVICE_LEFT_MOTOR_GO, LOW);
        digitalWrite(DEVICE_LEFT_MOTOR_BACK, HIGH);
        softPwmWrite(DEVICE_LEFT_MOTOR_PWM, speed / 2);
        delay(CONTROL_DELAY);
    }
}

void SmartCar::spin_right()
{
    if(is_init)
    {
        digitalWrite(DEVICE_RIGHT_MOTOR_GO, LOW);
        digitalWrite(DEVICE_RIGHT_MOTOR_BACK, HIGH);
        softPwmWrite(DEVICE_RIGHT_MOTOR_PWM, speed / 2);

        digitalWrite(DEVICE_LEFT_MOTOR_GO, HIGH);
        digitalWrite(DEVICE_LEFT_MOTOR_BACK, LOW);
        softPwmWrite(DEVICE_LEFT_MOTOR_PWM, speed / 2);
        delay(CONTROL_DELAY);
    }
}

void SmartCar::shutdown()
{
    stop();

    softPwmWrite(DEVICE_US_H_SERVO, 0);
    softPwmWrite(DEVICE_CAMERA_H_SERVO, 0);
    softPwmWrite(DEVICE_CAMERA_V_SERVO, 0);

    digitalWrite(DEVICE_LED_RED, HIGH);
    digitalWrite(DEVICE_LED_GREEN, HIGH);
    digitalWrite(DEVICE_LED_BLUE, HIGH);

    digitalWrite(DEVICE_FAN, HIGH);
}