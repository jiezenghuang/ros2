#ifndef _SMART_CAR_H_
#define _SMART_CAR_H_

class SmartCar
{
    public:
    static SmartCar& Instance()
    {
        static SmartCar car;
        return car;
    }

    bool init();
    void set_delay(unsigned int millis);
    void set_led(int id, int val);
    void set_color_led(bool on);
    void set_servo_angle(int id, float angle);
    float get_servo_angle(int id);
    void set_fan(int val);
    float get_distance();
    int get_infrared_sensor(int id);
    int get_light_sensor(int id);
    int get_track_sensor(int id);
    void go(float rate);
    void back(float rate);
    void stop();
    void turn_left();
    void turn_right();
    void spin_left();
    void spin_right();
    void shutdown();

    private:
    SmartCar();
    ~SmartCar();
    SmartCar(const SmartCar&);
	SmartCar& operator=(const SmartCar&);

    bool is_init;
    int speed;
    bool is_lighting;
    float us_servo_angle;
    float ch_servo_angle;
    float cv_servo_angle;

    void color_led();
    void servo_control(int id, float* angle);
};

#endif