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
    void set_led(int id, int val);
    void set_servo_angle(int id, float angle);
    void set_fan(int val);
    float get_distance();
    int get_infrared_sensor(int id);
    int get_light_sensor(int id);
    int get_track_sensor(int id);
    void go(float speed);
    void back(float speed);
    void stop();
    void turn_left(float speed);
    void turn_right(float speed);
    void spin_left(float speed);
    void spin_right(float speed);
    void shutdown();

    const int WHEEL_TRACK = 157;
    const int WHELL_RADIUS = 66;
    float SpeedDelta = 0.3;

    private:
    SmartCar();
    ~SmartCar();
    SmartCar(const SmartCar&);
	SmartCar& operator=(const SmartCar&);
};

#endif