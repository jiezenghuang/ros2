#ifndef _TELEOP_CAR_H_
#define _TELEOP_CAR_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "car_interface/srv/command.hpp"
#include "state_machine.hpp"

class CarStatus
{
    public:
    int left_track_sensor_1;
    int left_track_sensor_2;
    int right_track_sensor_1;
    int right_track_sensor_2;
    int left_light_sensor;
    int right_light_sensor;
    int left_infrared_sensor;
    int right_infrared_sensor;
    float speed;
    float distance;
};

class TeleopCar : public rclcpp::Node
{
    public:
    TeleopCar();
    ~TeleopCar() { machine_.stop();}

    void start();
    rclcpp::Client<car_interface::srv::Command>::SharedPtr teleop_client;

    private:        
    StateMachine machine_;
    CarStatus status_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr us_sub_;
    void us_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr is_sub_;
    void is_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr ls_sub_;
    void ls_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr ts_sub_;
    void ts_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

    void async_send_request(std::shared_ptr<car_interface::srv::Command::Request> request);
    void handle_service_response(const rclcpp::Client<car_interface::srv::Command>::SharedFuture future);

    std::shared_ptr<Alphabet> process_stop(const std::shared_ptr<Alphabet> alphabet);
    std::shared_ptr<Alphabet> process_go(const std::shared_ptr<Alphabet> alphabet);
    std::shared_ptr<Alphabet> process_back(const std::shared_ptr<Alphabet> alphabet);
    std::shared_ptr<Alphabet> process_turn_left(const std::shared_ptr<Alphabet> alphabet);
    std::shared_ptr<Alphabet> process_turn_right(const std::shared_ptr<Alphabet> alphabet);
};

#endif