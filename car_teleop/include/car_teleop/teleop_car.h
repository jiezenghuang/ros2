#ifndef _TELEOP_CAR_H_
#define _TELEOP_CAR_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "car_interface/srv/command.hpp"

class TeleopCar : rclcpp::Node
{
    public:
    TeleopCar();

    private:    
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr us_sub_;
    void us_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const;

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr is_sub_;
    void is_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const;

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr ls_sub_;
    void ls_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const;

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr ts_sub_;
    void ts_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const;

    rclcpp::Client<car_interface::srv::Command>::SharedPtr cmd_cli_;

};

#endif