#include "teleop_car.h"
#include <chrono>
#include "car_interface/msg/command_type.hpp"

using namespace std::chrono_literals;

TeleopCar::TeleopCar()
    : Node("teleop")
{
    us_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("ultra_sound_sensor", 10, 
        std::bind(&TeleopCar::us_callback, this, std::placeholders::_1));
    is_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("infrared_sensor", 10,
        std::bind(&TeleopCar::is_callback, this, std::placeholders::_1));
    ls_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("light_sensor", 10,
        std::bind(&TeleopCar::ls_callback, this, std::placeholders::_1));
    ts_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("track_sensor", 10,
        std::bind(&TeleopCar::ts_callback, this, std::placeholders::_1));
    cmd_cli_ = this->create_client<car_interface::srv::Command>("cmd_srv");
}


void TeleopCar::us_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
{
    if(msg->data.size() != 1)
        return;

    auto request = std::make_shared<car_interface::srv::Command::Request>();
    if(msg->data[0] > 0)
    {
        if(msg->data[0] < 1000)
        {
            request->type = car_interface::msg::CommandType::CAR_CMD_STOP;
            request->value = 0;
        }
        else if(msg->data[0] > 1000 && msg->data[0] < 2000)
        {
            request->type = car_interface::msg::CommandType::CAR_CMD_TURN_RIGHT;
            request->value = 0.2;
        }
    }
    else
    {
        request->type = car_interface::msg::CommandType::CAR_CMD_GO;
        request->value = 0.5;
    }
        

    if(cmd_cli_->service_is_ready())
        auto result = cmd_cli_->async_send_request(request);
}

void TeleopCar::is_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const
{
    if(msg->data.size() != 2)
        return;
    
    auto request = std::make_shared<car_interface::srv::Command::Request>();
    if(msg->data[0] == 1 && msg->data[1] == 0)
    {
        request->type = car_interface::msg::CommandType::CAR_CMD_TURN_LEFT;
        request->value = 0.2;       
    }
    else if(msg->data[0] == 0 && msg->data[1] == 1)
    {
        request->type = car_interface::msg::CommandType::CAR_CMD_TURN_RIGHT;
        request->value = 0.2; 
    }

}

void TeleopCar::ls_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const
{

}

void TeleopCar::ts_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const
{

}