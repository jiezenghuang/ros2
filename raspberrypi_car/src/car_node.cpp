#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "car_device.h"
#include "smart_car.h"
#include "car_node.h"
#include "car_interface/msg/command_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"

using namespace std::chrono_literals;

CarNode::CarNode()
        : Node("car")
{
    us_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("ultra_sound_sensor", 10);
    us_pub_timer_ = this->create_wall_timer(100ms, std::bind(&CarNode::us_pub_callback, this));

    is_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("infrared_sensor", 10);
    is_pub_timer_ = this->create_wall_timer(100ms, std::bind(&CarNode::is_pub_callback, this));

    ls_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("light_sensor", 10);
    ls_pub_timer_ = this->create_wall_timer(100ms, std::bind(&CarNode::ls_pub_callback, this));

    ts_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("track_sensor", 10);
    ts_pub_timer_ = this->create_wall_timer(100ms, std::bind(&CarNode::ts_pub_callback, this));


    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 0;
    range.to_value = 1;
    rcl_interfaces::msg::ParameterDescriptor fan_switch;
    fan_switch.description = "Switch of fan";
    fan_switch.integer_range.push_back(range);
    
    rcl_interfaces::msg::ParameterDescriptor led_red_switch;
    led_red_switch.description = "Switch of red led";
    led_red_switch.integer_range.push_back(range);

    rcl_interfaces::msg::ParameterDescriptor led_green_switch;
    led_green_switch.description = "Switch of green led";
    led_green_switch.integer_range.push_back(range);

    rcl_interfaces::msg::ParameterDescriptor led_blue_switch;
    led_blue_switch.description = "Switch of blue led";
    led_blue_switch.integer_range.push_back(range);

    this->declare_parameter("fan_switch", rclcpp::ParameterValue(0), fan_switch);
    this->declare_parameter("led_red_switch", rclcpp::ParameterValue(0), led_red_switch);
    this->declare_parameter("led_green_switch", rclcpp::ParameterValue(0), led_green_switch);
    this->declare_parameter("led_blue_switch", rclcpp::ParameterValue(0), led_blue_switch);

    rclcpp::QoS qos(rclcpp::KeepLast(100), rmw_qos_profile_sensor_data);
    parameter_event_sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events", qos, std::bind(&CarNode::parameter_event_callback, this, std::placeholders::_1));

    std::string node_name(this->get_name());
    cmd_srv_ = this->create_service<car_interface::srv::CommandArray>(node_name + "/cmd_srv", 
        std::bind(&CarNode::handle_command, this, std::placeholders::_1, std::placeholders::_2));

    // std::string node_name(this->get_name());
    // servo_act_ = rclcpp_action::create_server<car_interface::action::RotateServo>(
    //     this->get_node_base_interface(),
    //     this->get_node_clock_interface(),
    //     this->get_node_logging_interface(),
    //     this->get_node_waitables_interface(),
    //     node_name + "/action/rotate_servo",
    //     std::bind(&CarNode::act_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    //     std::bind(&CarNode::act_handle_cancel, this, std::placeholders::_1),
    //     std::bind(&CarNode::act_handle_accepted, this, std::placeholders::_1)
    // );
}

void CarNode::us_pub_callback()
{
    auto message = std_msgs::msg::Float32MultiArray();
    message.data.push_back(SmartCar::Instance().get_distance());

    us_pub_->publish(message);
}

void CarNode::is_pub_callback()
{
    auto message = std_msgs::msg::Int32MultiArray();
    message.data.push_back(SmartCar::Instance().get_infrared_sensor(SENSOR_LEFT_INFRARED));
    message.data.push_back(SmartCar::Instance().get_infrared_sensor(SENSOR_RIGHT_INFRARED));

    is_pub_->publish(message);
}

void CarNode::ls_pub_callback()
{
    auto message = std_msgs::msg::Int32MultiArray();
    message.data.push_back(SmartCar::Instance().get_infrared_sensor(SENSOR_LEFT_LIGHT));
    message.data.push_back(SmartCar::Instance().get_infrared_sensor(SENSOR_RIGHT_LIGHT));

    ls_pub_->publish(message);
}

void CarNode::ts_pub_callback()
{
    auto message = std_msgs::msg::Int32MultiArray();
    message.data.push_back(SmartCar::Instance().get_infrared_sensor(SENSOR_LEFT_TRACK_1));
    message.data.push_back(SmartCar::Instance().get_infrared_sensor(SENSOR_LEFT_TRACK_2));
    message.data.push_back(SmartCar::Instance().get_infrared_sensor(SENSOR_RIGHT_TRACK_1));
    message.data.push_back(SmartCar::Instance().get_infrared_sensor(SENSOR_RIGHT_TRACK_2));

    ts_pub_->publish(message);   
}

void CarNode::parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event) const
{
    if (event->node == this->get_fully_qualified_name())
    {
        for (auto & changed_parameter : event->changed_parameters)
        {
            if(changed_parameter.name == "fan_switch" 
                && changed_parameter.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
            {                
                SmartCar::Instance().set_fan(changed_parameter.value.integer_value);
            }

            if(changed_parameter.name == "led_red_switch"
                && changed_parameter.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
            {
                SmartCar::Instance().set_led(DEVICE_LED_RED, changed_parameter.value.integer_value);
            }

            if(changed_parameter.name == "led_green_switch"
                && changed_parameter.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
            {
                SmartCar::Instance().set_led(DEVICE_LED_GREEN, changed_parameter.value.integer_value);
            }

            if(changed_parameter.name == "led_blue_switch"
                && changed_parameter.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
            {
                SmartCar::Instance().set_led(DEVICE_LED_BLUE, changed_parameter.value.integer_value);
            }
        }
    }
}

void CarNode::handle_command(const std::shared_ptr<car_interface::srv::CommandArray::Request> request,
        std::shared_ptr<car_interface::srv::CommandArray::Response> response)
{    
    int result = 0;
    for(auto command : request->commands)
    {
        switch(command.type)
        {
            case car_interface::msg::CommandType::CAR_CMD_GO:
                    SmartCar::Instance().go(command.value);
                break;
            case car_interface::msg::CommandType::CAR_CMD_BACK:
                SmartCar::Instance().back(command.value);
                break;
            case car_interface::msg::CommandType::CAR_CMD_TURN_LEFT:
                SmartCar::Instance().turn_left(command.value);
                break;
            case car_interface::msg::CommandType::CAR_CMD_TURN_RIGHT:
                SmartCar::Instance().turn_right(command.value);
                break;
            case car_interface::msg::CommandType::CAR_CMD_SPIN_LEFT:
                SmartCar::Instance().spin_left(command.value);
                break;
            case car_interface::msg::CommandType::CAR_CMD_SPIN_RIGHT:
                SmartCar::Instance().spin_right(command.value);
                break;
            case car_interface::msg::CommandType::CAR_CMD_STOP:
                SmartCar::Instance().stop();
                break;
            case car_interface::msg::CommandType::SERVO_CMD_US_H:
                SmartCar::Instance().set_servo_angle(DEVICE_US_H_SERVO, command.value);
                break;
            case car_interface::msg::CommandType::SERVO_CMD_CAMERA_H:
                SmartCar::Instance().set_servo_angle(DEVICE_CAMERA_H_SERVO, command.value);
                break;
            case car_interface::msg::CommandType::SERVO_CMD_CAMERA_V:
                SmartCar::Instance().set_servo_angle(DEVICE_CAMERA_V_SERVO, command.value);
                break;
            default:
                result = -1;
                break;
        }
        response->results.push_back(result);
    }

}

// rclcpp_action::GoalResponse CarNode::act_handle_goal(const rclcpp_action::GoalUUID & uuid,
//         std::shared_ptr<const car_interface::action::RotateServo::Goal> goal)
// {
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received rotate servo request with id %d, angle %f", goal->servo, goal->thelta);
//     if(goal->servo == DEVICE_US_H_SERVO || goal->servo == DEVICE_CAMERA_H_SERVO || goal->servo == DEVICE_CAMERA_V_SERVO
//         && goal->thelta >= 0 && goal->thelta <= 180)
//         return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
//     else
//         return rclcpp_action::GoalResponse::REJECT;
// }

// void CarNode::act_execute_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<car_interface::action::RotateServo>> goal_handle)
// {
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing goal");
//     const auto goal = goal_handle->get_goal();
//     auto feedback = std::make_shared<car_interface::action::RotateServo::Feedback>();
//     float angle = SmartCar::Instance().get_servo_angle(goal->servo);    
//     float delta = 0;
//     //rclcpp::Rate loop_rate(30 * 1000 * 1000);
    
//     auto result = std::make_shared<car_interface::action::RotateServo::Result>();
//     while (abs(goal->thelta - angle - delta) > 0.1 && rclcpp::ok())
//     {
//         if(goal_handle->is_canceling())
//         {            
//             result->delta = abs(delta);
//             goal_handle->canceled(result);
//             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Canceled goal");
//             return;
//         }

//         SmartCar::Instance().set_servo_angle(goal->servo, angle + delta);        
//         if((angle + delta) < goal->thelta)
//             ++delta;
        
//         if((angle + delta)  > goal->thelta)
//             --delta;
        
//         feedback->remaining = abs(goal->thelta - angle - delta);
//         goal_handle->publish_feedback(feedback);
//         //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publish feedback");
//         //loop_rate.sleep();
//     }

//     if(rclcpp::ok())
//     {
//         result->delta = abs(delta);
//         goal_handle->succeed(result);
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execute goal success");
//     }
// }

// rclcpp_action::CancelResponse CarNode::act_handle_cancel(
//     const std::shared_ptr<rclcpp_action::ServerGoalHandle<car_interface::action::RotateServo>> goal_handle)
// {
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received request to cancel goal");
//     return rclcpp_action::CancelResponse::ACCEPT;
// }

// void CarNode::act_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<car_interface::action::RotateServo>> goal_handle)
// {
//     std::thread{std::bind(&CarNode::act_execute_goal, this, std::placeholders::_1), goal_handle}.detach();
// }