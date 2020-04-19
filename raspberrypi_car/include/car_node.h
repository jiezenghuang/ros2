#ifndef _CAR_NODE_H_
#define _CAR_NODE_H_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "car_interface/action/rotate_servo.hpp"
#include "car_interface/srv/command.hpp"

class CarNode : public rclcpp::Node
{
    public:
    CarNode();

    private:
    rclcpp::TimerBase::SharedPtr us_pub_timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr us_pub_;
    void us_pub_callback();

    rclcpp::TimerBase::SharedPtr is_pub_timer_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr is_pub_;
    void is_pub_callback();

    rclcpp::TimerBase::SharedPtr ls_pub_timer_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr ls_pub_;
    void ls_pub_callback();

    rclcpp::TimerBase::SharedPtr ts_pub_timer_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr ts_pub_;
    void ts_pub_callback();

    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
    void parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event) const;

    rclcpp::Service<car_interface::srv::Command>::SharedPtr cmd_srv_;
    void handle_command(const std::shared_ptr<car_interface::srv::Command::Request> request,
        std::shared_ptr<car_interface::srv::Command::Response> response);

    rclcpp_action::Server<car_interface::action::RotateServo>::SharedPtr servo_act_;
    rclcpp_action::GoalResponse act_handle_goal(const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const car_interface::action::RotateServo::Goal> goal);
    void act_execute_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<car_interface::action::RotateServo>> goal_handle);
    rclcpp_action::CancelResponse act_handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<car_interface::action::RotateServo>> goal_handle);
    void act_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<car_interface::action::RotateServo>> goal_handle);

};

#endif