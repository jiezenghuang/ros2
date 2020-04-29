#include "teleop_car.h"
#include <chrono>
#include "car_interface/msg/command_type.hpp"

using namespace std::chrono_literals;

enum CarState
{
    CAR_STATE_STOP = 100,
    CAR_STATE_GO,
    CAR_STATE_BACK,
    CAR_STATE_TURN_LEFT,
    CAR_STATE_TURN_RIGHT,
    CAR_STATE_SPIN_LEFT,
    CAR_STATE_SPIN_RIGHT
};

enum CarAlphabet
{
    CAR_ALPHABET_FAIL = 400,
    CAR_ALPHABET_OK,
    CAR_ALPHABET_OBSTACLE_LEFT,
    CAR_ALPHABET_OBSTACLE_RIGHT,
    CAR_ALPHABET_OBSTACLE_BOTH,
};

const int OBSTACLE_DETECT = 0;
const int OBSTACLE_CLEAR = 1;

const float SPEED_MAX = 0.4;
const float SPEED_MIN = 0.2;
const float DISTANCE_SATE = 200;

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
    teleop_client = this->create_client<car_interface::srv::CommandArray>("car/car_srv");
}

void TeleopCar::run()
{
    machine_.add_state(std::make_shared<State>(CAR_STATE_STOP, std::bind(&TeleopCar::process_stop, this, std::placeholders::_1)));
    machine_.add_state_transition(CAR_STATE_STOP, CAR_ALPHABET_FAIL, CAR_STATE_STOP);
    machine_.add_state_transition(CAR_STATE_STOP, CAR_ALPHABET_OK, CAR_STATE_GO);

    machine_.add_state(std::make_shared<State>(CAR_STATE_GO, std::bind(&TeleopCar::process_go, this, std::placeholders::_1)));
    machine_.add_state_transition(CAR_STATE_GO, CAR_ALPHABET_OBSTACLE_LEFT, CAR_STATE_TURN_RIGHT);
    machine_.add_state_transition(CAR_STATE_GO, CAR_ALPHABET_OBSTACLE_RIGHT, CAR_STATE_TURN_LEFT);
    machine_.add_state_transition(CAR_STATE_GO, CAR_ALPHABET_OBSTACLE_BOTH, CAR_STATE_BACK);
    machine_.add_state_transition(CAR_STATE_GO, CAR_ALPHABET_FAIL, CAR_STATE_STOP);

    machine_.add_state(std::make_shared<State>(CAR_STATE_BACK, std::bind(&TeleopCar::process_back, this, std::placeholders::_1)));
    machine_.add_state_transition(CAR_STATE_BACK, CAR_ALPHABET_OBSTACLE_LEFT, CAR_STATE_TURN_RIGHT);
    machine_.add_state_transition(CAR_STATE_BACK, CAR_ALPHABET_OBSTACLE_RIGHT, CAR_STATE_TURN_LEFT);
    //machine_.add_state_transition(CAR_STATE_BACK, CAR_ALPHABET_OK, CAR_STATE_SPIN_RIGHT);
    machine_.add_state_transition(CAR_STATE_BACK, CAR_ALPHABET_FAIL, CAR_STATE_STOP);

    //machine_.add_state(std::make_shared<State>(CAR_STATE_SPIN_LEFT, std::bind(&TeleopCar::process_spin_left, this, std::placeholders::_1)));
    //machine_.add_state_transition(CAR_STATE_SPIN_LEFT, CAR_ALPHABET_OK, CAR_STATE_GO);
    //machine_.add_state_transition(CAR_STATE_SPIN_LEFT, CAR_ALPHABET_FAIL, CAR_STATE_STOP);

    //machine_.add_state(std::make_shared<State>(CAR_STATE_SPIN_RIGHT, std::bind(&TeleopCar::process_spin_right, this, std::placeholders::_1)));
    //machine_.add_state_transition(CAR_STATE_SPIN_RIGHT, CAR_ALPHABET_OK, CAR_STATE_GO);
    //machine_.add_state_transition(CAR_STATE_SPIN_RIGHT, CAR_ALPHABET_FAIL, CAR_STATE_STOP);

    machine_.add_state(std::make_shared<State>(CAR_STATE_TURN_LEFT, std::bind(&TeleopCar::process_turn_left, this, std::placeholders::_1)));
    machine_.add_state_transition(CAR_STATE_TURN_LEFT, CAR_ALPHABET_OBSTACLE_LEFT, CAR_STATE_TURN_RIGHT);
    //machine_.add_state_transition(CAR_STATE_TURN_LEFT, CAR_ALPHABET_OBSTACLE_RIGHT, CAR_STATE_TURN_LEFT);
    machine_.add_state_transition(CAR_STATE_TURN_LEFT, CAR_ALPHABET_OK, CAR_STATE_GO);
    machine_.add_state_transition(CAR_STATE_TURN_LEFT, CAR_ALPHABET_FAIL, CAR_STATE_STOP);

    machine_.add_state(std::make_shared<State>(CAR_STATE_TURN_RIGHT, std::bind(&TeleopCar::process_turn_right, this, std::placeholders::_1)));
    //machine_.add_state_transition(CAR_STATE_TURN_RIGHT, CAR_ALPHABET_OBSTACLE_LEFT, CAR_STATE_TURN_RIGHT);
    machine_.add_state_transition(CAR_STATE_TURN_RIGHT, CAR_ALPHABET_OBSTACLE_RIGHT, CAR_STATE_TURN_LEFT);
    machine_.add_state_transition(CAR_STATE_TURN_RIGHT, CAR_ALPHABET_OK, CAR_STATE_GO);
    machine_.add_state_transition(CAR_STATE_TURN_RIGHT, CAR_ALPHABET_FAIL, CAR_STATE_STOP);

    machine_.start(CAR_STATE_STOP);
}

void TeleopCar::us_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if(msg->data.size() != 1)
        return;

    if(abs(status_.distance - msg->data[0]) > 0.1)
        status_.distance = msg->data[0];
}

void TeleopCar::is_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    if(msg->data.size() != 2)
        return;
    
    if(status_.left_infrared_sensor != msg->data[0])
        status_.left_infrared_sensor = msg->data[0];
    
    if(status_.left_infrared_sensor != msg->data[1])
        status_.right_infrared_sensor = msg->data[1];
}

void TeleopCar::ls_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) 
{
    if(msg->data.size() != 2)
        return;
    
    if(status_.left_light_sensor != msg->data[0])
        status_.left_light_sensor = msg->data[0];

    if(status_.right_light_sensor != msg->data[1])
        status_.right_light_sensor = msg->data[1];
}

void TeleopCar::ts_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    if(msg->data.size() != 4)
        return;

    if(status_.left_track_sensor_1 != msg->data[0])
        status_.left_track_sensor_1 = msg->data[0];

    if(status_.left_track_sensor_2 != msg->data[1])
        status_.left_track_sensor_2 = msg->data[1];

    if(status_.right_track_sensor_1 != msg->data[2])
        status_.right_track_sensor_1 = msg->data[2];

    if(status_.right_track_sensor_2 != msg->data[3])
        status_.right_track_sensor_2 = msg->data[3];
}

void TeleopCar::async_send_request(std::shared_ptr<car_interface::srv::CommandArray::Request> request)
{
    if(teleop_client->service_is_ready())
    {
       auto result = teleop_client->async_send_request(request, std::bind(&TeleopCar::handle_service_response, this, std::placeholders::_1));
       rclcpp::sleep_for(20ms);
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service %s not available", teleop_client->get_service_name());
    }
}

void TeleopCar::handle_service_response(const rclcpp::Client<car_interface::srv::CommandArray>::SharedFuture future)
{
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "response from service: %d", future.get()->result);
}

std::shared_ptr<Alphabet> TeleopCar::process_stop(std::shared_ptr<Alphabet> alphabet)
{
    if(alphabet == nullptr)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[stop]car init, distance %f", status_.distance);
        auto request = std::make_shared<car_interface::srv::CommandArray::Request>();
        auto command = car_interface::msg::Command();
        command.type = car_interface::msg::CommandType::SERVO_CMD_US_H;
        command.value = 90;
        request->commands.push_back(command);        
        async_send_request(request);
        return std::make_shared<Alphabet>(CAR_ALPHABET_OK);
    }        
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[stop]car stop, distance %f", status_.distance);
        auto request = std::make_shared<car_interface::srv::CommandArray::Request>();
        auto command = car_interface::msg::Command();
        command.type = car_interface::msg::CommandType::CAR_CMD_STOP;
        command.value = 0;
        request->commands.push_back(command);    
        async_send_request(request);
        machine_.stop();
        return nullptr;
    }
}

std::shared_ptr<Alphabet> TeleopCar::process_go(std::shared_ptr<Alphabet> alphabet)
{
    if(alphabet == nullptr)
        return std::make_shared<Alphabet>(CAR_ALPHABET_FAIL);
    else
    {
        auto request = std::make_shared<car_interface::srv::CommandArray::Request>();
        auto command = car_interface::msg::Command();
        command.type = car_interface::msg::CommandType::CAR_CMD_GO;
        command.value = SPEED_MAX;
        request->commands.push_back(command);    
        async_send_request(request);
    }

    do
    {
        rclcpp::Rate(100ms).sleep();

        if(status_.left_infrared_sensor == OBSTACLE_DETECT && status_.right_infrared_sensor == OBSTACLE_DETECT)
        {
            alphabet->key = CAR_ALPHABET_OBSTACLE_BOTH;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[go]obstacle detected on both side, distance %f", status_.distance);
            break;
        }
        else if(status_.left_infrared_sensor == OBSTACLE_DETECT && status_.right_infrared_sensor != OBSTACLE_DETECT)
        {
            alphabet->key = CAR_ALPHABET_OBSTACLE_LEFT;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[go]obstacle detected on left side, distance %f", status_.distance);
            break;
        }
        else if(status_.left_infrared_sensor != OBSTACLE_DETECT && status_.right_infrared_sensor == OBSTACLE_DETECT)
        {
            alphabet->key = CAR_ALPHABET_OBSTACLE_RIGHT;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[go]obstacle detected on right side, distance %f", status_.distance);
            break;
        }
        else
        {
            if(status_.distance < DISTANCE_SATE)
            {
                alphabet->key = CAR_ALPHABET_OBSTACLE_LEFT;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[go]obstacle too near, distance %f", status_.distance);
                break;
            }
            else
            {
                continue;
            }
        }
    }while(true);

    return alphabet;
}

std::shared_ptr<Alphabet> TeleopCar::process_back(std::shared_ptr<Alphabet> alphabet)
{
    if(alphabet == nullptr)
        return std::make_shared<Alphabet>(CAR_ALPHABET_FAIL);
    else
    {
        auto request = std::make_shared<car_interface::srv::CommandArray::Request>();
        auto command = car_interface::msg::Command();
        command.type = car_interface::msg::CommandType::CAR_CMD_BACK;
        command.value = SPEED_MIN;
        request->commands.push_back(command);  
        async_send_request(request);
    }

    do
    {  
        rclcpp::Rate(100ms).sleep();

        if(status_.left_infrared_sensor == OBSTACLE_DETECT && status_.right_infrared_sensor == OBSTACLE_DETECT)
        {
            alphabet->key = CAR_ALPHABET_OBSTACLE_BOTH;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[back]obstacle detected on both side, distance %f", status_.distance);
            continue;
        }
        else if(status_.left_infrared_sensor == OBSTACLE_DETECT && status_.right_infrared_sensor != OBSTACLE_DETECT)
        {
            alphabet->key = CAR_ALPHABET_OBSTACLE_LEFT;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[back]obstacle detected on left side, distance %f", status_.distance);
            break;
        }
        else if(status_.left_infrared_sensor != OBSTACLE_DETECT && status_.right_infrared_sensor == OBSTACLE_DETECT)
        {
            alphabet->key = CAR_ALPHABET_OBSTACLE_RIGHT;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[back]obstacle detected on right side, distance %f", status_.distance);
            break;
        }
        else
        {
            alphabet->key = CAR_ALPHABET_OBSTACLE_LEFT;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[back]obstacle clear, distance %f", status_.distance);
            break;
        }
    }while(true);

    return alphabet;
}

std::shared_ptr<Alphabet> TeleopCar::process_turn_left(std::shared_ptr<Alphabet> alphabet)
{
    if(alphabet == nullptr)
        return std::make_shared<Alphabet>(CAR_ALPHABET_FAIL);
    else
    {
        auto request = std::make_shared<car_interface::srv::CommandArray::Request>();
        auto command = car_interface::msg::Command();
        command.type = car_interface::msg::CommandType::CAR_CMD_TURN_LEFT;
        command.value = SPEED_MIN;
        request->commands.push_back(command);  
        async_send_request(request);
    }

    do
    {
        rclcpp::Rate(1s).sleep();

        if(status_.left_infrared_sensor == OBSTACLE_DETECT && status_.right_infrared_sensor == OBSTACLE_DETECT)
        {
            alphabet->key = CAR_ALPHABET_OBSTACLE_BOTH;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[turn left]obstacle detected on both side, distance %f", status_.distance);
            continue;
        }
        else if(status_.left_infrared_sensor == OBSTACLE_DETECT && status_.right_infrared_sensor != OBSTACLE_DETECT)
        {
            alphabet->key = CAR_ALPHABET_OBSTACLE_LEFT;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[turn left]obstacle detected on left side, distance %f", status_.distance);
            break;
        }
        else if(status_.left_infrared_sensor != OBSTACLE_DETECT && status_.right_infrared_sensor == OBSTACLE_DETECT)
        {
            alphabet->key = CAR_ALPHABET_OBSTACLE_LEFT;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[turn left]obstacle detected on right side, distance %f", status_.distance);
            continue;
        }
        else
        {
            alphabet->key = CAR_ALPHABET_OK;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[turn left]obstacle clear, distance %f", status_.distance);
            break;
        }
    }while(true);

    return alphabet;
}

std::shared_ptr<Alphabet> TeleopCar::process_turn_right(std::shared_ptr<Alphabet> alphabet)
{
    if(alphabet == nullptr)
        return std::make_shared<Alphabet>(CAR_ALPHABET_FAIL);
    else
    {
        auto request = std::make_shared<car_interface::srv::CommandArray::Request>();
        auto command = car_interface::msg::Command();
        command.type = car_interface::msg::CommandType::CAR_CMD_TURN_RIGHT;
        command.value = SPEED_MIN;
        request->commands.push_back(command);  
        async_send_request(request);
    }

    do
    {
        rclcpp::Rate(1s).sleep();

        if(status_.left_infrared_sensor == OBSTACLE_DETECT && status_.right_infrared_sensor == OBSTACLE_DETECT)
        {
            alphabet->key = CAR_ALPHABET_OBSTACLE_BOTH;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[turn right]obstacle detected on both side, distance %f", status_.distance);
            continue;
        }
        else if(status_.left_infrared_sensor == OBSTACLE_DETECT && status_.right_infrared_sensor != OBSTACLE_DETECT)
        {
            alphabet->key = CAR_ALPHABET_OBSTACLE_LEFT;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[turn right]obstacle detected on left side, distance %f", status_.distance);
            continue;
        }
        else if(status_.left_infrared_sensor != OBSTACLE_DETECT && status_.right_infrared_sensor == OBSTACLE_DETECT)
        {
            alphabet->key = CAR_ALPHABET_OBSTACLE_LEFT;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[turn right]obstacle detected on right side, distance %f", status_.distance);
            break;
        }
        else
        {
            alphabet->key = CAR_ALPHABET_OK;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[turn right]obstacle clear, distance %f", status_.distance);
            break;
        }
    }while(true);

    return alphabet;
}

std::shared_ptr<Alphabet> TeleopCar::process_spin_left(std::shared_ptr<Alphabet> alphabet)
{
    if(alphabet == nullptr)
        return std::make_shared<Alphabet>(CAR_ALPHABET_FAIL);
    else
    {
        auto request = std::make_shared<car_interface::srv::CommandArray::Request>();
        auto command = car_interface::msg::Command();
        command.type = car_interface::msg::CommandType::CAR_CMD_SPIN_LEFT;
        command.value = SPEED_MIN;
        request->commands.push_back(command); 
        async_send_request(request);
    }

    do
    {
        rclcpp::Rate(1s).sleep();

        if(status_.left_infrared_sensor == OBSTACLE_DETECT && status_.right_infrared_sensor == OBSTACLE_DETECT)
        {
            alphabet->key = CAR_ALPHABET_OBSTACLE_BOTH;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[spin left]obstacle detected on both side, distance %f", status_.distance);
            continue;
        }
        else if(status_.left_infrared_sensor == OBSTACLE_DETECT && status_.right_infrared_sensor != OBSTACLE_DETECT)
        {
            alphabet->key = CAR_ALPHABET_OBSTACLE_LEFT;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[spin left]obstacle detected on left side, distance %f", status_.distance);
            break;
        }
        else if(status_.left_infrared_sensor != OBSTACLE_DETECT && status_.right_infrared_sensor == OBSTACLE_DETECT)
        {
            alphabet->key = CAR_ALPHABET_OBSTACLE_LEFT;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[spin left]obstacle detected on right side, distance %f", status_.distance);
            continue;
        }
        else
        {
            alphabet->key = CAR_ALPHABET_OK;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[spin left]obstacle clear, distance %f", status_.distance);
            break;
        }
    }while(true);

    return alphabet;
}

std::shared_ptr<Alphabet> TeleopCar::process_spin_right(std::shared_ptr<Alphabet> alphabet)
{
    if(alphabet == nullptr)
        return std::make_shared<Alphabet>(CAR_ALPHABET_FAIL);
    else
    {
        auto request = std::make_shared<car_interface::srv::CommandArray::Request>();
        auto command = car_interface::msg::Command();
        command.type = car_interface::msg::CommandType::CAR_CMD_SPIN_RIGHT;
        command.value = SPEED_MIN;
        request->commands.push_back(command); 
        async_send_request(request);
    }

    do
    {
        rclcpp::Rate(1s).sleep();

        if(status_.left_infrared_sensor == OBSTACLE_DETECT && status_.right_infrared_sensor == OBSTACLE_DETECT)
        {
            alphabet->key = CAR_ALPHABET_OBSTACLE_BOTH;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[spin left]obstacle detected on both side, distance %f", status_.distance);
            continue;
        }
        else if(status_.left_infrared_sensor == OBSTACLE_DETECT && status_.right_infrared_sensor != OBSTACLE_DETECT)
        {
            alphabet->key = CAR_ALPHABET_OBSTACLE_LEFT;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[spin left]obstacle detected on left side, distance %f", status_.distance);
            continue;
        }
        else if(status_.left_infrared_sensor != OBSTACLE_DETECT && status_.right_infrared_sensor == OBSTACLE_DETECT)
        {
            alphabet->key = CAR_ALPHABET_OBSTACLE_LEFT;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[spin left]obstacle detected on right side, distance %f", status_.distance);
            break;
        }
        else
        {
            alphabet->key = CAR_ALPHABET_OK;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[spin left]obstacle clear, distance %f", status_.distance);
            break;
        }
    }while(true);

    return alphabet;
}