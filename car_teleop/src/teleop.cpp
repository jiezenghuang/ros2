#include "rclcpp/rclcpp.hpp"
#include "teleop_car.h"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopCar>());
    rclcpp::shutdown();
    return 0;
}