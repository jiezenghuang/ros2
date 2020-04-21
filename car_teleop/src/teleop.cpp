#include "rclcpp/rclcpp.hpp"
#include "teleop_car.h"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopCar>();

    while(!node->teleop_client->wait_for_service(1s))
    {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service %s not available, try next time again...", node->teleop_client->get_service_name());
    }

    node->run();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}