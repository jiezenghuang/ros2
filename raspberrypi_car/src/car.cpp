#include "rclcpp/rclcpp.hpp"

#include "smart_car.h"
#include "car_node.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  if(!SmartCar::Instance().init())
  {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "car init fail, exit...");
      return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "car start up finish");

  rclcpp::spin(std::make_shared<CarNode>());
  SmartCar::Instance().stop();
  rclcpp::shutdown();
  return 0;
}
