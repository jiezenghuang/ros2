#include <chrono>
#include <functional>
#include <sstream>
#include <string>
#include <opencv2/highgui/highgui.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node
{
  public:
    CameraPublisher(cv::VideoCapture& cap)
    : Node("camera"), capture(cap)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
      timer_ = this->create_wall_timer(
      30ms, std::bind(&CameraPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      capture >> frame;
      if(!frame.empty())
      {
          auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
          publisher_->publish(*message);
      }
    }

    cv::Mat frame;
    cv::VideoCapture& capture;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  };

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  if(argc != 2)
  {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "camera device id miss");
      rclcpp::shutdown();
      return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "start publish camera[%s]", argv[1]);
  std::istringstream ss(argv[1]);
  int video_source;
  if(!(ss >> video_source))
  {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "camera device id must be an number");
      rclcpp::shutdown();
      return -1;     
  }
  cv::VideoCapture cap(video_source);
  if(!cap.isOpened())
  {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "open camera[%d] fail", video_source);
      rclcpp::shutdown();
      return -1;         
  }
  
  rclcpp::spin(std::make_shared<CameraPublisher>(cap));
  rclcpp::shutdown();
  return 0;
}