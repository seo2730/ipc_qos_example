#ifndef NON_IPC_CAM_SUB_HPP
#define NON_IPC_CAM_SUB_HPP

// Case 0 : Shared_ptr & ipc false
// Case 1 : Shared_ptr & ipc true
// Case 2 : Unique_ptr & ipc false
// Case 3 : Unique_ptr & ipc true

#include <fstream>
#include <string>
#include <unistd.h>
#include <sstream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class WebcamSubscriber : public rclcpp::Node
{
 public:
  explicit WebcamSubscriber(const std::string &name, const std::string &input);
  ~WebcamSubscriber();

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber1_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber2_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber3_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber4_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber5_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber6_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber7_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber8_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber9_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber10_;

};

#endif
