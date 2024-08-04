#ifndef IPC_CAM_SUB_HPP
#define IPC_CAM_SUB_HPP

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
  void PrintProcessUsage(pid_t pid);
  pid_t pid_ = getpid();

};

#endif
