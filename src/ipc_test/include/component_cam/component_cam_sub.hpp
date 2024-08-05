#ifndef COMPONENT_CAM_SUB_HPP
#define COMPONENT_CAM_SUB_HPP

// Case 0 : Shared_ptr & ipc false
// Case 1 : Shared_ptr & ipc true
// Case 2 : Unique_ptr & ipc false
// Case 3 : Unique_ptr & ipc true

#include <fstream>
#include <string>
#include <unistd.h>
#include <sstream>
#include <vector>

#include "component_cam/cam_component_visualibility.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace component_cam
{

class WebcamSubscriber : public rclcpp::Node
{
 public:
  COMPOSITION_PUBLIC
  explicit WebcamSubscriber(const rclcpp::NodeOptions &options);
  ~WebcamSubscriber();

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
  void PrintProcessUsage(pid_t pid);
  pid_t pid_ = getpid();

};

}
#endif
