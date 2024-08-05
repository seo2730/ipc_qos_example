#ifndef COMPONENT_CAM_PUB_HPP
#define COMPONENT_CAM_SUB_HPP

#include <fstream>
#include <string>
#include <unistd.h>
#include <sstream>
#include <vector>

#include "component_cam/cam_component_visualibility.h"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

namespace component_cam
{

class WebcamPublisher : public rclcpp::Node
{
 public:
    COMPOSITION_PUBLIC
    explicit WebcamPublisher(const rclcpp::NodeOptions &options);
    ~WebcamPublisher();

 private:
    void timerCallback();
    void PrintProcessUsage(pid_t pid);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;

    pid_t pid_ = getpid();
};

}
#endif
