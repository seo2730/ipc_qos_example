#ifndef IPC_CAM_PUB_HPP
#define IPC_CAM_PUB_HPP

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

class WebcamPublisher : public rclcpp::Node
{
public:
    explicit WebcamPublisher(const std::string &name, const std::string &output);
    ~WebcamPublisher();

private:
    void timerCallback();
    void PrintProcessUsage(pid_t pid);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;

    pid_t pid_ = getpid();
};

#endif
