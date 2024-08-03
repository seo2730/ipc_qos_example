#ifndef IPC_TEST_CAM_HPP
#define IPC_TEST_CAM_HPP

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

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;

    double max_memory_;
    double average_memory_;
    double min_memory_;

    // double count_=0;
};

class WebcamSubscriber : public rclcpp::Node
{
 public:
  explicit WebcamSubscriber(const std::string &name, const std::string &input);
  ~WebcamSubscriber();

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
};

#endif
