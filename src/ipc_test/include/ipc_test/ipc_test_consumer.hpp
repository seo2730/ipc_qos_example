#ifndef NON_IPC_TEST_CONSUMER_HPP
#define NON_IPC_TEST_CONSUMER_HPP

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class Consumer : public rclcpp::Node
{
 public:
  explicit Consumer(const std::string &name, const std::string &input);
  ~Consumer();
 private:
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;

};

#endif
