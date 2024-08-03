#ifndef COMPONENT_CONSUMER_HPP
#define COMPONENT_CONSUMER_HPP

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "component_test/component_visualibility.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

namespace ipc_test
{

class Consumer : public rclcpp::Node
{
 public:
  COMPOSITION_PUBLIC
  explicit Consumer(const rclcpp::NodeOptions &options);
  ~Consumer();
 private:
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;

};

}

#endif
