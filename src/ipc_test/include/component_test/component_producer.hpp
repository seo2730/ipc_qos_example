#ifndef COMPONENT_PRODUCER_HPP
#define COMPONENT_PRODUCER_HPP

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

class Producer : public rclcpp::Node
{
 public:
  COMPOSITION_PUBLIC
  explicit Producer(const rclcpp::NodeOptions &options);
  ~Producer();

 private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}
#endif
