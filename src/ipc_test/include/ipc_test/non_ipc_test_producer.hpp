#ifndef NON_IPC_TEST_PRODUCER_HPP
#define NON_IPC_TEST_PRODUCER_HPP

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include <fstream>
#include <string>
#include <unistd.h>
#include <sstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class Producer : public rclcpp::Node
{
 public:
  explicit Producer(const std::string &name, const std::string &output);
  ~Producer();

 private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  pid_t pid_ = getpid();
};

#endif
