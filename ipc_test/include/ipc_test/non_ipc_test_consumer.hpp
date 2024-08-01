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
  explicit Consumer(const std::string &name, const std::string &input) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // Create a subscription on the input topic which prints on receipt of new messages
    this->sub_ = this->create_subscription<std_msgs::msg::Float64>(input, 10, 
      [](std_msgs::msg::Float64::UniquePtr msg){
        RCLCPP_WARN(rclcpp::get_logger("logger"),"Received message with value: %f and address: %p", msg->data, reinterpret_cast<std::uintptr_t>(msg.get()));
      }
    );
  } 
  ~Consumer(){};
 private:
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;

};

#endif