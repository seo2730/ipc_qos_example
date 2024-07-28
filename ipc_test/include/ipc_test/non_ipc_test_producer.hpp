#ifndef NON_IPC_TEST_PRODUCER_HPP
#define NON_IPC_TEST_PRODUCER_HPP

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

class Producer : public rclcpp::Node
{
 public:
  explicit Producer(const std::string &name, const std::string &output) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // Create a publisher on the output topic
    this->pub_ = this->create_publisher<std_msgs::msg::Float64>(output, 10);
    std::weak_ptr<std::remove_pointer<decltype(this->pub_.get())>::type> captured_pub = this->pub_;

    // Create a timer which publishes on the output topic at ~1Hz
    auto callback = [captured_pub]() -> void{
      auto pub_ptr = captured_pub.lock();
      if(!pub_ptr){
        return;
      }
      static double count = 0;
      std_msgs::msg::Float64::UniquePtr msg(new std_msgs::msg::Float64());
      msg->data = count++;
      RCLCPP_INFO(rclcpp::get_logger("logger"), "Published message with value  : %f, and address: %p", msg->data, reinterpret_cast<std::uintptr_t>(msg.get()));
      pub_ptr->publish(std::move(msg));
    };

    this->timer_ = this->create_wall_timer(1s, callback);
  }
  ~Producer(){};

 private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif
