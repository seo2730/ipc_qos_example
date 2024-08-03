#include "component_test/component_producer.hpp"

namespace ipc_test
{

Producer::Producer(const rclcpp::NodeOptions &options) : Node("Component_Proceduer", rclcpp::NodeOptions().use_intra_process_comms(true)) // options
{
  // Create a publisher on the output topic
  this->pub_ = this->create_publisher<std_msgs::msg::Float64>("/number", 10);
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

Producer::~Producer()
{

}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(ipc_test::Producer)
