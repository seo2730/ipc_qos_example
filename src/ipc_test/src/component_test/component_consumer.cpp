#include "component_test/component_consumer.hpp"

namespace ipc_test
{

Consumer::Consumer(const rclcpp::NodeOptions &options) : Node("Component_Consumer", rclcpp::NodeOptions().use_intra_process_comms(true))// options
{
  // Create a subscription on the input topic which prints on receipt of new messages
  this->sub_ = this->create_subscription<std_msgs::msg::Float64>("/number", 10, 
    [](std_msgs::msg::Float64::UniquePtr msg){
      RCLCPP_WARN(rclcpp::get_logger("logger"),"Received message with value: %f and address: %p", msg->data, reinterpret_cast<std::uintptr_t>(msg.get()));
    }
  );
} 
Consumer::~Consumer()
{
  
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(ipc_test::Consumer)
