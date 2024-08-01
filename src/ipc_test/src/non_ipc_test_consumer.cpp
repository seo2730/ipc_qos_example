#include "ipc_test/non_ipc_test_consumer.hpp"

Consumer::Consumer(const std::string &name, const std::string &input) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
  // Create a subscription on the input topic which prints on receipt of new messages
  this->sub_ = this->create_subscription<std_msgs::msg::Float64>(input, 10, 
    [](std_msgs::msg::Float64::UniquePtr msg){
      RCLCPP_WARN(rclcpp::get_logger("logger"),"Received message with value: %f and address: %p", msg->data, reinterpret_cast<std::uintptr_t>(msg.get()));
    }
  );
} 

Consumer::~Consumer()
{
    
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Consumer>("ipc_separate_node_consumer", "number");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
