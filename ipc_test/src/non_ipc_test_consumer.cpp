#include "ipc_test/non_ipc_test_consumer.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Consumer>("non_consumer", "number");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
