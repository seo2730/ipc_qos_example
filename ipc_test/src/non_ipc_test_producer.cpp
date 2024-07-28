#include "ipc_test/non_ipc_test_producer.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Producer>("non_producer", "number");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
