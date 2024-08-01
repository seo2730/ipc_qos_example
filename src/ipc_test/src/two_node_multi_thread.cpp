#include "ipc_test/ipc_test_producer.hpp"
#include "ipc_test/ipc_test_consumer.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto producer = std::make_shared<Producer>("ipc_producer", "number");
  auto consumer = std::make_shared<Consumer>("ipc_consumer", "number");

  executor.add_node(producer);
  executor.add_node(consumer);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
