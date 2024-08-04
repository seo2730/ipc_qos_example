#include "ipc_cam/ipc_cam_sub.hpp"

WebcamSubscriber::WebcamSubscriber(const std::string &name, const std::string &input) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
  // Create a subscription on the input topic which prints on receipt of new messages
  this->subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(input, 10, 
    [=](sensor_msgs::msg::Image::UniquePtr msg){
      RCLCPP_WARN(rclcpp::get_logger("logger"),"Received message address: %p", reinterpret_cast<std::uintptr_t>(msg.get()));
    }
  );

  // this->subscriber1_ = this->create_subscription<sensor_msgs::msg::Image>("/test_image1", 10, 
  //   [](sensor_msgs::msg::Image::UniquePtr msg){
  //     RCLCPP_WARN(rclcpp::get_logger("logger"),"Received message address: %p", reinterpret_cast<std::uintptr_t>(msg.get()));
  //   }
  // );

  // this->subscriber2_ = this->create_subscription<sensor_msgs::msg::Image>("/test_image2", 10, 
  //   [](sensor_msgs::msg::Image::UniquePtr msg){
  //     RCLCPP_WARN(rclcpp::get_logger("logger"),"Received message address: %p", reinterpret_cast<std::uintptr_t>(msg.get()));
  //   }
  // );

  // this->subscriber3_ = this->create_subscription<sensor_msgs::msg::Image>("/test_image3", 10, 
  //   [](sensor_msgs::msg::Image::UniquePtr msg){
  //     RCLCPP_WARN(rclcpp::get_logger("logger"),"Received message address: %p", reinterpret_cast<std::uintptr_t>(msg.get()));
  //   }
  // );

  // this->subscriber4_ = this->create_subscription<sensor_msgs::msg::Image>("/test_image4", 10, 
  //   [](sensor_msgs::msg::Image::UniquePtr msg){
  //     RCLCPP_WARN(rclcpp::get_logger("logger"),"Received message address: %p", reinterpret_cast<std::uintptr_t>(msg.get()));
  //   }
  // );

  // this->subscriber5_ = this->create_subscription<sensor_msgs::msg::Image>("/test_image5", 10, 
  //   [](sensor_msgs::msg::Image::UniquePtr msg){
  //     RCLCPP_WARN(rclcpp::get_logger("logger"),"Received message address: %p", reinterpret_cast<std::uintptr_t>(msg.get()));
  //   }
  // );

  // this->subscriber6_ = this->create_subscription<sensor_msgs::msg::Image>("/test_image6", 10, 
  //   [](sensor_msgs::msg::Image::UniquePtr msg){
  //     RCLCPP_WARN(rclcpp::get_logger("logger"),"Received message address: %p", reinterpret_cast<std::uintptr_t>(msg.get()));
  //   }
  // );

  // this->subscriber7_ = this->create_subscription<sensor_msgs::msg::Image>("/test_image7", 10, 
  //   [](sensor_msgs::msg::Image::UniquePtr msg){
  //     RCLCPP_WARN(rclcpp::get_logger("logger"),"Received message address: %p", reinterpret_cast<std::uintptr_t>(msg.get()));
  //   }
  // );

  // this->subscriber8_ = this->create_subscription<sensor_msgs::msg::Image>("/test_image8", 10, 
  //   [](sensor_msgs::msg::Image::UniquePtr msg){
  //     RCLCPP_WARN(rclcpp::get_logger("logger"),"Received message address: %p", reinterpret_cast<std::uintptr_t>(msg.get()));
  //   }
  // );

  // this->subscriber9_ = this->create_subscription<sensor_msgs::msg::Image>("/test_image9", 10, 
  //   [](sensor_msgs::msg::Image::UniquePtr msg){
  //     RCLCPP_WARN(rclcpp::get_logger("logger"),"Received message address: %p", reinterpret_cast<std::uintptr_t>(msg.get()));
  //   }
  // );

  // this->subscriber10_ = this->create_subscription<sensor_msgs::msg::Image>("/test_image10", 10, 
  //   [](sensor_msgs::msg::Image::UniquePtr msg){
  //     RCLCPP_WARN(rclcpp::get_logger("logger"),"Received message address: %p", reinterpret_cast<std::uintptr_t>(msg.get()));
  //   }
  // );
}

WebcamSubscriber::~WebcamSubscriber()
{

}
