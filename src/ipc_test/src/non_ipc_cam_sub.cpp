#include "ipc_test/non_ipc_cam_sub.hpp"

WebcamSubscriber::WebcamSubscriber(const std::string &name, const std::string &input) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(false))
{
  // Create a subscription on the input topic which prints on receipt of new messages
  this->subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(input, 10, 
    [](sensor_msgs::msg::Image::UniquePtr msg){
      RCLCPP_WARN(rclcpp::get_logger("logger"),"Received message address: %p", reinterpret_cast<std::uintptr_t>(msg.get()));
    }
  );
}

WebcamSubscriber::~WebcamSubscriber()
{

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto CamSub = std::make_shared<WebcamSubscriber>("CamSub","/image");
    rclcpp::spin(CamSub);
    rclcpp::shutdown();
    return 0;
}
