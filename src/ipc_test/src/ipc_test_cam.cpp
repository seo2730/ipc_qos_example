#include "ipc_test/ipc_test_cam.hpp"

WebcamPublisher::WebcamPublisher(const std::string &name, const std::string &output) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
  this->publisher_ = this->create_publisher<sensor_msgs::msg::Image>(output, 10);
  std::weak_ptr<std::remove_pointer<decltype(this->publisher_.get())>::type> captured_pub = this->publisher_;
  
  // Create a timer which publishes on the output topic at ~1Hz
  auto callback = [=]() -> void{
    auto pub_ptr = captured_pub.lock();
    if(!pub_ptr){
      return;
    }
    cv::Mat frame;
    cap_ >> frame;

    if(!frame.empty()) {
      //   OpenCV 이미지를 ROS 이미지 메시지로 변환
      std_msgs::msg::Header header; // 헤더를 초기화
      header.stamp = this->get_clock()->now(); // 현재 시간을 타임스탬프로 사용
      
      // cv_bridge를 사용하여 std::shared_ptr로 이미지 메시지 생성
      auto shared_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
      // std::shared_ptr를 std::unique_ptr로 이동하여 변환
      auto unique_msg = std::make_unique<sensor_msgs::msg::Image>(*shared_msg);
      RCLCPP_INFO(this->get_logger(), "Publishing image width : %f, height : %f", cap_.get(cv::CAP_PROP_FRAME_WIDTH), cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
      RCLCPP_INFO(this->get_logger(), "Publishing image message with address: %p", reinterpret_cast<std::uintptr_t>(unique_msg.get()));
      publisher_->publish(std::move(unique_msg));
      
      // RCLCPP_INFO(rclcpp::get_logger("logger"), "Published message with address: %p", reinterpret_cast<std::uintptr_t>(shared_msg.get()));  
      // publisher_->publish(*shared_msg);
      
      // auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
      // // sensor_msgs::msg::Image::UniquePtr img_msg(new sensor_msgs::msg::Image());
      // // img_msg = std::make_unique<sensor_msgs::msg::Image>(*msg);
      // // RCLCPP_INFO(rclcpp::get_logger("logger"), "Published message with address: %p", reinterpret_cast<std::uintptr_t>(img_msg.get()));
      // // pub_ptr->publish(std::move(img_msg));
      
      // RCLCPP_INFO(rclcpp::get_logger("logger"), "Published message with address: %p", reinterpret_cast<std::uintptr_t>(msg.get()));  
      // publisher_->publish(std::move(msg));
    } 
    else {
      RCLCPP_ERROR(this->get_logger(), "Failed to capture image");
    }
  };

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      callback
  );

  cap_.open(0); // 웹캠 장치 ID가 0번이라고 가정

  if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
  }

  // cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);  // 원하는 해상도 너비
  // cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480); // 원하는 해상도 높이
}

WebcamPublisher::~WebcamPublisher()
{
  if (cap_.isOpened()) {
    cap_.release();
    RCLCPP_INFO(this->get_logger(), "Camera released");
  }
}

void WebcamPublisher::timerCallback()
{
  cv::Mat frame;
  cap_ >> frame;

  if (!frame.empty()) {
    //   OpenCV 이미지를 ROS 이미지 메시지로 변환
    std_msgs::msg::Header header; // 헤더를 초기화
    header.stamp = this->get_clock()->now(); // 현재 시간을 타임스탬프로 사용

    auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
    publisher_->publish(*msg);
  } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to capture image");
  }
}

WebcamSubscriber::WebcamSubscriber(const std::string &name, const std::string &input) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
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
    rclcpp::executors::SingleThreadedExecutor executor;
    // rclcpp::executors::MultiThreadedExecutor executor;
    auto CamPub = std::make_shared<WebcamPublisher>("CamPub","/image");
    auto CamSub = std::make_shared<WebcamSubscriber>("CamSub","/image");
    executor.add_node(CamPub);
    executor.add_node(CamSub);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
