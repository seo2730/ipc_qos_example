#include "component_cam/component_cam_pub.hpp"

namespace component_cam
{

WebcamPublisher::WebcamPublisher(const rclcpp::NodeOptions &options) : Node("component_cam_pub", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  this->publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
  std::weak_ptr<std::remove_pointer<decltype(this->publisher_.get())>::type> captured_pub = this->publisher_;
  
  // Create a timer which publishes on the output topic at ~1Hz
  auto callback = [=]() -> void{
    auto pub_ptr = captured_pub.lock();
    if(!pub_ptr){
      return;
    }
    cv::Mat frame;
    cap_ >> frame;

    if (!frame.empty()) {
      //   OpenCV 이미지를 ROS 이미지 메시지로 변환
      std_msgs::msg::Header header; // 헤더를 초기화
      header.stamp = this->get_clock()->now(); // 현재 시간을 타임스탬프로 사용
      
      // cv_bridge를 사용하여 std::shared_ptr로 이미지 메시지 생성
      auto shared_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
      // std::shared_ptr를 std::unique_ptr로 이동하여 변환
      auto unique_msg = std::make_unique<sensor_msgs::msg::Image>(*shared_msg);
      RCLCPP_INFO(this->get_logger(), "Publishing image width : %f, height : %f", cap_.get(cv::CAP_PROP_FRAME_WIDTH), cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
      RCLCPP_INFO(this->get_logger(), "Publishing image message with size %d and address: %p", sizeof(unique_msg->data), reinterpret_cast<std::uintptr_t>(unique_msg.get()));
      publisher_->publish(std::move(unique_msg));
      this->PrintProcessUsage(this->pid_);

      // RCLCPP_INFO(rclcpp::get_logger("logger"), "Published message with address: %p", reinterpret_cast<std::uintptr_t>(shared_msg.get()));  
      // publisher_->publish(*shared_msg);
      // this->PrintProcessUsage(this->pid_);
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

void WebcamPublisher::PrintProcessUsage(pid_t pid) 
{
  // this->count_++;
  std::ifstream stat_file("/proc/" + std::to_string(pid) + "/stat");
  if (!stat_file.is_open()) {
      std::cerr << "Failed to open /proc/" << pid << "/stat" << std::endl;
      return;
  }

  std::string line;
  std::getline(stat_file, line);
  std::istringstream iss(line);

  std::vector<std::string> values;
  std::string value;
  while (iss >> value) {
      values.push_back(value);
  }

  stat_file.close();

  if (values.size() < 24) {
      std::cerr << "Error reading /proc/" << pid << "/stat" << std::endl;
      return;
  }

  unsigned long utime = std::stoul(values[13]);
  unsigned long stime = std::stoul(values[14]);
  unsigned long starttime = std::stoul(values[21]);
  long rss = std::stol(values[23]);

  // CPU 사용량 계산
  // double hertz = sysconf(_SC_CLK_TCK);
  // double total_time = (utime + stime) / hertz;

  std::ifstream uptime_file("/proc/uptime");
  double uptime;
  uptime_file >> uptime;
  uptime_file.close();

  // double process_uptime = uptime - (starttime / hertz);
  // double cpu_usage = 100.0 * (total_time / process_uptime);

  // 메모리 사용량 계산
  long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // 페이지 크기를 KB로 변환
  long resident_set = rss * page_size_kb;

  RCLCPP_ERROR(rclcpp::get_logger("logger"), "Process ID: %d, Memory: %dKB", static_cast<int>(pid), resident_set);
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(component_cam::WebcamPublisher)
