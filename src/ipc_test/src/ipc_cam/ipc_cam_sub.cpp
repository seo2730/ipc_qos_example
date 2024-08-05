#include "ipc_cam/ipc_cam_sub.hpp"

WebcamSubscriber::WebcamSubscriber(const std::string &name, const std::string &input) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
  // Create a subscription on the input topic which prints on receipt of new messages
  this->subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(input, 10, 
    [=](sensor_msgs::msg::Image::UniquePtr msg){
      RCLCPP_WARN(rclcpp::get_logger("logger"),"Received message address: %p", reinterpret_cast<std::uintptr_t>(msg.get()));
      // this->PrintProcessUsage(this->pid_);
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

void WebcamSubscriber::PrintProcessUsage(pid_t pid) 
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
