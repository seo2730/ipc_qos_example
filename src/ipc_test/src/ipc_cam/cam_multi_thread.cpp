#include "ipc_cam/ipc_cam_pub.hpp"
#include "ipc_cam/ipc_cam_sub.hpp"

void PrintProcessUsage(pid_t pid) 
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

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto cam_pub = std::make_shared<WebcamPublisher>("ipc_cam_pub", "image");
  auto cam_sub = std::make_shared<WebcamSubscriber>("ipc_cam_sub", "image");

  executor.add_node(cam_pub);
  executor.add_node(cam_sub);

  pid_t pid = getpid();
  auto previous_time = std::chrono::steady_clock::now();
  
  while(rclcpp::ok())
  {
    executor.spin_some();

    // 현재 시간 계산
    auto current_time = std::chrono::steady_clock::now();

    // 두 시간 사이의 경과 시간 계산
    auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - previous_time);

    // 1초 이상 경과했는지 확인
    if (elapsed_time.count() >= 1)
    {
        PrintProcessUsage(pid); // 프로세스 사용량 출력
        previous_time = current_time; // 이전 시간 업데이트
    }
  }
  rclcpp::shutdown();

  return 0;
}
