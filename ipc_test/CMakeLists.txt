cmake_minimum_required(VERSION 3.5)
project(ipc_test)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# include directories
include_directories(include
  src/ipc_test/include
)

# set 
set(ipc_test_1_src
  src/ipc_test1.cpp  
)

set(non_ipc_test_producer_src
  src/non_ipc_test_producer.cpp  
)

set(non_ipc_test_consumer_src
  src/non_ipc_test_consumer.cpp  
)

# add executable
add_executable(ipc_test1 ${ipc_test_1_src})
ament_target_dependencies(ipc_test1 rclcpp std_msgs)

add_executable(non_ipc_test_producer ${non_ipc_test_producer_src})
ament_target_dependencies(non_ipc_test_producer rclcpp std_msgs)

add_executable(non_ipc_test_consumer ${non_ipc_test_consumer_src})
ament_target_dependencies(non_ipc_test_consumer rclcpp std_msgs)

# install
install(TARGETS
  ipc_test1
  non_ipc_test_producer
  non_ipc_test_consumer
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
