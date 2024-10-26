cmake_minimum_required(VERSION 3.5)
project(ipc_qos_example)

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
find_package(OpenCV 4.4.0 REQUIRED)
find_package(cv_bridge REQUIRED)

# gtest 라이브러리 찾기
find_package(ament_cmake_gtest REQUIRED)

# To use rclcpp_components_register_nodes
find_package(rclcpp_components REQUIRED)

# include directories
include_directories(include
  src/ipc_test/include
  ${OpenCV_INCLUDE_DIRS}
)

# Register Component Progress
# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(component_producer SHARED
  src/component_test/component_producer.cpp)
target_compile_definitions(component_producer
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(component_producer
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_nodes(component_producer "ipc_test::Producer")
set(node_plugins "${node_plugins}ipc_test::Producer;$<TARGET_FILE:component_producer>\n")
add_library(component_consumer SHARED
  src/component_test/component_consumer.cpp)
target_compile_definitions(component_consumer
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(component_consumer
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_nodes(component_consumer "ipc_test::Consumer")
set(node_plugins "${node_plugins}ipc_test::Consumer;$<TARGET_FILE:component_consumer>\n")

set(node_plugins "")
add_library(component_cam_pub SHARED
  src/component_cam/component_cam_pub.cpp)
target_compile_definitions(component_cam_pub
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(component_cam_pub ${OpenCV_LIBS})
ament_target_dependencies(component_cam_pub
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs"
  "OpenCV"
  "cv_bridge")
rclcpp_components_register_nodes(component_cam_pub "component_cam::WebcamPublisher")
set(node_plugins "${node_plugins}component_cam::WebcamPublisher;$<TARGET_FILE:component_cam_pub>\n")

add_library(component_cam_sub SHARED
  src/component_cam/component_cam_sub.cpp)
target_compile_definitions(component_cam_sub
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(component_cam_sub ${OpenCV_LIBS})
ament_target_dependencies(component_cam_sub
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs"
  "OpenCV"
  "cv_bridge")
rclcpp_components_register_nodes(component_cam_sub "component_cam::WebcamSubscriber")
set(node_plugins "${node_plugins}component_cam::WebcamSubscriber;$<TARGET_FILE:component_cam_sub>\n")

# set 
set(ipc_test_1_src
  src/ipc_test/ipc_test1.cpp  
)
set(non_ipc_test_producer_src
  src/ipc_test/non_ipc_test_producer.cpp  
)
set(non_ipc_test_consumer_src
  src/ipc_test/non_ipc_test_consumer.cpp  
)
set(two_node_single_thread_src
  src/ipc_test/ipc_test_producer.cpp
  src/ipc_test/ipc_test_consumer.cpp  
  src/ipc_test/two_node_single_thread.cpp
)
set(two_node_multi_thread_src
  src/ipc_test/ipc_test_producer.cpp
  src/ipc_test/ipc_test_consumer.cpp  
  src/ipc_test/two_node_multi_thread.cpp
)

set(ipc_test_cam_src
  src/ipc_cam/ipc_test_cam.cpp
)
set(non_ipc_cam_pub_src
  src/ipc_cam/non_ipc_cam_pub.cpp
)
set(non_ipc_cam_sub_src
  src/ipc_cam/non_ipc_cam_sub.cpp
)
set(cam_single_thread_src
  src/ipc_cam/ipc_cam_pub.cpp
  src/ipc_cam/ipc_cam_sub.cpp
  src/ipc_cam/cam_single_thread.cpp
)
set(cam_multi_thread_src
  src/ipc_cam/ipc_cam_pub.cpp
  src/ipc_cam/ipc_cam_sub.cpp
  src/ipc_cam/cam_multi_thread.cpp
)

# add executable
add_executable(ipc_test1 ${ipc_test_1_src})
ament_target_dependencies(ipc_test1 rclcpp std_msgs)
add_executable(non_ipc_test_producer ${non_ipc_test_producer_src})
ament_target_dependencies(non_ipc_test_producer rclcpp std_msgs)
add_executable(non_ipc_test_consumer ${non_ipc_test_consumer_src})
ament_target_dependencies(non_ipc_test_consumer rclcpp std_msgs)
add_executable(two_node_single_thread ${two_node_single_thread_src})
ament_target_dependencies(two_node_single_thread rclcpp std_msgs)
add_executable(two_node_multi_thread ${two_node_multi_thread_src})
ament_target_dependencies(two_node_multi_thread rclcpp sensor_msgs)

add_executable(ipc_test_cam ${ipc_test_cam_src})
ament_target_dependencies(ipc_test_cam rclcpp std_msgs OpenCV cv_bridge)
target_link_libraries(ipc_test_cam ${OpenCV_LIBS})
add_executable(non_ipc_cam_pub ${non_ipc_cam_pub_src})
ament_target_dependencies(non_ipc_cam_pub rclcpp std_msgs OpenCV cv_bridge)
target_link_libraries(non_ipc_cam_pub ${OpenCV_LIBS})
add_executable(non_ipc_cam_sub ${non_ipc_cam_sub_src})
ament_target_dependencies(non_ipc_cam_sub rclcpp std_msgs OpenCV cv_bridge)
target_link_libraries(non_ipc_cam_sub ${OpenCV_LIBS})
add_executable(cam_single_thread ${cam_single_thread_src})
ament_target_dependencies(cam_single_thread rclcpp std_msgs OpenCV cv_bridge)
target_link_libraries(cam_single_thread ${OpenCV_LIBS})
add_executable(cam_multi_thread ${cam_multi_thread_src})
ament_target_dependencies(cam_multi_thread rclcpp std_msgs OpenCV cv_bridge)
target_link_libraries(cam_multi_thread ${OpenCV_LIBS})

set(test_src
  test/test_ipc_test_consumer.cpp
  src/ipc_test/ipc_test_consumer.cpp
)
# 테스트 파일 추가
ament_add_gtest(test_ipc_test_consumer ${test_src})
ament_target_dependencies(test_ipc_test_consumer rclcpp std_msgs)

# install
install(TARGETS
  ipc_test1
  non_ipc_test_producer
  non_ipc_test_consumer
  two_node_single_thread
  two_node_multi_thread
  ipc_test_cam
  non_ipc_cam_pub
  non_ipc_cam_sub
  cam_single_thread
  cam_multi_thread
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  component_producer
  component_consumer
  component_cam_pub
  component_cam_sub
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
