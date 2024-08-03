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
find_package(OpenCV 4.4.0 REQUIRED)
find_package(cv_bridge REQUIRED)

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

# No inherit Node CPP File
# add_library(node_like_listener_component SHARED
#   src/node_like_listener_component.cpp)
# target_compile_definitions(node_like_listener_component
#   PRIVATE "COMPOSITION_BUILDING_DLL")
# ament_target_dependencies(node_like_listener_component
#   "rclcpp"
#   "rclcpp_components"
#   "std_msgs")
# rclcpp_components_register_nodes(node_like_listener_component "composition::NodeLikeListener")
# set(node_plugins "${node_plugins}composition::NodeLikeListener;$<TARGET_FILE:node_like_listener_component>\n")


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
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  component_producer
  component_consumer
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
