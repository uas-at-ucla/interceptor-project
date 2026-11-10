cmake_minimum_required(VERSION 3.8)
project(interceptor_mavros_stub)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mavros_stub_node src/mavros_stub_node.cpp)
target_compile_features(mavros_stub_node PUBLIC cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(mavros_stub_node PRIVATE -Wall -Wextra -Wpedantic)
endif()
ament_target_dependencies(mavros_stub_node rclcpp std_msgs)

install(TARGETS mavros_stub_node DESTINATION lib/${PROJECT_NAME})

ament_package()
