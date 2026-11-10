cmake_minimum_required(VERSION 3.8)
project(interceptor_vision)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(vision_camera_node src/vision_camera_node.cpp)
target_compile_features(vision_camera_node PUBLIC cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(vision_camera_node PRIVATE -Wall -Wextra -Wpedantic)
endif()
ament_target_dependencies(vision_camera_node rclcpp std_msgs)

install(TARGETS vision_camera_node DESTINATION lib/${PROJECT_NAME})

ament_package()
