cmake_minimum_required(VERSION 3.8)
project(interceptor_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interceptor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(manager_node src/manager_node.cpp)
target_compile_features(manager_node PUBLIC cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(manager_node PRIVATE -Wall -Wextra -Wpedantic)
endif()
ament_target_dependencies(manager_node
  rclcpp geometry_msgs interceptor_msgs mavros_msgs std_srvs tf2)

install(TARGETS manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
