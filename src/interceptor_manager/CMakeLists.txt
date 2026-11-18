cmake_minimum_required(VERSION 3.8)
project(interceptor_manager)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interceptor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)

# Pure geometry, no ROS node code, so it can be unit-tested on its own.
add_library(manager_math src/manager_math.cpp)
target_include_directories(manager_math PUBLIC include)
ament_target_dependencies(manager_math geometry_msgs)

add_executable(manager_node src/manager_node.cpp)
ament_target_dependencies(manager_node
  rclcpp geometry_msgs interceptor_msgs mavros_msgs std_srvs tf2)

install(TARGETS manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_manager_math test/test_manager_math.cpp)
  target_link_libraries(test_manager_math manager_math)
endif()

ament_package()
