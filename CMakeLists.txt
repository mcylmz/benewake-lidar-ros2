cmake_minimum_required(VERSION 3.14)
project(benewake_lidar)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# System dependencies
find_package(Boost REQUIRED COMPONENTS system)
find_package(nlohmann_json REQUIRED)

# Executable
add_executable(benewake_lidar_scan_node
  src/benewake_lidar_node.cpp
  src/udp_receiver.cpp
  src/scan_assembler.cpp
  src/sensor_rest_client.cpp
)

target_include_directories(benewake_lidar_scan_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(benewake_lidar_scan_node
  rclcpp
  sensor_msgs
)

target_link_libraries(benewake_lidar_scan_node
  Boost::system
  nlohmann_json::nlohmann_json
)

# Install
install(TARGETS benewake_lidar_scan_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
