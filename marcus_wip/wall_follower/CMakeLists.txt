cmake_minimum_required(VERSION 3.8)
project(wall_follower)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(lidar_processor REQUIRED)

# Include directories
include_directories(include)

# Build the wall follower executable
add_executable(wall_follower_node
  src/main.cpp
  src/wall_follower.cpp
)

# Link dependencies
ament_target_dependencies(wall_follower_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  lidar_processor
)

# Install executable
install(TARGETS wall_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers (for other packages to use)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
