cmake_minimum_required(VERSION 3.8)
project(trajectory_mapper)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(include)

# Build trajectory mapper node
add_executable(trajectory_mapper_node
  src/main.cpp
  src/trajectory_mapper.cpp
)

# Link dependencies
ament_target_dependencies(trajectory_mapper_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
)

# Install executable
install(TARGETS trajectory_mapper_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
