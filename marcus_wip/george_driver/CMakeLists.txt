cmake_minimum_required(VERSION 3.8)
project(taylor_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(taylor_driver_node
  src/main.cpp
  src/taylor_driver.cpp
)

ament_target_dependencies(taylor_driver_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2   
  std_msgs
)

install(TARGETS
  taylor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

