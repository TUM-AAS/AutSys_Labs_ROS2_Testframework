cmake_minimum_required(VERSION 3.8)
project(testframework)

# Use at least C++14 for rclcpp
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find ament and ROS2 deps
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(mav_planning_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include dirs
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(trajectory_test
  src/trajectory_test.cpp
)

target_include_directories(trajectory_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(trajectory_test
  rclcpp
  nav_msgs
  mav_msgs
  mav_planning_msgs
)

# Install the node binary
install(
  TARGETS trajectory_test
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
