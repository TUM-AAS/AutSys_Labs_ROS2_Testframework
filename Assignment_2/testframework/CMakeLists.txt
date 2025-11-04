cmake_minimum_required(VERSION 3.10)
project(testframework)

# C++ standard (ROS 2 typically uses C++17)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Executable
add_executable(controller_test src/controller_test.cpp)
ament_target_dependencies(controller_test
  rclcpp
  tf2_eigen
  mav_msgs
  trajectory_msgs
)

# (If you need Eigen includes explicitly)
target_include_directories(controller_test PRIVATE ${EIGEN3_INCLUDE_DIRS})

# Install so ros2 can find the binary
install(TARGETS controller_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Exports
ament_export_dependencies(
  rclcpp
  tf2_eigen
  Eigen3
)

ament_package()
