#ifndef TRAJECTORY_TEST_HPP_
#define TRAJECTORY_TEST_HPP_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>
#include <mav_msgs/msg/actuators.hpp>
#include <mav_planning_msgs/msg/polynomial_trajectory4_d.hpp>
#include <vector>

class TrajectoryTest : public rclcpp::Node
{
public:
  explicit TrajectoryTest(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Public for potential unit tests; used as callbacks internally
  void checkRunTime();
  void checkRotorSpeeds(const mav_msgs::msg::Actuators::SharedPtr rotor_speeds);
  void checkSentTrajectory(const mav_planning_msgs::msg::PolynomialTrajectory4D::SharedPtr msg);
  void checkDronePosition(const nav_msgs::msg::Odometry::SharedPtr cur_state);
  void writeTestResult();

private:
  void readReferencePositions();
  void checkGoalPositions(const Eigen::Vector3d & drone_position);

  rclcpp::Time startup_time_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_drone_position_;
  rclcpp::Subscription<mav_planning_msgs::msg::PolynomialTrajectory4D>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<mav_msgs::msg::Actuators>::SharedPtr sub_rotor_speed_cmds_;
  rclcpp::TimerBase::SharedPtr timer_check_run_time_;

  std::vector<Eigen::Vector3d> reference_positions_;
  std::vector<unsigned int> reached_position_n_times_;
  std::vector<unsigned int> position_sequence_;
  std::vector<bool> drone_at_goal_position_;

  int number_calls_;
  unsigned long number_calls_prop_speeds_;
  Eigen::Vector4d min_wrench_;
  Eigen::Vector4d max_wrench_;

  std::vector<Eigen::Vector4d> stop_positions_;
  bool received_trajectory_;

  double flight_duration_;
};

#endif  // TRAJECTORY_TEST_HPP_
