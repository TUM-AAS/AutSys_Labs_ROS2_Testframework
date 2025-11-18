#include "trajectory_test.hpp"

#include <fstream>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

#define MAX_FLIGHT_TIME 250.0

static double signed_pow2(double val)
{
  return val > 0 ? std::pow(val, 2) : -std::pow(val, 2);
}

TrajectoryTest::TrajectoryTest(const rclcpp::NodeOptions & options)
: Node("trajectory_test_node", options),
  startup_time_(this->now()),
  number_calls_(0),
  number_calls_prop_speeds_(0),
  min_wrench_(10000.0, 10.0, 10.0, 10.0),
  max_wrench_(-10000.0, -10.0, -10.0, -10.0),
  received_trajectory_(false),
  flight_duration_(MAX_FLIGHT_TIME)
{
  // Subscribers
  sub_drone_position_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "current_state", 5,
    std::bind(&TrajectoryTest::checkDronePosition, this, std::placeholders::_1));

  sub_trajectory_ = this->create_subscription<mav_planning_msgs::msg::PolynomialTrajectory4D>(
    "trajectory", 5,
    std::bind(&TrajectoryTest::checkSentTrajectory, this, std::placeholders::_1));

  sub_rotor_speed_cmds_ = this->create_subscription<mav_msgs::msg::Actuators>(
    "rotor_speed_cmds", 5,
    std::bind(&TrajectoryTest::checkRotorSpeeds, this, std::placeholders::_1));

  // Timer (10 Hz)
  timer_check_run_time_ = this->create_wall_timer(
    100ms, std::bind(&TrajectoryTest::checkRunTime, this));

  // Load reference positions from parameters (adapt your YAML to ROS2 param naming)
  readReferencePositions();
}

void TrajectoryTest::readReferencePositions()
{
  reference_positions_.resize(6);
  reached_position_n_times_.resize(reference_positions_.size());
  drone_at_goal_position_.resize(reference_positions_.size());

  for (size_t i = 1; i <= reference_positions_.size(); ++i) {
    std::string vertex_name = "Vertex_" + std::to_string(i);

    // Example ROS2-style parameter name: "Vertex.Vertex_1.pos"
    std::string param_name = "Vertex." + vertex_name + ".pos";

    // Declare with empty default; you can also pre-declare in constructor if you prefer
    this->declare_parameter<std::vector<double>>(param_name, std::vector<double>{});

    std::vector<double> pos;
    if (!this->get_parameter(param_name, pos) || pos.size() < 3) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Parameter '%s' not set or invalid. Expecting at least 3 doubles.", param_name.c_str());
      pos = {0.0, 0.0, 0.0};
    }

    reference_positions_[i - 1] << pos[0], pos[1], pos[2];
  }
}

void TrajectoryTest::checkRunTime()
{
  rclcpp::Time actual_time = this->now();
  double time = (actual_time - startup_time_).seconds();
  if (time > flight_duration_) {
    writeTestResult();
    rclcpp::shutdown();
  }
}

void TrajectoryTest::checkRotorSpeeds(const mav_msgs::msg::Actuators::SharedPtr rotor_speeds)
{
  number_calls_prop_speeds_++;

  Eigen::Vector4d props;
  for (int i = 0; i < 4 && i < static_cast<int>(rotor_speeds->angular_velocities.size()); ++i) {
    props[i] = signed_pow2(rotor_speeds->angular_velocities[i]);
  }

  const double d_hat = 0.3 / std::sqrt(2.0);
  const double cd = 1e-5;
  const double cf = 1e-3;

  Eigen::Matrix4d F;
  F << cf, cf, cf, cf,
       cf * d_hat,  cf * d_hat, -cf * d_hat, -cf * d_hat,
      -cf * d_hat,  cf * d_hat,  cf * d_hat, -cf * d_hat,
       cd,         -cd,          cd,         -cd;

  Eigen::Vector4d wrench = F * props;

  if (received_trajectory_) {
    for (int i = 0; i < 4; ++i) {
      if (wrench[i] < min_wrench_[i]) {
        min_wrench_[i] = wrench[i];
      } else if (wrench[i] > max_wrench_[i]) {
        max_wrench_[i] = wrench[i];
      }
    }
  }
}

void TrajectoryTest::checkSentTrajectory(
  const mav_planning_msgs::msg::PolynomialTrajectory4D::SharedPtr msg)
{
  startup_time_ = this->now();
  flight_duration_ = 2.0;

  for (const auto & segment : msg->segments) {
    // segment_time is builtin_interfaces::msg::Duration
    flight_duration_ += rclcpp::Duration(segment.segment_time).seconds();
  }

  received_trajectory_ = true;
}

void TrajectoryTest::checkDronePosition(const nav_msgs::msg::Odometry::SharedPtr cur_state)
{
  number_calls_++;

  Eigen::Vector3d drone_position;
  drone_position << cur_state->pose.pose.position.x,
                    cur_state->pose.pose.position.y,
                    cur_state->pose.pose.position.z;

  checkGoalPositions(drone_position);

  Eigen::Vector3d vel;
  vel << cur_state->twist.twist.linear.x,
         cur_state->twist.twist.linear.y,
         cur_state->twist.twist.linear.z;

  Eigen::Vector3d last_stop_position;
  if (!stop_positions_.empty()) {
    last_stop_position << stop_positions_.back()[1],
                          stop_positions_.back()[2],
                          stop_positions_.back()[3];
  }

  if (vel.norm() < 0.03 &&
      drone_position.norm() > 0.1 &&
      (stop_positions_.empty() ||
       (drone_position - last_stop_position).norm() > 0.04))
  {
    rclcpp::Time actual_time = this->now();
    double time = (actual_time - startup_time_).seconds();
    Eigen::Vector4d stop_position(time, drone_position[0], drone_position[1], drone_position[2]);
    stop_positions_.push_back(stop_position);
  }
}

void TrajectoryTest::checkGoalPositions(const Eigen::Vector3d & drone_position)
{
  for (size_t i = 0; i < reference_positions_.size(); ++i) {
    if (std::abs((drone_position - reference_positions_[i]).norm()) < 0.5) {
      if (!drone_at_goal_position_[i]) {
        position_sequence_.push_back(static_cast<unsigned int>(i));
        reached_position_n_times_[i]++;
        drone_at_goal_position_[i] = true;
      }
    } else {
      drone_at_goal_position_[i] = false;
    }
  }
}

void TrajectoryTest::writeTestResult()
{
  std::ofstream results("../results.txt");
  if (!results.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "No Results available!");
    return;
  }

  if (number_calls_prop_speeds_ == 0) {
    max_wrench_ = { -std::nan(""), -std::nan(""), -std::nan(""), -std::nan("") };
    min_wrench_ = {  std::nan(""),  std::nan(""),  std::nan(""),  std::nan("") };
  }

  results << "##########################\nResult Report:\n"
          << "Recorded flight time: " << flight_duration_ << std::endl
          << "Tested drone positions: " << number_calls_ << std::endl
          << "Tested rotor speed commands: " << number_calls_prop_speeds_ << std::endl
          << "Received (elementwise) maximum wrench: "
          << max_wrench_[0] << ", " << max_wrench_[1] << ", "
          << max_wrench_[2] << ", " << max_wrench_[3] << "\n"
          << "Received (elementwise) minimum wrench: "
          << min_wrench_[0] << ", " << min_wrench_[1] << ", "
          << min_wrench_[2] << ", " << min_wrench_[3] << "\n"
          << "Goal sequence: ";

  // print sequence
  for (size_t i = 0; i < position_sequence_.size(); ++i) {
    int pos = static_cast<int>(position_sequence_[i]) + 1;
    results << pos;
    if (i != position_sequence_.size() - 1) {
      results << ", ";
    }
  }

  // print reached positions
  results << "\nCounted drone transitions for all "
          << reference_positions_.size() << " goals:\n";
  for (size_t i = 0; i < reference_positions_.size(); ++i) {
    results << "    Counted transitions for ["
            << reference_positions_[i][0] << ", "
            << reference_positions_[i][1] << ", "
            << reference_positions_[i][2]
            << "]: " << reached_position_n_times_[i] << std::endl;
  }

  results << "Reported number of drone positions with 0 velocity: "
          << stop_positions_.size() << std::endl;

  for (const auto & pos : stop_positions_) {
    results << "    [" << pos[1] << ", " << pos[2] << ", " << pos[3]
            << "] at time: " << pos[0] << std::endl;
  }

  results << "##########################\n\n";
  results.close();
}

// ROS2 main
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryTest>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
