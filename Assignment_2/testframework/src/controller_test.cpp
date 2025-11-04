#include <cmath>
#include <fstream>
#include <limits>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>

#include <nav_msgs/msg/odometry.hpp>
#include <mav_msgs/msg/actuators.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

#define START_TIME 5.0
#define SHUTDOWN_TIME 30.0

class ControllerTest : public rclcpp::Node
{
public:
  ControllerTest()
  : rclcpp::Node("controller_test_node"),
    maxDeviation_(0.0),
    averageDeviation_(0.0),
    numberDeviationOutsideThreshold_(0),
    numberCalls_(0),
    numberCallsPropSpeeds_(0),
    referencePosition_(0.0, 0.0, 0.0),
    minWrench_(10000.0, 10.0, 10.0, 10.0),
    maxWrench_(-10000.0, -10.0, -10.0, -10.0),
    averageWrench_(0.0, 0.0, 0.0, 0.0)
  {
    startup_time_ = this->now();

    sub_dronePosition_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "current_state", rclcpp::QoS(5),
      std::bind(&ControllerTest::checkDronePosition, this, std::placeholders::_1));

    sub_desPosition_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(
      "desired_state", rclcpp::QoS(5),
      std::bind(&ControllerTest::saveReferencePosition, this, std::placeholders::_1));

    sub_rotorSpeedCmds_ = this->create_subscription<mav_msgs::msg::Actuators>(
      "rotor_speed_cmds", rclcpp::QoS(5),
      std::bind(&ControllerTest::checkRotorSpeeds, this, std::placeholders::_1));

    // 10 Hz watchdog for runtime limit
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ControllerTest::checkRunTime, this));

    RCLCPP_INFO(this->get_logger(), "controller_test node started.");
  }

private:
  static double signed_pow2(double val) { return val >= 0 ? val * val : -val * val; }

  void checkRunTime()
  {
    const double t = (this->now() - startup_time_).seconds();
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /*ms*/,
      "controller_test running time: %.1f s", t);
    if (t > SHUTDOWN_TIME) {
      writeTestResult();
      rclcpp::shutdown();
    }
  }

  void checkRotorSpeeds(const mav_msgs::msg::Actuators::SharedPtr rotor_speeds)
  {
    const double t = (this->now() - startup_time_).seconds();
    if (t < START_TIME) return;

    if (rotor_speeds->angular_velocities.size() < 4) return;

    numberCallsPropSpeeds_++;

    Eigen::Vector4d props;
    for (int i = 0; i < 4; ++i) {
      props[i] = signed_pow2(rotor_speeds->angular_velocities[i]);
    }

    const double d_hat = 0.3 / std::sqrt(2.0);
    const double cd = 1e-5;
    const double cf = 1e-3;

    Eigen::Matrix4d F;
    F <<  cf,       cf,       cf,       cf,
          cf*d_hat, cf*d_hat, -cf*d_hat, -cf*d_hat,
         -cf*d_hat, cf*d_hat,  cf*d_hat, -cf*d_hat,
          cd,      -cd,        cd,      -cd;

    Eigen::Vector4d wrench = F * props;

    averageWrench_ += wrench;
    for (int i = 0; i < 4; ++i) {
      if (wrench[i] < minWrench_[i]) minWrench_[i] = wrench[i];
      if (wrench[i] > maxWrench_[i]) maxWrench_[i] = wrench[i];
    }
  }

  void saveReferencePosition(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint::SharedPtr des_state)
  {
    if (des_state->transforms.empty()) return;
    const auto & t = des_state->transforms[0].translation;
    referencePosition_ << t.x, t.y, t.z;
  }

  void checkDronePosition(const nav_msgs::msg::Odometry::SharedPtr cur_state)
  {
    const double t = (this->now() - startup_time_).seconds();
    if (t < START_TIME) return;

    Eigen::Vector3d dronePosition(
      cur_state->pose.pose.position.x,
      cur_state->pose.pose.position.y,
      cur_state->pose.pose.position.z
    );

    const double deviation = (dronePosition - referencePosition_).norm();
    averageDeviation_ += deviation;
    if (deviation > maxDeviation_) maxDeviation_ = deviation;
    if (deviation > 0.5) ++numberDeviationOutsideThreshold_;
    ++numberCalls_;
  }

  void writeTestResult()
  {
    std::ofstream results("../results.txt");
    if (!results.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "No Results available (failed to open results file).");
      return;
    }

    if (numberCalls_ == 0) {
      maxDeviation_ = std::numeric_limits<double>::quiet_NaN();
      numberDeviationOutsideThreshold_ = std::numeric_limits<int>::max();
    }

    if (numberCallsPropSpeeds_ == 0) {
      const double NaN = std::numeric_limits<double>::quiet_NaN();
      maxWrench_ = Eigen::Vector4d(-NaN, -NaN, -NaN, -NaN);
      minWrench_ = Eigen::Vector4d(NaN, NaN, NaN, NaN);
    } else {
      averageWrench_ /= static_cast<double>(numberCallsPropSpeeds_);
    }

    results << "##########################\nResult Report:\n"
            << "Tested drone positions: " << numberCalls_ << "\n"
            << "Average deviation from optimal route: "
            << (numberCalls_ ? (averageDeviation_ / static_cast<double>(numberCalls_)) : std::numeric_limits<double>::quiet_NaN()) << "\n"
            << "Maximum deviation from optimal route: " << maxDeviation_ << "\n"
            << "Number of drone positions outside flight path threshold: " << numberDeviationOutsideThreshold_ << "\n"
            << "Tested rotor speed commands: " << numberCallsPropSpeeds_ << "\n"
            << "Received (elementwise) average wrench: " << averageWrench_[0] << ", " << averageWrench_[1] << ", " << averageWrench_[2] << ", " << averageWrench_[3] << "\n"
            << "Received (elementwise) maximum wrench: " << maxWrench_[0] << ", " << maxWrench_[1] << ", " << maxWrench_[2] << ", " << maxWrench_[3] << "\n"
            << "Received (elementwise) minimum wrench: " << minWrench_[0] << ", " << minWrench_[1] << ", " << minWrench_[2] << ", " << minWrench_[3] << "\n"
            << "##########################\n\n";
    results.close();
    RCLCPP_INFO(this->get_logger(), "Wrote test results to ../../../results.txt");
  }

private:
  rclcpp::Time startup_time_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_dronePosition_;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr sub_desPosition_;
  rclcpp::Subscription<mav_msgs::msg::Actuators>::SharedPtr sub_rotorSpeedCmds_;
  rclcpp::TimerBase::SharedPtr timer_;

  Eigen::Vector3d referencePosition_;

  double maxDeviation_;
  double averageDeviation_;
  int    numberDeviationOutsideThreshold_;
  int    numberCalls_;
  unsigned long numberCallsPropSpeeds_;
  Eigen::Vector4d minWrench_;
  Eigen::Vector4d maxWrench_;
  Eigen::Vector4d averageWrench_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerTest>());
  rclcpp::shutdown();
  return 0;
}
