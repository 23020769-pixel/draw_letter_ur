#include <array>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "moveit_probe",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  {
    moveit::planning_interface::MoveGroupInterface move_group(
      node, "ur_manipulator");

    move_group.setPoseReferenceFrame(move_group.getPlanningFrame());
    move_group.setEndEffectorLink("tool0");

    const auto current = move_group.getCurrentPose("tool0");

    RCLCPP_INFO(node->get_logger(),
                "Pose goc: x=%.3f, y=%.3f, z=%.3f",
                current.pose.position.x,
                current.pose.position.y,
                current.pose.position.z);

    const std::array<std::pair<std::string, std::array<double, 3>>, 6> tests{{
      {"X+", { 0.01, 0.00, 0.00}},
      {"X-", {-0.01, 0.00, 0.00}},
      {"Y+", { 0.00, 0.01, 0.00}},
      {"Y-", { 0.00,-0.01, 0.00}},
      {"Z+", { 0.00, 0.00, 0.01}},
      {"Z-", { 0.00, 0.00,-0.01}}
    }};

    for (const auto & test : tests) {
      geometry_msgs::msg::Pose target = current.pose;
      target.position.x += test.second[0];
      target.position.y += test.second[1];
      target.position.z += test.second[2];

      move_group.setStartStateToCurrentState();

      moveit_msgs::msg::RobotTrajectory trajectory;
      moveit_msgs::msg::MoveItErrorCodes error_code;
      const double fraction = move_group.computeCartesianPath(
        std::vector<geometry_msgs::msg::Pose>{target},
        0.002, 0.0, trajectory, true, &error_code);

      RCLCPP_INFO(node->get_logger(),
                  "Huong %s: fraction=%.3f, error_code=%d",
                  test.first.c_str(), fraction, error_code.val);
    }
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
