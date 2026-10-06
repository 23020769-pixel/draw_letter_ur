#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

using MoveGroup = moveit::planning_interface::MoveGroupInterface;

bool moveCartesianTo(
  MoveGroup & group,
  const geometry_msgs::msg::Pose & target,
  const rclcpp::Logger & logger,
  const std::string & name)
{
  const auto start = group.getCurrentPose("tool0").pose;

  const double dx = target.position.x - start.position.x;
  const double dy = target.position.y - start.position.y;
  const double dz = target.position.z - start.position.z;
  const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
  const int count = std::max(1, static_cast<int>(std::ceil(length / 0.01)));

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.reserve(count);

  for (int i = 1; i <= count; ++i) {
    const double t = static_cast<double>(i) / count;
    auto point = start;
    point.position.x += t * dx;
    point.position.y += t * dy;
    point.position.z += t * dz;
    waypoints.push_back(point);
  }

  group.setStartStateToCurrentState();

  moveit_msgs::msg::RobotTrajectory trajectory;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  const double fraction = group.computeCartesianPath(
    waypoints, 0.005, 0.0, trajectory, true, &error_code);

  RCLCPP_INFO(logger, "%s: fraction=%.3f, error_code=%d",
              name.c_str(), fraction, error_code.val);

  if (fraction < 0.99) {
    RCLCPP_ERROR(logger, "%s chua dat 99%%; dung lai.",
                 name.c_str());
    return false;
  }

  const auto result = group.execute(trajectory);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Thuc thi %s that bai.", name.c_str());
    return false;
  }

  return true;
}

bool drawLetterT(
  MoveGroup & group,
  const geometry_msgs::msg::Pose & start,
  const rclcpp::Logger & logger)
{
  constexpr double half_width = 0.03;   // tổng rộng 6 cm
  constexpr double height = 0.08;      // cao 8 cm

  auto right = start;
  right.position.x += half_width;

  auto left = start;
  left.position.x -= half_width;

  auto center = start;

  auto bottom = start;
  bottom.position.z -= height;

  if (!moveCartesianTo(group, right, logger, "Thanh ngang: giua -> phai")) {
    return false;
  }
  if (!moveCartesianTo(group, left, logger, "Thanh ngang: phai -> trai")) {
    return false;
  }
  if (!moveCartesianTo(group, center, logger, "Thanh ngang: trai -> giua")) {
    return false;
  }
  return moveCartesianTo(group, bottom, logger, "Than chu T");
}

bool drawCircle(
  MoveGroup & group,
  const geometry_msgs::msg::Pose & start,
  const rclcpp::Logger & logger)
{
  constexpr double radius = 0.03;
  constexpr int count = 72;
  constexpr double pi = 3.14159265358979323846;

  const double center_x = start.position.x - radius;
  const double center_z = start.position.z;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.reserve(count);

  for (int i = 1; i <= count; ++i) {
    const double angle = 2.0 * pi * i / count;
    auto point = start;
    point.position.x = center_x + radius * std::cos(angle);
    point.position.z = center_z + radius * std::sin(angle);
    waypoints.push_back(point);
  }

  group.setStartStateToCurrentState();

  moveit_msgs::msg::RobotTrajectory trajectory;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  const double fraction = group.computeCartesianPath(
    waypoints, 0.005, 0.0, trajectory, true, &error_code);

  RCLCPP_INFO(logger, "Hinh tron: fraction=%.3f, error_code=%d",
              fraction, error_code.val);

  if (fraction < 0.99) {
    RCLCPP_ERROR(logger, "Hinh tron chua dat 99%%; dung lai.");
    return false;
  }

  return group.execute(trajectory) ==
    moveit::core::MoveItErrorCode::SUCCESS;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "draw_letter",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  if (!node->has_parameter("shape")) {
    node->declare_parameter<std::string>("shape", "letter");
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  bool success = false;

  {
    MoveGroup group(node, "ur_manipulator");
    group.setPoseReferenceFrame(group.getPlanningFrame());
    group.setEndEffectorLink("tool0");
    group.setMaxVelocityScalingFactor(0.05);
    group.setMaxAccelerationScalingFactor(0.05);

    const auto start = group.getCurrentPose("tool0").pose;
    RCLCPP_INFO(node->get_logger(),
                "Bat dau tai x=%.3f, y=%.3f, z=%.3f",
                start.position.x, start.position.y, start.position.z);

    const auto shape = node->get_parameter("shape").as_string();

    if (shape == "circle") {
      success = drawCircle(group, start, node->get_logger());
    } else if (shape == "letter") {
      success = drawLetterT(group, start, node->get_logger());
    } else {
      RCLCPP_ERROR(node->get_logger(),
                   "shape phai la 'letter' hoac 'circle'.");
    }

    RCLCPP_INFO(node->get_logger(), "%s",
                success ? "Hoan thanh." : "Chua hoan thanh.");
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
