#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "safe_pose",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  bool success = false;

  {
    moveit::planning_interface::MoveGroupInterface move_group(
      node, "ur_manipulator");

    move_group.setPlanningTime(10.0);
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);
    move_group.setStartStateToCurrentState();

    // Tư thế gập nhẹ để tránh bắt đầu ở q3≈0 và q5≈0.
    const std::vector<double> target_joints{
      0.0, -1.0, -1.0, -1.2, -1.0, 0.0
    };

    if (!move_group.setJointValueTarget(target_joints)) {
      RCLCPP_ERROR(node->get_logger(), "Khong chap nhan duoc joint target.");
    } else {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto plan_result = move_group.plan(plan);

      if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Da lap duoc joint-space plan.");
        const auto execute_result = move_group.execute(plan);

        if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(node->get_logger(), "Da den tu the moi.");
          const auto pose = move_group.getCurrentPose("tool0");
          RCLCPP_INFO(node->get_logger(),
                      "Pose moi: x=%.3f, y=%.3f, z=%.3f",
                      pose.pose.position.x,
                      pose.pose.position.y,
                      pose.pose.position.z);
          success = true;
        } else {
          RCLCPP_ERROR(node->get_logger(), "Thuc thi joint-space plan that bai.");
        }
      } else {
        RCLCPP_ERROR(node->get_logger(), "MoveIt khong lap duoc joint-space plan.");
      }
    }
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
