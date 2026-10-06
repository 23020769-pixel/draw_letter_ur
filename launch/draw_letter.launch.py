from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    set_tolerance = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "for joint in shoulder_pan_joint shoulder_lift_joint elbow_joint "
            "wrist_1_joint wrist_2_joint wrist_3_joint; do "
            "ros2 param set /joint_trajectory_controller "
            "constraints.${joint}.trajectory 0.25 || exit 1; done",
        ],
        output="screen",
    )

    safe_pose = Node(
        package="draw_letter_ur",
        executable="safe_pose",
        name="safe_pose",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    draw_t = Node(
        package="draw_letter_ur",
        executable="draw_letter",
        name="draw_letter",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "shape": "letter",
            }
        ],
    )

    def after_tolerance(event, context):
        if event.returncode == 0:
            return [LogInfo(msg="Da dat tolerance."), safe_pose]
        return [LogInfo(msg="Khong dat duoc tolerance; dung lai.")]

    def after_safe_pose(event, context):
        if event.returncode == 0:
            return [LogInfo(msg="safe_pose thanh cong; bat dau ve chu T."), draw_t]
        return [LogInfo(msg="safe_pose that bai; khong chay node ve.")]

    return LaunchDescription(
        [
            RegisterEventHandler(
                OnProcessExit(
                    target_action=set_tolerance,
                    on_exit=after_tolerance,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=safe_pose,
                    on_exit=after_safe_pose,
                )
            ),
            set_tolerance,
        ]
    )
