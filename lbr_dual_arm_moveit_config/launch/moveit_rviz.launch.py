import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, add_debuggable_node


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("mode", default_value="mock"))
    ld.add_action(DeclareLaunchArgument("use_gripper", default_value="true"))
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description=(
                "Run RViz on the Gazebo /clock -- set true when move_group is "
                "launched with use_sim_time:=true against a gz-sim rig, or "
                "RViz's TF display drops out on the wall/sim clock mismatch."
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="robot_name",
            default_value="lbr_dual_arm",
            description=(
                "Namespace robot_state_publisher/move_group publish their "
                "topics under. RViz itself stays unnamespaced, so the "
                "topics it needs (robot_description, joint_states, "
                "planning_scene, ...) are individually remapped into this "
                "namespace to match."
            ),
        )
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "lbr_dual_arm",
            package_name="lbr_dual_arm_moveit_config",
        )
        .robot_description(
            os.path.join(
                get_package_share_directory("lbr_dual_arm_description"),
                "urdf/lbr_dual_arm.xacro",
            ),
            mappings={
                "mode": LaunchConfiguration("mode"),
                "use_gripper": LaunchConfiguration("use_gripper"),
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory("lbr_dual_arm_moveit_config"),
                "config",
                "lbr_dual_arm.srdf.xacro",
            ),
            mappings={"use_gripper": LaunchConfiguration("use_gripper")},
        )
        .to_moveit_configs()
    )

    robot_name = LaunchConfiguration("robot_name")

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("robot_description", PathJoinSubstitution([robot_name, "robot_description"])),
            (
                "robot_description_semantic",
                PathJoinSubstitution([robot_name, "robot_description_semantic"]),
            ),
            ("joint_states", PathJoinSubstitution([robot_name, "joint_states"])),
            ("planning_scene", PathJoinSubstitution([robot_name, "planning_scene"])),
            (
                "monitored_planning_scene",
                PathJoinSubstitution([robot_name, "monitored_planning_scene"]),
            ),
            (
                "planning_scene_world",
                PathJoinSubstitution([robot_name, "planning_scene_world"]),
            ),
            (
                "display_planned_path",
                PathJoinSubstitution([robot_name, "display_planned_path"]),
            ),
            (
                "recognized_object_array",
                PathJoinSubstitution([robot_name, "recognized_object_array"]),
            ),
        ],
    )
    return ld
