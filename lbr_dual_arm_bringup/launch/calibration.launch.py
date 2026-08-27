"""Hand-guided calibration bring-up -- torque-mode twin of hardware.launch.py.

Brings both arms up in gravity-compensation mode (command_mode:=torque,
gravity_compensation_lbr_one/_lbr_two instead of joint_trajectory_controller)
so the operator can hand-guide either arm to a pose and let go, instead of
sending MoveGroup goals. See
src/calibration/capture_flange_poses_dual_handguided.py in Masterthesis-vision
for the capture script that expects this launch file already running.

use_gripper defaults to true (matching hardware.launch.py/mock.launch.py) --
gravity_compensation's torque term accounts for the Y-gripper's mass via
y_gripper.xacro's <inertial> tags, but not for any camera/mount bolted on
beyond that.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            name="robot_name",
            default_value="lbr_dual_arm",
            description="Namespace for the dual-arm bringup nodes.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="use_gripper",
            default_value="true",
            description="Attach the Y-gripper to each arm's flange. false = bare flange, no end effector.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="arms",
            default_value="both",
            description="Which arm(s) to bring up as real FRI hardware for hand-guided "
            "calibration. The other arm (if any) is loaded as a mock component instead, so "
            "ros2_control_node doesn't block forever in on_activate waiting for a robot "
            "controller that isn't connected, and its gravity_compensation controller is not "
            "spawned.",
            choices=["both", "lbr_one", "lbr_two"],
        )
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("lbr_dual_arm_description"),
                            "urdf",
                            "lbr_dual_arm.xacro",
                        ]
                    ),
                    " mode:=hardware",
                    " command_mode:=torque",
                    " arms:=",
                    LaunchConfiguration("arms"),
                    " use_gripper:=",
                    LaunchConfiguration("use_gripper"),
                ]
            ),
            value_type=str,
        )
    }

    ld.add_action(
        LBRROS2ControlMixin.node_robot_state_publisher(
            robot_description=robot_description,
            robot_name=LaunchConfiguration("robot_name"),
            use_sim_time=False,
        )
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"use_sim_time": False},
            PathJoinSubstitution(
                [
                    FindPackageShare("lbr_dual_arm_description"),
                    "ros2_control",
                    "dual_arm_gravity_compensation_controllers.yaml",
                ]
            ),
            robot_description,
        ],
        namespace=LaunchConfiguration("robot_name"),
        remappings=[("~/robot_description", "robot_description")],
    )
    ld.add_action(ros2_control_node)

    joint_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        robot_name=LaunchConfiguration("robot_name"),
        controller="joint_state_broadcaster",
    )
    gravity_compensation_lbr_one = LBRROS2ControlMixin.node_controller_spawner(
        robot_name=LaunchConfiguration("robot_name"),
        controller="gravity_compensation_lbr_one",
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("arms"), "' in ['both', 'lbr_one']"]
            )
        ),
    )
    gravity_compensation_lbr_two = LBRROS2ControlMixin.node_controller_spawner(
        robot_name=LaunchConfiguration("robot_name"),
        controller="gravity_compensation_lbr_two",
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("arms"), "' in ['both', 'lbr_two']"]
            )
        ),
    )

    ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action=ros2_control_node,
                on_start=[
                    joint_state_broadcaster,
                    gravity_compensation_lbr_one,
                    gravity_compensation_lbr_two,
                ],
            )
        )
    )
    return ld
