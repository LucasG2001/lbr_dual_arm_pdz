"""Cartesian-impedance bring-up -- torque-mode twin of hardware.launch.py.

Brings both arms up with cartesian_impedance_lbr_one/_two active (instead of
joint_trajectory_controller) so a client can drive either arm's flange with
a compliant, task-space PD response and runtime-adjustable gains.

Controllers are spawned sequentially to avoid controller_manager startup
races:

    ros2_control_node
        -> joint_state_broadcaster
        -> cartesian_impedance_lbr_one
        -> cartesian_impedance_lbr_two
        -> camera_scene_publisher
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from lbr_bringup.ros2_control import LBRROS2ControlMixin


# Absolute path to the Masterthesis-vision repo, so
# publish_camera_scene_objects can be run as a Python module regardless of
# which directory `ros2 launch` was invoked from.
MASTERTHESIS_VISION_DIR = "/home/pdzuser/Masterthesis-vision"


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
            description=(
                "Attach the Y-gripper to each arm's flange. "
                "false = bare flange, no end effector."
            ),
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
                    " use_gripper:=",
                    LaunchConfiguration("use_gripper"),
                ]
            ),
            value_type=str,
        )
    }

    # -------------------------------------------------------------------------
    # Robot State Publisher
    # -------------------------------------------------------------------------

    ld.add_action(
        LBRROS2ControlMixin.node_robot_state_publisher(
            robot_description=robot_description,
            robot_name=LaunchConfiguration("robot_name"),
            use_sim_time=False,
        )
    )

    # -------------------------------------------------------------------------
    # ros2_control / Controller Manager
    # -------------------------------------------------------------------------

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"use_sim_time": False},
            PathJoinSubstitution(
                [
                    FindPackageShare("lbr_dual_arm_description"),
                    "ros2_control",
                    "dual_arm_cartesian_impedance_controllers.yaml",
                ]
            ),
            robot_description,
        ],
        namespace=LaunchConfiguration("robot_name"),
        remappings=[("~/robot_description", "robot_description")],
    )

    ld.add_action(ros2_control_node)

    # -------------------------------------------------------------------------
    # Controller Spawners
    # -------------------------------------------------------------------------

    joint_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        robot_name=LaunchConfiguration("robot_name"),
        controller="joint_state_broadcaster",
    )

    cartesian_impedance_lbr_one = LBRROS2ControlMixin.node_controller_spawner(
        robot_name=LaunchConfiguration("robot_name"),
        controller="cartesian_impedance_lbr_one",
    )

    cartesian_impedance_lbr_two = LBRROS2ControlMixin.node_controller_spawner(
        robot_name=LaunchConfiguration("robot_name"),
        controller="cartesian_impedance_lbr_two",
    )

    # -------------------------------------------------------------------------
    # Sequential controller startup
    #
    # Do NOT spawn all controllers simultaneously.
    #
    # ros2_control_node
    #       |
    #       v
    # joint_state_broadcaster
    #       |
    #       | spawner exits successfully
    #       v
    # cartesian_impedance_lbr_one
    #       |
    #       | spawner exits successfully
    #       v
    # cartesian_impedance_lbr_two
    #
    # This prevents multiple spawners from racing controller_manager while it
    # is still initializing.
    # -------------------------------------------------------------------------

    ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action=ros2_control_node,
                on_start=[joint_state_broadcaster],
            )
        )
    )

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[cartesian_impedance_lbr_one],
            )
        )
    )

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=cartesian_impedance_lbr_one,
                on_exit=[cartesian_impedance_lbr_two],
            )
        )
    )

    # -------------------------------------------------------------------------
    # Camera scene publisher
    #
    # Start this only after BOTH impedance controllers have finished spawning,
    # so it does not compete with controller_manager during startup.
    # -------------------------------------------------------------------------

    camera_scene_publisher = ExecuteProcess(
        cmd=[
            "python3",
            "-m",
            "src.calibration.publish_camera_scene_objects",
            "--dual-arm",
        ],
        cwd=MASTERTHESIS_VISION_DIR,
        output="screen",
    )

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=cartesian_impedance_lbr_two,
                on_exit=[camera_scene_publisher],
            )
        )
    )

    return ld
