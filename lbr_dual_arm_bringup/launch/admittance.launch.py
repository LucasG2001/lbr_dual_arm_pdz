"""Software-admittance bring-up -- position-mode sibling of hardware.launch.py.

Brings both arms up with, per arm, lbr_state_broadcaster_lbr_{one,two}
(publishes LBRState incl. external_torque) and
lbr_joint_position_command_controller_lbr_{one,two} (position setpoint
streaming) active instead of joint_trajectory_controller. See
src/calibration/admittance_dual_arm.py in Masterthesis-vision for the client
that runs the actual admittance law against this bring-up (reuses
lbr_demos_advanced_py's AdmittanceController per arm) and publishes to each
arm's command/joint_position topic.

Stays in POSITION command mode throughout (mode:=hardware, no
command_mode:=torque) -- no torque command interface is used anywhere in
this bring-up, matching hardware.launch.py's default system config.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
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

# Absolute path to the Masterthesis-vision repo, so publish_camera_scene_objects
# (a plain python module, not an installed ROS package) can be run as
# `python3 -m ...` regardless of which directory `ros2 launch` was invoked from.
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
            description="Attach the Y-gripper to each arm's flange. false = bare flange, no end effector.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="arms",
            default_value="both",
            description="Which arm(s) to bring up as real FRI hardware. The other arm (if any) "
            "is loaded as a mock component instead, so ros2_control_node doesn't block forever "
            "in on_activate waiting for a robot controller that isn't connected, and its "
            "lbr_state_broadcaster/lbr_joint_position_command_controller are not spawned.",
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
                    "dual_arm_admittance_controllers.yaml",
                ]
            ),
            robot_description,
        ],
        namespace=LaunchConfiguration("robot_name"),
        remappings=[
            ("~/robot_description", "robot_description"),
            # lbr_state_broadcaster.cpp creates its "state" publisher with a
            # plain (non-private) topic name, so it resolves against the
            # shared /lbr_dual_arm namespace rather than each controller
            # instance's own name -- both arms' broadcasters collide on
            # /lbr_dual_arm/state without these node-scoped remaps. Matches
            # what admittance_dual_arm.py's ARM_KEYS already expects
            # (lbr_state_broadcaster_lbr_{one,two}/state).
            ("lbr_state_broadcaster_lbr_one:state", "lbr_state_broadcaster_lbr_one/state"),
            ("lbr_state_broadcaster_lbr_two:state", "lbr_state_broadcaster_lbr_two/state"),
        ],
    )
    ld.add_action(ros2_control_node)

    joint_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        robot_name=LaunchConfiguration("robot_name"),
        controller="joint_state_broadcaster",
    )
    lbr_one_active = IfCondition(
        PythonExpression(["'", LaunchConfiguration("arms"), "' in ['both', 'lbr_one']"])
    )
    lbr_two_active = IfCondition(
        PythonExpression(["'", LaunchConfiguration("arms"), "' in ['both', 'lbr_two']"])
    )
    lbr_state_broadcaster_lbr_one = LBRROS2ControlMixin.node_controller_spawner(
        robot_name=LaunchConfiguration("robot_name"),
        controller="lbr_state_broadcaster_lbr_one",
        condition=lbr_one_active,
    )
    lbr_state_broadcaster_lbr_two = LBRROS2ControlMixin.node_controller_spawner(
        robot_name=LaunchConfiguration("robot_name"),
        controller="lbr_state_broadcaster_lbr_two",
        condition=lbr_two_active,
    )
    lbr_joint_position_command_controller_lbr_one = (
        LBRROS2ControlMixin.node_controller_spawner(
            robot_name=LaunchConfiguration("robot_name"),
            controller="lbr_joint_position_command_controller_lbr_one",
            condition=lbr_one_active,
        )
    )
    lbr_joint_position_command_controller_lbr_two = (
        LBRROS2ControlMixin.node_controller_spawner(
            robot_name=LaunchConfiguration("robot_name"),
            controller="lbr_joint_position_command_controller_lbr_two",
            condition=lbr_two_active,
        )
    )

    ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action=ros2_control_node,
                on_start=[
                    joint_state_broadcaster,
                    lbr_state_broadcaster_lbr_one,
                    lbr_state_broadcaster_lbr_two,
                    lbr_joint_position_command_controller_lbr_one,
                    lbr_joint_position_command_controller_lbr_two,
                ],
            )
        )
    )

    # Publishes the ZED cameras + their mounting holders as MoveIt
    # CollisionObjects on /planning_scene (see
    # src/calibration/publish_camera_scene_objects.py in Masterthesis-vision),
    # so move_group's collision checking knows about the camera rig -- not
    # started by calibration.launch.py, since that's the procedure that
    # produces the extrinsics this publisher reads.
    ld.add_action(
        ExecuteProcess(
            cmd=["python3", "-m", "src.calibration.publish_camera_scene_objects", "--dual-arm"],
            cwd=MASTERTHESIS_VISION_DIR,
            output="screen",
        )
    )
    return ld
