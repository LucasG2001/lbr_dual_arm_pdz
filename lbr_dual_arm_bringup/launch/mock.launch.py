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

# Absolute path to the Masterthesis-vision repo, so publish_camera_scene_objects
# (a plain python module, not an installed ROS package) can be run as
# `python3 -m ...` regardless of which directory `ros2 launch` was invoked from.
MASTERTHESIS_VISION_DIR = "/home/pdzuser/Masterthesis-vision"


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            name="ctrl",
            default_value="joint_trajectory_controller",
            description="Desired default controller.",
            choices=["joint_trajectory_controller"],
        )
    )
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
                    " mode:=mock",
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
                    "dual_arm_controllers.yaml",
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
    controller = LBRROS2ControlMixin.node_controller_spawner(
        robot_name=LaunchConfiguration("robot_name"),
        controller=LaunchConfiguration("ctrl"),
    )

    # Spawn joint_state_broadcaster first, then joint_trajectory_controller
    # only once it has actually loaded+activated (spawner process exits) --
    # rather than firing both load_controller calls at controller_manager
    # at once, right as it's coming up. See hardware.launch.py's matching
    # comment for why (CONTROL_FAILED debugging, 2026-08-18) -- kept
    # consistent here even though mock mode is less likely to hit the race.
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
                on_exit=[controller],
            )
        )
    )

    # Publishes the ZED cameras + their mounting holders as MoveIt
    # CollisionObjects on /planning_scene (see
    # src/calibration/publish_camera_scene_objects.py in Masterthesis-vision),
    # so move_group's collision checking knows about the camera rig -- not
    # started by calibration.launch.py, since that's the procedure that
    # produces the extrinsics this publisher reads. Deferred until after the
    # essential controllers above have finished spawning.
    camera_scene_publisher = ExecuteProcess(
        cmd=["python3", "-m", "src.calibration.publish_camera_scene_objects", "--dual-arm"],
        cwd=MASTERTHESIS_VISION_DIR,
        output="screen",
    )
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=controller,
                on_exit=[camera_scene_publisher],
            )
        )
    )
    return ld
