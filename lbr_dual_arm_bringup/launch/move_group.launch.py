import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, add_debuggable_node

# Absolute path to this package's own source tree, so its plain (not
# ROS-installed) scripts/publish_mock_scene_objects.py and
# config/mock_scene_objects.yaml can be found regardless of which directory
# `ros2 launch` was invoked from -- lbr_dual_arm_bringup is ament_cmake and
# only installs doc/launch (see CMakeLists.txt), so these live purely in
# source, same reasoning as mock.launch.py's MASTERTHESIS_VISION_DIR.
LBR_DUAL_ARM_BRINGUP_SRC_DIR = (
    "/home/pdzuser/franka_ros2_ws/src/lbr_fri_ros2_stack/lbr_demos/lbr_dual_arm/lbr_dual_arm_bringup"
)


def launch_setup(context, *args, **kwargs):
    # Resolve control_mode before constructing MoveItConfigsBuilder because
    # trajectory_execution() requires a concrete YAML file path.
    control_mode = LaunchConfiguration("control_mode").perform(context)
    mode = LaunchConfiguration("mode").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)

    moveit_config_dir = os.path.join(
        get_package_share_directory("lbr_dual_arm_moveit_config"),
        "config",
    )

    # control_mode is independent of the `mode` arg (mock vs. hardware URDF).
    # Empty (the default) means joint_trajectory_controller, whose config is
    # plain moveit_controllers.yaml with no infix; any other value selects
    # moveit_<control_mode>_controllers.yaml (cartesian_impedance, ...).
    controllers_filename = (
        f"moveit_{control_mode}_controllers.yaml" if control_mode else "moveit_controllers.yaml"
    )
    controllers_file = os.path.join(moveit_config_dir, controllers_filename)

    moveit_config_builder = (
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
            mappings={
                "use_gripper": LaunchConfiguration("use_gripper"),
            },
        )
        .trajectory_execution(
            file_path=controllers_file,
        )
    )

    moveit_config = moveit_config_builder.to_moveit_configs()

    # move_group node -- this inlines moveit_configs_utils'
    # generate_move_group_launch() (moveit_configs_utils/launches.py) rather
    # than calling it, purely so ros_arguments can be passed to the Node (that
    # helper exposes no hook for it). Keep the parameter/flag list in sync with
    # upstream on MoveIt upgrades; the only intentional addition is the
    # ros_arguments log-level below.
    move_group_ld = LaunchDescription()
    move_group_ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    move_group_ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    move_group_ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    move_group_ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        )
    )
    move_group_ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities["disable_capabilities"],
        )
    )
    move_group_ld.add_action(
        DeclareBooleanLaunchArg("monitor_dynamics", default_value=False)
    )
    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }
    move_group_params = [moveit_config.to_dict(), move_group_configuration]

    # mode:=gazebo: lbr_dual_arm_pdz_bringup's joint_state_broadcaster publishes
    # the two pdz finger joints (lbr_{one,two}_pdz_gripper_left_finger_joint),
    # which move_group's use_gripper:=false robot model does not contain -- so
    # CurrentStateMonitor logs "Joint '...' not found in model" at ERROR on
    # every /joint_states message (~50 Hz). It is harmless (all 14 arm joints
    # are present; planning and execution work), but it floods the console.
    # Raise just that one logger's threshold for the gz-sim path; mock/hardware
    # are left completely untouched (ros_arguments stays None).
    move_group_ros_arguments = None
    if mode == "gazebo":
        move_group_ros_arguments = [
            "--log-level",
            "moveit_robot_model.robot_model:=FATAL",
        ]

    add_debuggable_node(
        move_group_ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
        ros_arguments=move_group_ros_arguments,
    )

    # move_group must run under the same namespace as robot_state_publisher
    # and ros2_control_node (see hardware.launch.py/mock.launch.py) so its
    # default topic/action names (joint_states, the controller's
    # follow_joint_trajectory action, ...) resolve to the running hardware
    # instead of an empty global namespace.
    entities = [
        GroupAction(
            [
                PushRosNamespace(robot_name),
                # When driving a gz-sim rig (mode:=gazebo,
                # lbr_dual_arm_pdz_bringup/launch/gazebo.launch.py), move_group
                # must run on the Gazebo /clock: otherwise it stamps
                # trajectories and evaluates goal-time tolerance against wall
                # time while cartesian_impedance_lbr_one/_two run on sim time,
                # and every execution aborts as GOAL_TOLERANCE_VIOLATED.
                # Stays false (wall time) for mock/hardware.
                SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
                *move_group_ld.entities,
            ]
        )
    ]

    if mode == "mock":
        # Publishes the static mock-scene box(es) from
        # config/mock_scene_objects.yaml (e.g. a box resting on the table)
        # as CollisionObjects, for collision-aware planning against
        # move_group's mock scene only -- not started for hardware mode,
        # where any such object should reflect what's actually on the real
        # table. This is a plain script (see its own docstring), not a
        # launch_ros Node, so it doesn't automatically inherit the
        # PushRosNamespace above -- pass the namespace explicitly so its
        # relative "planning_scene" topic still resolves to the same
        # /<robot_name>/planning_scene move_group itself listens on.
        entities.append(
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(LBR_DUAL_ARM_BRINGUP_SRC_DIR, "scripts", "publish_mock_scene_objects.py"),
                    "--config",
                    os.path.join(LBR_DUAL_ARM_BRINGUP_SRC_DIR, "config", "mock_scene_objects.yaml"),
                    "--ros-args", "-r", f"__ns:=/{robot_name}",
                ],
                output="screen",
            )
        )

    return entities


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            name="mode",
            default_value="mock",
            description="Dual-arm description mode.",
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

    ld.add_action(
        DeclareLaunchArgument(
            name="robot_name",
            default_value="lbr_dual_arm",
            description="Namespace for move_group, matching hardware.launch.py/mock.launch.py.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="control_mode",
            default_value="",
            choices=[
                "",
                "cartesian_impedance",
            ],
            description=(
                "Select the MoveIt controller configuration. Empty (default) "
                "loads moveit_controllers.yaml (joint_trajectory_controller); "
                "otherwise loads moveit_<control_mode>_controllers.yaml."
            ),
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="rviz",
            default_value="true",
            description="Launch RViz alongside move_group.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="false",
            description=(
                "Run move_group and RViz on the Gazebo /clock. Set true "
                "together with mode:=gazebo and robot_name:=lbr_dual_arm_pdz "
                "to drive lbr_dual_arm_pdz_bringup's gz-sim rig; leave false "
                "for mock/hardware."
            ),
        )
    )

    ld.add_action(
        OpaqueFunction(function=launch_setup)
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("lbr_dual_arm_moveit_config"),
                        "launch",
                        "moveit_rviz.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "mode": LaunchConfiguration("mode"),
                "use_gripper": LaunchConfiguration("use_gripper"),
                "robot_name": LaunchConfiguration("robot_name"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("rviz")),
        )
    )

    return ld
