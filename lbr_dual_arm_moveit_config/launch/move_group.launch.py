import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def launch_setup(context, *args, **kwargs):
    # Which MoveIt controller-manager config to load is a choice that has
    # to be made while building the MoveItConfigsBuilder in Python (it takes
    # a concrete file path, not a launch-time substitution) -- hence
    # OpaqueFunction, so the launch argument is resolved before this runs.
    use_cartesian_impedance = (
        LaunchConfiguration("use_cartesian_impedance").perform(context).lower() == "true"
    )

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
            mappings={"use_gripper": LaunchConfiguration("use_gripper")},
        )
    )

    # Depending on the launch: joint_trajectory_controller (position-mode,
    # moveit_controllers.yaml, picked automatically) vs.
    # cartesian_impedance_lbr_one/_two (torque-mode,
    # cartesian_impedance_controllers.yaml, see
    # lbr_dual_arm_bringup/launch/cartesian_impedance.launch.py).
    if use_cartesian_impedance:
        moveit_config_builder.trajectory_execution(
            file_path="config/cartesian_impedance_controllers.yaml"
        )

    moveit_config = moveit_config_builder.to_moveit_configs()

    return generate_move_group_launch(moveit_config).entities


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("mode", default_value="mock"))
    ld.add_action(DeclareLaunchArgument("use_gripper", default_value="true"))
    ld.add_action(
        DeclareLaunchArgument(
            name="use_cartesian_impedance",
            default_value="false",
            description=(
                "Use cartesian_impedance_lbr_one/_two's FollowJointTrajectory "
                "action (cartesian_impedance_controllers.yaml) as MoveIt's "
                "execution backend instead of joint_trajectory_controller. "
                "Only meaningful together with lbr_dual_arm_bringup's "
                "cartesian_impedance.launch.py."
            ),
        )
    )
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
