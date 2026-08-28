"""Gazebo (gz-sim) bringup for the dual-arm KUKA iiwa7 rig with the pdz gripper: spawns the
robot (lbr_dual_arm.xacro mode:=gazebo gripper:=pdz), the plumbers_block fixture + 5 parts at
their logs/plumbers_block_sim pickup poses, and the controllers gz_ros2_control creates in the
ONE combined controller_manager (see lbr_dual_arm.xacro's spawn_gazebo_plugin/
xacro:lbr_gazebo_plugin: gripper:=pdz collapses to a single combined gz_ros2_control plugin
instance, namespace /<robot_name>, instead of one plugin instance per arm -- each per-arm
instance was found to claim every <ros2_control> hardware component in the whole model
regardless of which arm "owns" it, see dual_arm_pdz_gazebo_controllers.yaml's header comment).

arm_control (launch arg) selects the arm controllers:
  joint_trajectory (default) -- two position JointTrajectoryControllers
    (joint_trajectory_controller_lbr_one/_two), replayed by plan_executor_node as before.
  cartesian_impedance -- two effort-interface cartesian_impedance_controller instances
    (cartesian_impedance_lbr_one/_two) + two per-arm pdz gripper_controller_lbr_one/_two,
    see dual_arm_gazebo_cartesian_impedance_controllers.yaml. Run plan_executor_node with
    --arm-control cartesian_impedance to replay against these.

Modeled on lbr_bringup/launch/gazebo.launch.py (GazeboMixin, single controller_manager namespace
pattern) and lbr_dual_arm_bringup/launch/mock.launch.py (robot_description/RSP construction), see
the approved plan at ~/.claude/plans/wise-prancing-cocke.md, Phase 4, and the controller-
restructure plan at ~/.claude/plans/foamy-booping-prism.md section 4.
"""
import json
import os

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from lbr_bringup.gazebo import GazeboMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # gz-sim resolves a spawned model's package://<pkg>/<rel> mesh URIs as model://<pkg>/<rel>,
    # which needs GZ_SIM_RESOURCE_PATH to include the *parent* of a directory literally named
    # <pkg> (ament's install/<pkg>/share/<pkg>/... layout gives exactly that at
    # install/<pkg>/share). The workspace's default GZ_SIM_RESOURCE_PATH only covers
    # lbr_description -- add the three packages this bringup introduces.
    for pkg in ("lbr_dual_arm_description", "pdz_gripper_description", "plumbers_block_description"):
        ld.add_action(
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", os.path.dirname(get_package_share_directory(pkg))
            )
        )

    ld.add_action(
        DeclareLaunchArgument(
            name="robot_name",
            default_value="lbr_dual_arm_pdz",
            description="Namespace for robot_state_publisher / the spawned model / the single"
            " gz_ros2_control plugin's controller_manager (lbr_dual_arm.xacro's own robot_name"
            " arg must match -- passed through below). Deliberately NOT 'lbr_dual_arm' -- this"
            " workstation has long-running, unrelated processes already publishing"
            " robot_state_publisher under that exact namespace (predating this bringup), and"
            " ros_gz_sim create's robot_description topic subscription collided with theirs,"
            " silently spawning their (Y-gripper) robot instead of this package's pdz one."
            " Confirmed via `ign model -m lbr_dual_arm -j` showing lbr_one_left_finger_joint"
            " (Y-gripper) even though gz_ros2_control's own hardware-component scan (a separate"
            " ROS parameter read, not a topic subscription) correctly saw the pdz ros2_control"
            " blocks -- a genuine split-brain, not a bug in lbr_dual_arm.xacro itself.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="arm_control",
            default_value="joint_trajectory",
            choices=["joint_trajectory", "cartesian_impedance"],
            description="joint_trajectory (default): both arms on position-interface"
            " JointTrajectoryControllers (joint_trajectory_controller_lbr_one/_two), replayed"
            " by plan_executor_node as before. cartesian_impedance: both arms on the"
            " effort-interface cartesian_impedance_controller (cartesian_impedance_lbr_one/_two"
            " + per-arm pdz gripper_controller_lbr_one/_two), see"
            " dual_arm_gazebo_cartesian_impedance_controllers.yaml. Passed through to"
            " lbr_dual_arm.xacro (which also forces the arm joints to an effort-only command"
            " interface) -- run plan_executor_node with a matching --arm-control.",
        )
    )

    # Optionally start the unified plan_executor_node in-launch (default: off -- the operator
    # usually watches the sim come up first, then runs it by hand). When on, it is started with
    # the Gazebo-side args so the ONLY difference vs a real/mock run is which bringup launch
    # ran: --controller-topology per-arm, this launch's robot_name / arm_control,
    # --gripper-backend sim, --visualize-held-parts, --use-sim-time, --on-failure continue.
    ld.add_action(
        DeclareLaunchArgument(
            name="run_executor",
            default_value="false",
            description="Start plan_executor_node from this launch once the sim is up"
            " (TimerAction-delayed). Off by default.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="gripper_backend",
            default_value="sim",
            choices=["none", "sim", "servo"],
            description="Passed to plan_executor_node --gripper-backend when run_executor:=true."
            " sim (default) drives the simulated pdz finger joint.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="log_dir",
            default_value="",
            description="motion.pkl / traj.npy directory for plan_executor_node when"
            " run_executor:=true (empty = the node's own default,"
            " ~/Fabrica/logs/plumbers_block_sim).",
        )
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution(
                        [FindPackageShare("lbr_dual_arm_description"), "urdf", "lbr_dual_arm.xacro"]
                    ),
                    " mode:=gazebo",
                    " gripper:=pdz",
                    " use_gripper:=true",
                    " arm_control:=", LaunchConfiguration("arm_control"),
                    " robot_name:=", LaunchConfiguration("robot_name"),
                ]
            ),
            value_type=str,
        )
    }

    # Single robot_state_publisher under /<robot_name> -- matches the single gz_ros2_control
    # plugin instance's own namespace, so its default (gazebo_robot_param_node="") relative
    # "robot_state_publisher" lookup resolves without any per-arm duplication.
    ld.add_action(
        LBRROS2ControlMixin.node_robot_state_publisher(
            robot_description=robot_description,
            robot_name=LaunchConfiguration("robot_name"),
            use_sim_time=True,
        )
    )

    world_path = os.path.join(
        get_package_share_directory("lbr_dual_arm_pdz_bringup"), "worlds", "plumbers_block.sdf"
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
            ),
            launch_arguments={"gz_args": f"-r {world_path}"}.items(),
        )
    )
    ld.add_action(GazeboMixin.node_clock_bridge())

    spawn_robot = GazeboMixin.node_create(robot_name=LaunchConfiguration("robot_name"))
    ld.add_action(spawn_robot)

    # Fixture + 5 parts, at their logs/plumbers_block_sim pickup poses (see
    # planning/utils/generate_plumbers_block_gz_assets.py in the Fabrica repo, which produced
    # both the models/ this reads and spawn_poses.json).
    plumbers_block_share = get_package_share_directory("plumbers_block_description")
    with open(os.path.join(plumbers_block_share, "spawn_poses.json")) as f:
        spawn_poses = json.load(f)

    # spawn_poses.json z's are in the dual-arm board frame whose origin (base_link, where the
    # robot spawns at world z=0) is the two KUKA bases' mounting plane -- 2.5cm ABOVE the table
    # surface, since the real rig sits both bases on a ~2.5cm riser block
    # (get_kuka_mount_block_height() in Fabrica's planning/robot/workcell.py; same convention as
    # lbr_dual_arm_bringup/config/mock_scene_objects.yaml). worlds/plumbers_block.sdf's "table"
    # model has its top face at z=-0.025 to match, so drop every spawned asset by this riser
    # height to seat it on the table rather than leaving it floating 2.5cm above.
    TABLE_RISER_M = 0.025

    spawn_asset_nodes = []
    for model_name, pose in spawn_poses.items():
        # TEMP diagnostic (2026-08-25): skip the fixture (heaviest real-mesh collision, main RTF
        # cost) so parts+arms can be tested in isolation with gravity off. Revert once confirmed.
        if model_name == "plumbers_block_fixture":
            continue
        model_sdf = os.path.join(plumbers_block_share, "models", model_name, "model.sdf")
        x, y, z = pose["pos"]
        z -= TABLE_RISER_M
        roll, pitch, yaw = pose["rpy"]
        spawn_asset_nodes.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-file", model_sdf,
                    "-name", model_name,
                    "-x", str(x), "-y", str(y), "-z", str(z),
                    "-R", str(roll), "-P", str(pitch), "-Y", str(yaw),
                    "-allow_renaming", "false",
                ],
                output="screen",
            )
        )

    # Spawn the fixture/parts once the robot is in, to avoid every `create` process racing gz-sim
    # startup at once.
    ld.add_action(
        RegisterEventHandler(
            OnProcessStart(target_action=spawn_robot, on_start=spawn_asset_nodes)
        )
    )

    # gz_ros2_control brings up its own controller_manager under /<robot_name> as soon as the
    # single <gazebo><plugin> activates -- there is no separate ros2_control_node to start
    # (unlike mock/hardware mode). Which controllers to spawn depends on arm_control, resolved
    # here via OpaqueFunction since it's a LaunchConfiguration.
    ld.add_action(OpaqueFunction(function=_spawn_controllers))

    ld.add_action(OpaqueFunction(function=_maybe_run_executor))

    return ld


def _maybe_run_executor(context):
    """Start plan_executor_node from this launch iff run_executor:=true. Resolved here (not a
    plain IfCondition Node) so log_dir can be omitted when empty and so the Gazebo-side args
    are assembled in one place. TimerAction-delayed to let the gz controllers spawn first --
    the node still bounded-waits on its action server, this just avoids the first-try timeout."""
    if LaunchConfiguration("run_executor").perform(context).lower() not in ("true", "1"):
        return []

    args = [
        "--controller-topology", "per-arm",
        "--robot-name", LaunchConfiguration("robot_name").perform(context),
        "--arm-control", LaunchConfiguration("arm_control").perform(context),
        "--gripper-backend", LaunchConfiguration("gripper_backend").perform(context),
        "--world", "plumbers_block",
        "--visualize-held-parts",
        "--use-sim-time",
        "--on-failure", "continue",
    ]
    log_dir = LaunchConfiguration("log_dir").perform(context)
    if log_dir:
        args.insert(0, log_dir)

    return [
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package="lbr_dual_arm_pdz_bringup",
                    executable="plan_executor_node",
                    output="screen",
                    arguments=args,
                )
            ],
        )
    ]


def _spawn_controllers(context):
    """Build the controller-spawner actions for the resolved arm_control value.

    joint_trajectory: one controller_manager, two JointTrajectoryController instances (one per
    arm, see dual_arm_pdz_gazebo_controllers.yaml), each claiming only its own arm's joints --
    spawned directly, matching lbr_bringup/launch/gazebo.launch.py's simplicity.

    cartesian_impedance: joint_state_broadcaster, then cartesian_impedance_lbr_one/_two, then
    per-arm gripper_controller_lbr_one/_two (see
    dual_arm_gazebo_cartesian_impedance_controllers.yaml). Spawned sequentially via OnProcessExit
    -- five controllers racing the controller_manager during startup is the same failure mode
    lbr_dual_arm_bringup/launch/cartesian_impedance.launch.py sequences around.
    """
    robot_name = LaunchConfiguration("robot_name")
    arm_control = LaunchConfiguration("arm_control").perform(context)

    def spawner(controller):
        return LBRROS2ControlMixin.node_controller_spawner(
            robot_name=robot_name, controller=controller
        )

    if arm_control == "joint_trajectory":
        return [
            spawner("joint_state_broadcaster"),
            spawner("joint_trajectory_controller_lbr_one"),
            spawner("joint_trajectory_controller_lbr_two"),
        ]

    # cartesian_impedance -- sequential startup
    chain = [
        "joint_state_broadcaster",
        "cartesian_impedance_lbr_one",
        "cartesian_impedance_lbr_two",
        "gripper_controller_lbr_one",
        "gripper_controller_lbr_two",
    ]
    spawners = [spawner(name) for name in chain]
    actions = [spawners[0]]
    for prev, nxt in zip(spawners, spawners[1:]):
        actions.append(
            RegisterEventHandler(OnProcessExit(target_action=prev, on_exit=[nxt]))
        )
    return actions
