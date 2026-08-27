"""Replays a Fabrica motion plan (planning/run_planning.sh's logs/<exp>/... output, here
logs/plumbers_block_sim) on the dual-arm KUKA + pdz gripper Gazebo rig, via the
FollowJointTrajectory action -- no custom control interfaces.

Pass --arm-control cartesian_impedance (matching gazebo.launch.py's arm_control launch arg) to
replay against the effort-interface cartesian_impedance_controller instead of the position
JointTrajectoryControllers; default is joint_trajectory. See the "Controller topology" note
below.

Design (see the approved plan at ~/.claude/plans/wise-prancing-cocke.md, Phase 5, and its "Key
findings" #1):
  - motion.pkl's 43 entries [motion_type, body_type, value, active_part, description] form ONE
    strict global timeline (verified: only one arm ever moves at a time) -- so this node just
    walks the list in order, sending one goal at a time and blocking on its result. That block/
    goal/feedback/result cycle IS the closed-loop execution the plan asked for; no extra topics
    or actions are introduced.
  - motion_type ('move'/'hold') is a role, not a robot name. hold ~= lbr_one (holds the base part
    steady while assembling), move ~= lbr_two (fetches/places the other parts) -- see
    planning/robot/workcell.py's get_hold_arm_pos/get_move_arm_pos comment for the real-rig
    ground truth this is matched against, and lbr_dual_arm.xacro's base joints (lbr_one at
    y=-0.42, lbr_two at y=+0.42).
  - Arm paths are resampled at a fixed 30 fps (planning/robot/motion_plan_arm.py:
    interpolate_arm_path) -- each waypoint is exactly 1/30 s apart, so trajectory points get
    time_from_start = i / 30.0.
  - There is no reliable, version-independent way to make a Gazebo part "stick" to the gripper
    on demand (DetachableJoint is built for the opposite case: pre-attached, one-shot release).
    Instead this node re-uses logs/plumbers_block_sim/traj.npy -- Fabrica's own rendered
    simulation, which already computed the correct part_i pose for every frame of the whole plan,
    including while carried -- and teleports the held part to match via gz-sim's
    /world/<world>/set_pose service (ros_gz_interfaces/srv/SetEntityPose) at the same 30 Hz
    cadence as the arm trajectories. traj.npy's frame 0 is the pre-command initial state and
    frame (i+1) is the state after motion.pkl's i-th flattened waypoint/command -- verified by
    checking sum(len(waypoints) for arm entries, else 1) == len(traj.npy) - 1.
  - open_ratio -> pdz finger joint position: clamp(open_ratio * 0.032, 0, 0.032) -- 0.032m (=
    PDZ_JAW_TRAVEL=3.2cm per-finger stroke) is the real pdz retargeting formula, recovered from
    the reference branch (julien-pdz/dual-arm-kuka's planning/robot/geometry.py) and confirmed
    against grasps.pkl's stored open_ratio values -- see kuka_pdz_executor_node_handoff.md's
    "OPEN TODO" for the derivation. (Supersedes an earlier, first-principles 0.04 guess that was
    never cross-checked and made grasps ~20% narrower than intended below open_ratio=0.8.)
  - Controller topology (updated for the controller/actuation restructure, see
    kuka_pdz_controller_and_actuation_handoff.md): one gz_ros2_control plugin instance under
    /<robot_name> loads per-arm controllers this node targets with one FollowJointTrajectory
    action client per (controller) endpoint:
      * arm_control=joint_trajectory (default): TWO JointTrajectoryController instances,
        joint_trajectory_controller_lbr_one/_two, each claiming its arm's 7 joints + its own
        gripper's driving joint -- one client per arm serves both arm and gripper goals.
      * arm_control=cartesian_impedance: TWO cartesian_impedance_controller instances,
        cartesian_impedance_lbr_one/_two (7 arm joints, effort; the patched kuka_lbr_control
        controller samples the joint trajectory, drives the Cartesian target through FK, and
        uses it as the nullspace target), PLUS TWO separate position gripper_controller_lbr_
        one/_two for the finger joints -- so arm goals and gripper goals go to different
        clients (see dual_arm_gazebo_cartesian_impedance_controllers.yaml).
"""
import os
import pickle
import sys
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args
from ros_gz_interfaces.srv import SetEntityPose
from scipy.spatial.transform import Rotation
from trajectory_msgs.msg import JointTrajectoryPoint

FPS = 30.0
CM_TO_M = 0.01
GRIPPER_JOINT_MAX_M = 0.032
GRIPPER_OPEN_RATIO_TO_M = 0.032  # see module docstring -- confirmed real pdz retargeting formula

ROLE_TO_ARM = {'hold': 'lbr_one', 'move': 'lbr_two'}
ARM_JOINTS = {
    'lbr_one': [f'lbr_one_A{i}' for i in range(1, 8)],
    'lbr_two': [f'lbr_two_A{i}' for i in range(1, 8)],
}
# Only the left (driving) joint has a command_interface -- right_finger_joint is a <mimic> of it
# (see pdz_gripper_macro.xacro / lbr_dual_arm.xacro's pdz gripper comment) and is therefore not in
# the configured joints list of whichever controller owns the gripper (joint_trajectory_
# controller_lbr_* or gripper_controller_lbr_*); naming it in a goal's joint_names would make the
# controller reject the goal outright (invalid joint).
GRIPPER_JOINTS = {
    'lbr_one': 'lbr_one_pdz_gripper_left_finger_joint',
    'lbr_two': 'lbr_two_pdz_gripper_left_finger_joint',
}
PART_MODEL_NAMES = {
    '0': 'plumbers_block_part0_pipe',
    '1': 'plumbers_block_part1_screw_a',
    '2': 'plumbers_block_part2_base',
    '3': 'plumbers_block_part3_top',
    '4': 'plumbers_block_part4_screw_b',
}


def gripper_open_ratio_to_position(open_ratio: float) -> float:
    return float(np.clip(open_ratio * GRIPPER_OPEN_RATIO_TO_M, 0.0, GRIPPER_JOINT_MAX_M))


class PlanExecutor(Node):
    def __init__(self, log_dir: str, world_name: str, robot_name: str = 'lbr_dual_arm_pdz',
                 arm_control: str = 'joint_trajectory'):
        super().__init__('plan_executor_node')
        self.set_parameters([Parameter('use_sim_time', value=True)])

        with open(os.path.join(log_dir, 'motion.pkl'), 'rb') as f:
            self.motion = pickle.load(f)
        self.traj = np.load(os.path.join(log_dir, 'traj.npy'), allow_pickle=True)

        if arm_control not in ('joint_trajectory', 'cartesian_impedance'):
            raise ValueError(f'arm_control must be joint_trajectory|cartesian_impedance, got {arm_control!r}')
        self.arm_control = arm_control

        # One controller_manager (under /<robot_name>, one gz_ros2_control plugin instance).
        # Which FollowJointTrajectory action serves each arm depends on how gazebo.launch.py
        # brought the rig up (its arm_control launch arg -- must match this node's):
        #   joint_trajectory   -> joint_trajectory_controller_lbr_{one,two}, which ALSO carries
        #                         that arm's pdz gripper driving joint (see
        #                         dual_arm_pdz_gazebo_controllers.yaml) -- one client per arm
        #                         serves both arm paths and gripper commands.
        #   cartesian_impedance -> cartesian_impedance_lbr_{one,two} for the 7 arm joints (the
        #                         patched kuka_lbr_control controller samples the joint
        #                         trajectory, drives the Cartesian target via FK, and uses it as
        #                         the nullspace target), plus a SEPARATE per-arm position
        #                         gripper_controller_lbr_{one,two} for the finger joint (see
        #                         dual_arm_gazebo_cartesian_impedance_controllers.yaml).
        # allow_partial_joints_goal (JTC side) lets each goal name only the joints it moves.
        if arm_control == 'cartesian_impedance':
            arm_ctrl_name = {a: f'cartesian_impedance_{a}' for a in ('lbr_one', 'lbr_two')}
            gripper_ctrl_name = {a: f'gripper_controller_{a}' for a in ('lbr_one', 'lbr_two')}
        else:
            arm_ctrl_name = {a: f'joint_trajectory_controller_{a}' for a in ('lbr_one', 'lbr_two')}
            gripper_ctrl_name = dict(arm_ctrl_name)

        clients_by_ctrl = {}

        def _client(ctrl):
            if ctrl not in clients_by_ctrl:
                c = ActionClient(
                    self, FollowJointTrajectory,
                    f'/{robot_name}/{ctrl}/follow_joint_trajectory')
                self.get_logger().info(f'Waiting for {ctrl} action server...')
                c.wait_for_server()
                clients_by_ctrl[ctrl] = c
            return clients_by_ctrl[ctrl]

        self._arm_clients = {a: _client(arm_ctrl_name[a]) for a in ('lbr_one', 'lbr_two')}
        self._gripper_clients = {a: _client(gripper_ctrl_name[a]) for a in ('lbr_one', 'lbr_two')}

        self._set_pose_client = self.create_client(SetEntityPose, f'/world/{world_name}/set_pose')
        if not self._set_pose_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                f'/world/{world_name}/set_pose service not available -- held parts will not '
                'visually follow the gripper. Continuing anyway (arm/gripper motion is unaffected).')

        self.frame_idx = 0  # traj.npy index of the current (already-reached) state
        self.held_part = {'hold': None, 'move': None}

    # -- part-pose following -------------------------------------------------------------------
    def _publish_held_part_pose(self):
        for role, part_id in self.held_part.items():
            if part_id is None:
                continue
            if self.frame_idx >= len(self.traj):
                continue
            T = self.traj[self.frame_idx][f'part{part_id}']
            pos = T[:3, 3] * CM_TO_M
            quat = Rotation.from_matrix(T[:3, :3]).as_quat()  # xyzw
            req = SetEntityPose.Request()
            req.entity.name = PART_MODEL_NAMES[part_id]
            req.entity.type = 2  # MODEL
            req.pose.position.x, req.pose.position.y, req.pose.position.z = pos.tolist()
            (req.pose.orientation.x, req.pose.orientation.y,
             req.pose.orientation.z, req.pose.orientation.w) = quat.tolist()
            if self._set_pose_client.service_is_ready():
                self._set_pose_client.call_async(req)  # fire-and-forget; next tick supersedes it

    # -- single-goal execution, ticking frame_idx + part pose at 30 Hz alongside it -------------
    def _run_goal(self, arm: str, action_client, joint_names, points, n_frames_to_advance: int):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        goal.trajectory.points = points
        goal.goal_time_tolerance.sec = 5

        send_future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'[{arm}] goal rejected')
            return
        result_future = goal_handle.get_result_async()

        # Tick frame_idx (and any held-part pose) at ~30Hz while the controller executes the
        # trajectory, instead of jumping straight to the end -- keeps the teleported part in sync
        # with the arm's real progress through the segment, not just its final pose.
        dt = 1.0 / FPS
        for _ in range(n_frames_to_advance):
            t0 = time.time()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=dt)
            if result_future.done():
                break
            self.frame_idx += 1
            self._publish_held_part_pose()
            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None or result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f'[{arm}] trajectory execution failed: {result}')

    def _send_arm_path(self, role: str, path: np.ndarray, description: str, active_part):
        arm = ROLE_TO_ARM[role]
        points = []
        for i, q in enumerate(path):
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in q]
            p.time_from_start.sec = int((i + 1) / FPS)
            p.time_from_start.nanosec = int(((i + 1) / FPS % 1.0) * 1e9)
            points.append(p)
        self.get_logger().info(
            f'[{arm}] {description} ({len(path)} waypoints, part={active_part})')
        self._run_goal(arm, self._arm_clients[arm], ARM_JOINTS[arm], points,
                       n_frames_to_advance=len(path))

    def _send_gripper_command(self, role: str, open_ratio: float, description: str, active_part):
        arm = ROLE_TO_ARM[role]
        position = gripper_open_ratio_to_position(open_ratio)
        p = JointTrajectoryPoint()
        p.positions = [position]  # left driving joint only -- right is a mimic, see GRIPPER_JOINTS
        p.time_from_start.sec = 0
        p.time_from_start.nanosec = int(0.5e9)
        self.get_logger().info(
            f'[{arm}] gripper {description} open_ratio={open_ratio:.3f} -> {position * 1000:.1f}mm')
        self._run_goal(arm, self._gripper_clients[arm], [GRIPPER_JOINTS[arm]], [p],
                       n_frames_to_advance=1)

        if description == 'close' and active_part is not None:
            self.held_part[role] = active_part
            self.get_logger().info(f'[{arm}] now holding part {active_part}')
        elif description == 'open' and self.held_part[role] is not None:
            self.get_logger().info(f'[{arm}] released part {self.held_part[role]}')
            self.held_part[role] = None

    def run(self):
        for motion_type, body_type, value, active_part, description in self.motion:
            role = motion_type
            if body_type == 'arm':
                self._send_arm_path(role, np.asarray(value), description, active_part)
            elif body_type == 'gripper':
                self._send_gripper_command(role, float(value), description, active_part)
            else:
                self.get_logger().warn(f'Unknown body_type {body_type}, skipping')
        self.get_logger().info('Plan complete.')


def main(args=None):
    rclpy.init(args=args)
    # Positional: [log_dir] [world_name]; optional: --arm-control joint_trajectory|cartesian_impedance
    # (must match gazebo.launch.py's arm_control). remove_ros_args strips `--ros-args ...`.
    argv = list(remove_ros_args(args=sys.argv))[1:]
    arm_control = 'joint_trajectory'
    if '--arm-control' in argv:
        i = argv.index('--arm-control')
        arm_control = argv[i + 1]
        del argv[i:i + 2]

    default_log_dir = os.path.expanduser('~/Fabrica/logs/plumbers_block_sim')
    log_dir = argv[0] if len(argv) > 0 else default_log_dir
    world_name = argv[1] if len(argv) > 1 else 'plumbers_block'

    node = PlanExecutor(log_dir, world_name, robot_name='lbr_dual_arm_pdz', arm_control=arm_control)
    try:
        node.run()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
