"""Replays a Fabrica motion plan (see plan_executor_node.py's docstring for motion.pkl's
format) on the dual-arm KUKA against real FRI hardware or mock hardware (lbr_dual_arm_bringup's
hardware.launch.py / mock.launch.py), via the standard joint_trajectory_controller/
FollowJointTrajectory action -- no custom control interfaces, no Gazebo dependency.

Differences from plan_executor_node.py (the Gazebo/pdz-gripper version), and why:
  - One combined 14-joint joint_trajectory_controller for both arms (dual_arm_controllers.yaml),
    not two per-arm controllers -- and allow_partial_joints_goal is NOT set there (defaults
    false), so every goal sent to it must name all 14 joints or the controller rejects it
    outright. This node always sends the full 14-joint set: the moving arm's path, and the idle
    arm held at its last known real position (see closed-loop note below).
  - Gripper actuation is opt-in via --gripper (default off): without it, this node never touches
    the grippers at all (no service wait, no homing, no commands) and gripper entries in
    motion.pkl are logged and skipped -- only the arm trajectory is replayed. With --gripper,
    the behavior below applies.
  - Gripper actuation is opt-in via --gripper (default off), through the servo_gripper_julien
    package's per-side services (node name gripper_controller, workspace ~/Workspaces/servo_test
    -- source setup_gripper_env.sh first). NOT through ros2_control/joint_trajectory_controller
    (dual_arm_controllers.yaml only claims the 14 arm joints; both grippers are bare
    `mock_components/GenericSystem` stubs). The dual-gripper launch namespaces the services by
    SIDE -- /left/gripper_controller and /right/gripper_controller. This node maps plan roles to
    sides via ROLE_TO_GRIPPER_NS (hold->left, move->right); the grippers are identical, so that
    mapping is arbitrary but MUST agree with however the launch assigns left/right to the
    physical arms. The five services this node uses (exact `ros2 service call` commands in
    hardware_plan_executor_node.md, "Gripper service interface"):
        ~/calibrate    std_srvs/srv/Trigger               -- sweep the stroke to learn the
                                                             open/closed encoder limits; BLOCKS.
                                                             ~/set_position needs it first.
        ~/open         std_srvs/srv/Trigger               -- drive to the open endpoint; BLOCKS
                                                             until the jaws get there
        ~/set_position servo_gripper_julien/srv/SetPosition -- drive to a normalized target and
                       {position: float 0.0=open..1.0=closed} stop (NO continuous torque); BLOCKS
        ~/hold_close   servo_gripper_julien/srv/HoldClose  -- start closing and keep applying
                       {torque_limit: uint16 0..500}         `torque_limit` holding torque;
                                                             RETURNS IMMEDIATELY, torque persists
                                                             until ~/open, ~/set_position, ~/stop
        ~/stop         std_srvs/srv/Trigger               -- drop holding torque without opening
    **Safety note: the gripper servos are real hardware always** -- there is no mock/sim variant
    of servo_gripper_julien, so gripper motion happens for real even during an arm-mock dry run.
      * motion.pkl gripper entries carry Fabrica's open_ratio (0=closed, 1=open, from
        planning/robot/geometry.py) plus a description ('init' | 'open' | 'close'). This node
        routes on the description, NOT a ratio threshold (the plan's thin-part pre-grasp and
        release moves are description=='open' at open_ratio well below 0.5, so a threshold
        misfires them):
          close + real grasp   -> ~/hold_close at GRIPPER_HOLD_TORQUE_DEFAULT (250,
              --gripper-torque), then _spin_sleep(GRIPPER_CLOSE_SETTLE_S) before the next move
              (the transport carrying the just-gripped part). "Real grasp" per
              _gripper_close_is_grasp(): a part is carried right after this close, or this grip
              is reopened later for the same role (covers a regrasp). Grasps MUST use hold_close.
          close, NOT a grasp   -> ~/set_position(1 - open_ratio); the plan's final 'park' closes,
              no holding torque left on empty jaws.
          open, open_ratio >= GRIPPER_FULL_OPEN_RATIO (0.98) -> ~/open (full-open Trigger).
              Nothing in plumbers_block hits this; startup homing does.
          open, below that     -> ~/set_position(1 - open_ratio): a partial open to the plan's
              own target width (every plan open here -- pre-grasp widen, release).
        open_ratio (0=closed..1=open) maps to position (0.0=open..1.0=closed) as
        position = 1 - open_ratio, clamped to [0, 1].
      * Every service call blocks on its result (up to GRIPPER_MOTION_TIMEOUT_S) and raises on a
        failed or timed-out result -- matching the arm path's stop-the-plan-on-any-unexplained-
        failure behavior. ~/open and ~/set_position only return once the jaws reach the target,
        so nothing extra is needed after them. ~/hold_close returns as soon as the servo STARTS
        closing, so this node then _spin_sleep(GRIPPER_CLOSE_SETTLE_S) (10.0s) after a grasp --
        before the plan's next move, the transport that carries the just-gripped part.
      * At startup this node homes both grippers before the plan, each stage fired on both sides
        at once (_call_gripper_all): ~/calibrate (sweeps the stroke so ~/set_position has
        open/closed encoder limits to map against -- the plan's first gripper entry is a
        set_position), then ~/hold_close (confirms the jaws travel and clears anything left in
        them from a prior run), a settle wait, then ~/open (the plan's own starting state for
        both jaws). Stages stay sequential. Because the startup homing already does this, run()
        SKIPS the plan's own description=='init' gripper entries (motion.pkl entries 2-3);
        otherwise the grippers visibly home twice, back to back.
      * On an aborted run (any exception, incl. Ctrl-C) main() calls ~/stop on both grippers so
        the servos aren't left gripping after the node exits. A normal plan completion leaves the
        grippers in whatever state its last entries commanded (plumbers_block: both
        ~/set_position(0.5), a non-grasp park).
      * Requires gripper_controller running for both sides (source setup_gripper_env.sh, then the
        dual-gripper launch) -- this node fails fast at startup (RuntimeError) if any of
        /{left,right}/gripper_controller/{calibrate,open,set_position,hold_close,stop} is missing,
        and also if the servo_gripper_julien interfaces aren't importable. Verify the live rig with
        `ros2 service list -t | grep gripper_controller`; if the namespaces differ, update
        GRIPPER_CONTROLLER_NODE / GRIPPER_NAMESPACES / ROLE_TO_GRIPPER_NS here.
  - No held-part pose teleporting (that was Gazebo-only, via /world/<world>/set_pose -- there is
    no such service, and no simulated part, on real/mock hardware).
  - Closed-loop, for real: this node subscribes to /<robot_name>/joint_states and uses the latest
    real feedback for BOTH the idle arm's "hold here" position AND the moving arm's start
    position when building the next combined goal, rather than assuming either is still at
    whatever pose a prior goal commanded. Real/mock joints settle slightly off a commanded
    setpoint; feeding the cached commanded endpoint (not the measured pose) into
    _timed_waypoints() hides that error, so the current-position -> path[0] segment gets only the
    nominal 1/30s and the controller snaps across the settle error in a single tick -- a visible
    discontinuity at the start of each goal (was most obvious in the holder arm's first
    transport). Reading measured feedback here lets _timed_waypoints() stretch that first segment
    like any other oversized jump.
  - Velocities are set per waypoint via a finite-difference of the 30fps arm path (central
    difference internally, one-sided at the first sample, zero at the last so each motion.pkl
    entry's goal decelerates to a stop at its boundary -- matching the original node's one-
    goal-per-entry design). controller_manager's update_rate is already 100Hz/10ms for both mock
    and hardware (dual_arm_controllers.yaml), matching the KUKA FRI server's required 10ms send
    period exactly -- joint_trajectory_controller's own spline interpolation already resamples
    onto that cadence regardless of the 30fps input spacing, so no manual resampling onto a fixed
    10ms grid is needed. What DOES matter is not leaving velocities implicitly zero at every
    waypoint (only true zero-order-hold position targets would need that), which would otherwise
    produce a velocity discontinuity every 1/30s -- hence computing them explicitly here.
  - Waypoint blending: unlike the Gazebo version, the per-waypoint time_from_start is NOT a fixed
    i/FPS -- see _timed_waypoints() below. Real/mock hardware has no guarantee that a new goal's
    path[0] is where the moving arm actually is (true by construction at the very start of the
    plan, and demonstrably not always true between consecutive same-arm entries in
    plumbers_block_sim/motion.pkl -- e.g. entries 21 and 38 land 4.9deg/6.0deg off). A fixed 1/30s
    budget for that gap implies velocities that trip lbr_fri_ros2::CommandGuard's hard per-joint
    ceiling and crash the whole hardware interface (std::runtime_error on the FRI realtime
    thread). _timed_waypoints() instead stretches (never shortens) any segment -- including the
    current-position -> path[0] one -- whose implied velocity would exceed a safety-margined
    fraction of that ceiling. See hardware_plan_executor_node.md for the full incident writeup.
  - On any rejected goal or non-SUCCESSFUL result, this node stops the whole plan immediately
    (raises) rather than logging and continuing -- continuing to send further goals to real
    hardware after an unexplained failure is not an acceptable default.

Usage: same CLI shape as plan_executor_node.py, but pointed at lbr_dual_arm_bringup's namespace
instead of the Gazebo/pdz one:
    ros2 launch lbr_dual_arm_bringup mock.launch.py       # or hardware.launch.py, for real FRI
    ros2 launch lbr_dual_arm_moveit_config moveit_rviz.launch.py mode:=mock   # visualization only
    source setup_gripper_env.sh                           # only if using --gripper
    python3 hardware_plan_executor_node.py [log_dir] [robot_name] [--gripper] [--gripper-torque N]
Pass --gripper to also home and actuate the grippers (via servo_gripper_julien -- requires
gripper_controller running for both sides, see below): real grasps use hold_close, every other
gripper move uses set_position to the plan's own target width. Without it, only the arm
trajectory is replayed. --gripper-torque N (0..500, default 250) sets the hold_close torque_limit.
"""
import argparse
import os
import pickle
import sys
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

# servo_gripper_julien's custom requests: HoldClose (torque_limit uint16, close-and-hold) and
# SetPosition (position float 0.0=open..1.0=closed, drive-and-stop). Only needed for --gripper;
# imported lazily-tolerant so an arm-only replay doesn't require the gripper workspace on the
# path. __init__ raises a clear error if either is still None then.
try:
    from servo_gripper_julien.srv import HoldClose, SetPosition
except ImportError:
    HoldClose = None
    SetPosition = None

FPS = 30.0

# Per-joint hard velocity ceiling (rad/s), order A1..A7, same for both arms -- this is the LBR
# med7's true max_velocity as wired through lbr_system_interface.xacro into
# lbr_fri_ros2::CommandGuard::command_in_velocity_limits_ (lbr_fri_ros2/src/command_guard.cpp),
# which throws std::runtime_error on the FRI realtime thread (see
# lbr_fri_ros2/src/interfaces/position_command.cpp) -- i.e. crashes the whole hardware interface
# -- the instant a commanded/measured joint delta implies more than this. See
# hardware_plan_executor_node.md ("Waypoint blending") for how this is used.
JOINT_MAX_VELOCITY_RAD_S = np.array([
    1.7104226669544429, 1.7104226669544429, 1.7453292519943295,
    2.2689280275926285, 2.4434609527920612, 3.1415926535897931, 3.1415926535897931,
])
# Blended segments target this fraction of the hard ceiling above, not the wall itself -- margin
# for command_guard comparing *measured* (not commanded) position deltas, the joint position
# filter's smoothing in position_command.cpp, and this node's own finite-difference velocities
# being an approximation rather than the controller's actual spline.
BLEND_VELOCITY_SAFETY_FACTOR = 0.3
# Log (not just silently stretch) any segment that needed more than this multiple of the nominal
# 1/FPS spacing -- signal that the plan had a real discontinuity, not just blending noise.
BLEND_LOG_THRESHOLD = 1.5

# --- servo_gripper_julien service interface (see hardware_plan_executor_node.md,
# "Gripper service interface", for the exact `ros2 service call` commands) ------------------------
# Node name of the running gripper controllers, and the two SIDE namespaces the dual-gripper
# launch puts them under: /left/gripper_controller/... and /right/gripper_controller/... . Verify
# the live rig with `ros2 service list -t | grep gripper_controller`. Single-gripper launches
# expose /gripper_controller/... with no side prefix; this node only targets the dual launch.
GRIPPER_CONTROLLER_NODE = 'gripper_controller'
GRIPPER_NAMESPACES = ('left', 'right')
# Timeout on each blocking gripper service call (open / set_position / hold_close / stop).
GRIPPER_MOTION_TIMEOUT_S = 30.0
GRIPPER_HOMING_SERVICE_TIMEOUT_S = 10.0
# motion.pkl gripper entries carry Fabrica's open_ratio (0=closed, 1=open); ~/set_position wants
# the opposite normalization (0.0=open, 1.0=closed), so position = 1 - open_ratio. A
# description=='open' entry at or above this ratio uses the full-open Trigger instead of
# set_position (nothing in plumbers_block does; startup homing does).
GRIPPER_FULL_OPEN_RATIO = 0.98
# torque_limit field of servo_gripper_julien/srv/HoldClose (uint16, 0..500) -- the holding torque
# the servo keeps applying after the jaws stop closing. 250 is the value in the rig owner's
# gripper_service_commands.md example. Override per run with --gripper-torque.
GRIPPER_HOLD_TORQUE_DEFAULT = 250
GRIPPER_HOLD_TORQUE_MAX = 500
# hold_close returns as soon as the servo STARTS closing (open, by contrast, blocks until the
# jaws reach the open endpoint). Give the jaws this long to actually close on and grip the part
# before the plan's next arm move carries it. Tune against the real observed close time.
GRIPPER_CLOSE_SETTLE_S = 10.0

ROLE_TO_ARM = {'hold': 'lbr_one', 'move': 'lbr_two'}
# Plan role -> gripper SIDE namespace (one of GRIPPER_NAMESPACES). The two grippers are
# identical, so this mapping is arbitrary -- but it MUST match however the dual-gripper launch
# assigns left/right to the physical arms, or a "hold" grasp actuates the gripper on the moving
# arm. Decoupled from ROLE_TO_ARM on purpose: the arm joints stay /lbr_one|/lbr_two regardless.
ROLE_TO_GRIPPER_NS = {'hold': 'left', 'move': 'right'}
ARM_JOINTS = {
    'lbr_one': [f'lbr_one_A{i}' for i in range(1, 8)],
    'lbr_two': [f'lbr_two_A{i}' for i in range(1, 8)],
}
# The combined controller's full joint set -- every goal must name exactly these 14, in any
# order (allow_partial_joints_goal is false in dual_arm_controllers.yaml).
ALL_JOINTS = ARM_JOINTS['lbr_one'] + ARM_JOINTS['lbr_two']


class HardwarePlanExecutor(Node):
    def __init__(self, log_dir: str, robot_name: str = 'lbr_dual_arm', gripper: bool = False,
                 gripper_torque: int = GRIPPER_HOLD_TORQUE_DEFAULT):
        super().__init__('hardware_plan_executor_node')
        self._gripper_enabled = gripper
        self._gripper_torque = int(np.clip(gripper_torque, 0, GRIPPER_HOLD_TORQUE_MAX))
        if gripper and self._gripper_torque != gripper_torque:
            self.get_logger().warn(
                f'--gripper-torque {gripper_torque} out of range 0..{GRIPPER_HOLD_TORQUE_MAX}, '
                f'clamped to {self._gripper_torque}')

        with open(os.path.join(log_dir, 'motion.pkl'), 'rb') as f:
            self.motion = pickle.load(f)

        self._current_positions = {}  # joint name -> latest measured position, from joint_states
        self.create_subscription(
            JointState, f'/{robot_name}/joint_states', self._joint_state_cb, 10)

        self._action_client = ActionClient(
            self, FollowJointTrajectory,
            f'/{robot_name}/joint_trajectory_controller/follow_joint_trajectory')
        self.get_logger().info('Waiting for joint_trajectory_controller action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Waiting for initial /joint_states...')
        while rclpy.ok() and not all(j in self._current_positions for j in ALL_JOINTS):
            rclpy.spin_once(self, timeout_sec=0.5)

        # Per side: /<side>/gripper_controller/{calibrate,open,set_position,hold_close,stop}.
        # calibrate, open and stop are std_srvs/Trigger; set_position is
        # servo_gripper_julien/srv/SetPosition (position); hold_close is
        # servo_gripper_julien/srv/HoldClose (torque_limit). ~/calibrate sweeps the stroke to
        # learn the open/closed encoder limits -- ~/set_position needs it (raises "position
        # unavailable: run ~/calibrate first" otherwise); startup homing runs it once per side.
        self._gripper_calibrate_clients = {}
        self._gripper_open_clients = {}
        self._gripper_set_position_clients = {}
        self._gripper_hold_close_clients = {}
        self._gripper_stop_clients = {}
        if not self._gripper_enabled:
            self.get_logger().info(
                'Gripper actuation disabled (pass --gripper to enable) -- replaying arm '
                'trajectory only, gripper entries in the plan will be skipped.')
            return

        if HoldClose is None or SetPosition is None:
            raise RuntimeError(
                'servo_gripper_julien.srv (HoldClose / SetPosition) could not be imported -- '
                'source the gripper workspace (setup_gripper_env.sh) before running with '
                '--gripper so the servo_gripper_julien interfaces are on the path.')

        for ns in GRIPPER_NAMESPACES:
            base = f'/{ns}/{GRIPPER_CONTROLLER_NODE}'
            self._gripper_calibrate_clients[ns] = self.create_client(
                Trigger, f'{base}/calibrate')
            self._gripper_open_clients[ns] = self.create_client(Trigger, f'{base}/open')
            self._gripper_set_position_clients[ns] = self.create_client(
                SetPosition, f'{base}/set_position')
            self._gripper_hold_close_clients[ns] = self.create_client(
                HoldClose, f'{base}/hold_close')
            self._gripper_stop_clients[ns] = self.create_client(Trigger, f'{base}/stop')

        self.get_logger().info(
            f'Waiting for {GRIPPER_CONTROLLER_NODE} calibrate / open / set_position / hold_close '
            f'/ stop services on /{GRIPPER_NAMESPACES[0]} and /{GRIPPER_NAMESPACES[1]}...')
        for ns in GRIPPER_NAMESPACES:
            base = f'/{ns}/{GRIPPER_CONTROLLER_NODE}'
            for service, client in (
                    ('calibrate', self._gripper_calibrate_clients[ns]),
                    ('open', self._gripper_open_clients[ns]),
                    ('set_position', self._gripper_set_position_clients[ns]),
                    ('hold_close', self._gripper_hold_close_clients[ns]),
                    ('stop', self._gripper_stop_clients[ns])):
                if not client.wait_for_service(timeout_sec=GRIPPER_HOMING_SERVICE_TIMEOUT_S):
                    raise RuntimeError(
                        f'{base}/{service} service not available -- is '
                        f'{GRIPPER_CONTROLLER_NODE} running for both sides (source '
                        'setup_gripper_env.sh, then the dual-gripper launch)? Check '
                        '`ros2 service list -t | grep gripper_controller`; if the live '
                        f'namespaces are not /{GRIPPER_NAMESPACES[0]} and '
                        f'/{GRIPPER_NAMESPACES[1]}, update GRIPPER_CONTROLLER_NODE / '
                        'GRIPPER_NAMESPACES / ROLE_TO_GRIPPER_NS here.')

        # Bring both grippers to a known physical state before the plan. Each stage runs on BOTH
        # sides concurrently (_call_gripper_all): ~/calibrate first (sweeps the stroke so
        # ~/set_position has open/closed encoder limits to map against -- the plan's very first
        # gripper entry is a set_position), then one hold_close per side (confirms the jaws
        # travel, and clears anything left in them from a prior run), a settle wait (hold_close
        # returns immediately -- see _call_hold_close), then open per side (the plan's own
        # starting state for both jaws). Stages stay sequential (calibrate must finish before the
        # jaws are driven). Because startup homing already does this, run() skips the plan's own
        # description=='init' gripper entries so the grippers don't visibly home twice.
        self.get_logger().info(
            'Homing both grippers before the plan (each stage both sides in parallel): '
            'calibrate, hold_close (confirm travel), settle, then open...')
        self._call_gripper_all(
            self._gripper_calibrate_clients, Trigger.Request(), 'calibrate', 'home')
        hold_close_req = HoldClose.Request()
        hold_close_req.torque_limit = self._gripper_torque
        self._call_gripper_all(
            self._gripper_hold_close_clients, hold_close_req, 'hold_close', 'home')
        self._spin_sleep(GRIPPER_CLOSE_SETTLE_S)
        self._call_gripper_all(
            self._gripper_open_clients, Trigger.Request(), 'open', 'home')

    def _spin_sleep(self, seconds: float) -> None:
        """Sleep `seconds` while still spinning the node, so /joint_states keeps flowing and
        service futures keep completing while we wait for gripper hardware to move."""
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, end - time.time())))

    def _call_gripper_all(self, clients_by_ns: dict, request, label: str, context: str) -> None:
        """Call the same service on every gripper side CONCURRENTLY: send all requests, then wait
        for all results, so both sides calibrate / home at once instead of left-then-right. The
        one `request` instance is reused for every side -- safe, call_async serializes it
        synchronously and keeps no reference. Raises if any side fails or times out (shared
        GRIPPER_MOTION_TIMEOUT_S budget across the batch); every response type used here carries
        .success / .message. Blocking, like the single-side helpers."""
        tag = f'{label} ({context})' if context else label
        futures = {ns: client.call_async(request) for ns, client in clients_by_ns.items()}
        deadline = time.time() + GRIPPER_MOTION_TIMEOUT_S
        while rclpy.ok() and not all(f.done() for f in futures.values()):
            if time.time() >= deadline:
                break
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, deadline - time.time())))
        failures = []
        for ns, future in futures.items():
            result = future.result() if future.done() else None
            if result is None or not result.success:
                failures.append(
                    f'[{ns}] '
                    f'{result.message if result is not None else "service call timed out"}')
            else:
                self.get_logger().info(f'[{ns}] gripper {tag}: {result.message}')
        if failures:
            raise RuntimeError(f'gripper {tag} failed -- ' + '; '.join(failures))

    def _call_gripper_trigger(self, gripper: str, client, service_name: str,
                              context: str = '') -> None:
        """Blocking call of an ~/open or ~/stop std_srvs/Trigger service; raises on a failed or
        timed-out result. ~/open only returns once the jaws reach the open endpoint, so no settle
        wait is needed after it (unlike hold_close)."""
        tag = f'{service_name} ({context})' if context else service_name
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=GRIPPER_MOTION_TIMEOUT_S)
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError(
                f'[{gripper}] gripper {tag} failed: '
                f'{result.message if result is not None else "service call timed out"}')
        self.get_logger().info(f'[{gripper}] gripper {tag}: {result.message}')

    def _call_set_position(self, gripper: str, open_ratio: float, context: str) -> None:
        """Blocking call of ~/set_position (servo_gripper_julien/srv/SetPosition): drive to a
        normalized target and stop, with NO continuous holding torque. Takes Fabrica's open_ratio
        (0=closed, 1=open) and sends the service's opposite normalization,
        position = 1 - open_ratio (0.0=open, 1.0=closed), clamped to [0, 1]. For non-grasp moves
        only -- pre-grasp widths, releases, and the plan's init/park closes. Raises on a failed
        or timed-out result."""
        position = float(np.clip(1.0 - open_ratio, 0.0, 1.0))
        req = SetPosition.Request()
        req.position = position
        future = self._gripper_set_position_clients[gripper].call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=GRIPPER_MOTION_TIMEOUT_S)
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError(
                f'[{gripper}] gripper set_position({position:.3f}) ({context}) failed: '
                f'{result.message if result is not None else "service call timed out"}')
        self.get_logger().info(
            f'[{gripper}] gripper set_position({position:.3f}, open_ratio={open_ratio:.3f}, '
            f'{context}): {result.message}')

    def _call_hold_close(self, gripper: str, context: str) -> None:
        """Blocking call of the ~/hold_close servo_gripper_julien/srv/HoldClose service with this
        run's torque_limit; raises on a failed or timed-out result. NOTE: the service returns as
        soon as the servo STARTS closing and keeps applying torque until ~/open, ~/set_position
        or ~/stop -- callers must _spin_sleep(GRIPPER_CLOSE_SETTLE_S) afterwards before relying on
        the grip."""
        req = HoldClose.Request()
        req.torque_limit = self._gripper_torque
        future = self._gripper_hold_close_clients[gripper].call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=GRIPPER_MOTION_TIMEOUT_S)
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError(
                f'[{gripper}] gripper hold_close ({context}) failed: '
                f'{result.message if result is not None else "service call timed out"}')
        self.get_logger().info(
            f'[{gripper}] gripper hold_close ({context}, torque_limit={self._gripper_torque}): '
            f'{result.message}')

    def _release_grippers(self, reason: str) -> None:
        """Call ~/stop on both grippers to drop any holding torque (best effort -- logs, does not
        raise). Used on an aborted run so the servos aren't left gripping after the node exits."""
        if not self._gripper_enabled:
            return
        self.get_logger().warn(f'{reason} -- calling ~/stop on both grippers to release torque')
        for ns in GRIPPER_NAMESPACES:
            try:
                self._call_gripper_trigger(ns, self._gripper_stop_clients[ns], 'stop', reason)
            except Exception as exc:  # noqa: BLE001 -- best-effort cleanup
                self.get_logger().error(f'[{ns}] gripper stop during abort failed: {exc}')

    def _joint_state_cb(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            if name in ARM_JOINTS['lbr_one'] or name in ARM_JOINTS['lbr_two']:
                self._current_positions[name] = position

    def _timed_waypoints(self, moving_arm: str, current_position: np.ndarray, path: np.ndarray):
        """Per-waypoint (time_from_start, velocity) for `path`, blending -- stretching, never
        shortening -- the nominal 1/FPS spacing of any segment (including the current-position ->
        path[0] segment, which the plan itself says nothing about) whose implied per-joint
        velocity would exceed JOINT_MAX_VELOCITY_RAD_S * BLEND_VELOCITY_SAFETY_FACTOR. Falls back
        to the old fixed-dt central-difference math exactly when nothing needs blending. See
        hardware_plan_executor_node.md ("Waypoint blending") for why this exists."""
        n = len(path)
        nominal_dt = 1.0 / FPS
        safe_max_velocity = JOINT_MAX_VELOCITY_RAD_S * BLEND_VELOCITY_SAFETY_FACTOR

        extended = np.vstack([current_position[np.newaxis, :], path])
        gaps = np.diff(extended, axis=0)  # gaps[0] = path[0] - current_position
        per_joint_dt = np.abs(gaps) / safe_max_velocity
        seg_dt = np.maximum(nominal_dt, per_joint_dt.max(axis=1))

        for i in np.flatnonzero(per_joint_dt.max(axis=1) > nominal_dt * BLEND_LOG_THRESHOLD):
            joint = ARM_JOINTS[moving_arm][int(np.argmax(per_joint_dt[i]))]
            self.get_logger().warn(
                f'[{moving_arm}] blending oversized jump before waypoint {i} ({joint}: '
                f'{np.degrees(gaps[i, np.argmax(per_joint_dt[i])]):+.1f} deg) -- stretching this '
                f'segment to {seg_dt[i] * 1000:.0f} ms (nominal {nominal_dt * 1000:.0f} ms)')

        times = np.cumsum(seg_dt)
        velocities = np.zeros_like(path)
        if n > 1:
            velocities[1:-1] = (path[2:] - path[:-2]) / (times[2:] - times[:-2])[:, np.newaxis]
            velocities[0] = (path[1] - path[0]) / seg_dt[1]
        # velocities[-1] stays 0.0: decelerate to a stop at this entry's goal boundary.
        return times, velocities

    def _send_arm_path(self, role: str, path: np.ndarray, description: str, active_part):
        moving_arm = ROLE_TO_ARM[role]
        idle_arm = 'lbr_two' if moving_arm == 'lbr_one' else 'lbr_one'

        # Refresh /joint_states so BOTH arms' positions below are measured, not a cached copy of
        # what a prior goal commanded. joint_states publishes at 100Hz, so a couple of spins gets
        # a fresh sample. This matters for the moving arm: real/mock joints settle a few tenths of
        # a degree off the last setpoint, and _timed_waypoints() needs the true gap between where
        # the arm actually is and path[0] -- otherwise it budgets that segment only the nominal
        # 1/FPS and the controller snaps across the settle error in one tick (the discontinuity
        # seen at the start of the holder arm's first transport).
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.02)
        idle_positions = [self._current_positions[j] for j in ARM_JOINTS[idle_arm]]
        current_position = np.array([self._current_positions[j] for j in ARM_JOINTS[moving_arm]])

        times, velocities = self._timed_waypoints(moving_arm, current_position, path)

        points = []
        for q, v, t in zip(path, velocities, times):
            p = JointTrajectoryPoint()
            if moving_arm == 'lbr_one':
                p.positions = [float(x) for x in q] + idle_positions
                p.velocities = [float(x) for x in v] + [0.0] * 7
            else:
                p.positions = idle_positions + [float(x) for x in q]
                p.velocities = [0.0] * 7 + [float(x) for x in v]
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t % 1.0) * 1e9)
            points.append(p)

        self.get_logger().info(
            f'[{moving_arm}] {description} ({len(path)} waypoints, part={active_part}), '
            f'holding {idle_arm} at its current position')
        self._run_goal(points)
        # Deliberately NOT caching path[-1] into self._current_positions here -- the next
        # _send_arm_path() re-reads live /joint_states instead, so the real settle error at the
        # end of this goal is visible to _timed_waypoints() and gets blended, not snapped.

    def _send_gripper_command(self, role: str, open_ratio: float, description: str,
                              active_part, is_grasp: bool) -> None:
        gripper = ROLE_TO_GRIPPER_NS[role]
        if not self._gripper_enabled:
            self.get_logger().info(
                f'[{gripper}] gripper {description} (part={active_part}) skipped -- run with '
                '--gripper to actuate grippers')
            return
        if description == 'close' and is_grasp:
            # The one case that MUST be a torque grip. hold_close returns immediately; wait out
            # the close before the plan's next move (the transport carrying this part) relies on
            # the grip.
            self.get_logger().info(
                f'[{gripper}] gripper close (part={active_part}, open_ratio={open_ratio:.3f}) '
                f'-> ~/hold_close torque_limit={self._gripper_torque} [grasp]')
            self._call_hold_close(gripper, description)
            self._spin_sleep(GRIPPER_CLOSE_SETTLE_S)
        elif description == 'open' and open_ratio >= GRIPPER_FULL_OPEN_RATIO:
            self.get_logger().info(
                f'[{gripper}] gripper open (open_ratio={open_ratio:.3f}) -> ~/open [full open]')
            self._call_gripper_trigger(
                gripper, self._gripper_open_clients[gripper], 'open', description)
        else:
            # Partial open (pre-grasp widen / release) or a non-grasp close (init/park): drive to
            # the plan's own target width with no holding torque.
            self.get_logger().info(
                f'[{gripper}] gripper {description} (part={active_part}, '
                f'open_ratio={open_ratio:.3f}) -> ~/set_position [no torque]')
            self._call_set_position(gripper, open_ratio, description)

    def _run_goal(self, points):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ALL_JOINTS
        goal.trajectory.points = points
        goal.goal_time_tolerance.sec = 5

        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('goal rejected by joint_trajectory_controller')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None or result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f'trajectory execution failed: {result}')

    def _gripper_close_is_grasp(self, idx: int, role: str) -> bool:
        """Classify a description=='close' gripper entry as a real grasp (needs ~/hold_close, a
        continuous torque grip) vs. a non-grasp positioning close (the plan's final 'park'
        closes -- driven with ~/set_position, no holding torque). description=='init' closes are
        filtered out before this is called. Either signal is sufficient:
          (a) before the next gripper entry, an arm segment names an active_part -- the plan
              starts carrying a part immediately after this close;
          (b) a later gripper entry for the SAME role is a description=='open' -- this grip gets
              released, so it held something (covers a regrasp, where the holder re-clamps a part
              it is already carrying and no fresh active_part tag follows).
        A close with neither -- nothing carried after it, never reopened -- is a park."""
        tail = self.motion[idx + 1:]
        for _mt, body_type, _value, part, _desc in tail:
            if body_type == 'gripper':
                break
            if body_type == 'arm' and part is not None:
                return True
        for mt, body_type, _value, _part, desc in tail:
            if body_type == 'gripper' and mt == role and desc == 'open':
                return True
        return False

    def run(self):
        for idx, (motion_type, body_type, value, active_part, description) in enumerate(
                self.motion):
            role = motion_type
            if body_type == 'arm':
                self._send_arm_path(role, np.asarray(value), description, active_part)
            elif body_type == 'gripper':
                if description == 'init' and self._gripper_enabled:
                    # Startup homing (see __init__) already brought both jaws to a known open
                    # state -- replaying the plan's own init entries here just homes them again.
                    self.get_logger().info(
                        f'[{ROLE_TO_GRIPPER_NS[role]}] gripper init entry skipped -- startup '
                        'homing already established a known state')
                    continue
                is_grasp = (description == 'close'
                            and self._gripper_close_is_grasp(idx, role))
                self._send_gripper_command(
                    role, float(value), description, active_part, is_grasp)
            else:
                self.get_logger().warn(f'Unknown body_type {body_type}, skipping')
        self.get_logger().info('Plan complete.')


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'log_dir', nargs='?', default=os.path.expanduser('~/Fabrica/logs/plumbers_block_sim'),
        help='directory containing motion.pkl (default: %(default)s)')
    parser.add_argument(
        'robot_name', nargs='?', default='lbr_dual_arm',
        help='ros2_control robot_name namespace (default: %(default)s)')
    parser.add_argument(
        '--gripper', action='store_true',
        help='wait for the servo_gripper_julien open / set_position / hold_close / stop services '
             '(namespaced /left/... and /right/...), home both grippers at startup, and actuate '
             'them from the plan: real grasps use hold_close (torque grip), every other move '
             "(pre-grasp width, release, init/park) uses set_position to the plan's own target "
             'width. Without this flag, gripper entries are skipped and only the arm trajectory '
             'is replayed. Requires the gripper workspace on the path -- source '
             'setup_gripper_env.sh first.')
    parser.add_argument(
        '--gripper-torque', type=int, default=GRIPPER_HOLD_TORQUE_DEFAULT,
        metavar=f'0..{GRIPPER_HOLD_TORQUE_MAX}',
        help='torque_limit for every grasp hold_close call (servo_gripper_julien/srv/HoldClose, '
             f'uint16 0..{GRIPPER_HOLD_TORQUE_MAX}; default %(default)s). Out-of-range values '
             'are clamped. Only meaningful with --gripper.')
    cli_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:] if args is None else args
    parsed = parser.parse_args(cli_args)

    node = HardwarePlanExecutor(parsed.log_dir, robot_name=parsed.robot_name,
                                 gripper=parsed.gripper, gripper_torque=parsed.gripper_torque)
    try:
        node.run()
    except BaseException as exc:  # incl. KeyboardInterrupt -- don't leave the servos gripping
        node._release_grippers(f'run aborted ({type(exc).__name__})')
        raise
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
