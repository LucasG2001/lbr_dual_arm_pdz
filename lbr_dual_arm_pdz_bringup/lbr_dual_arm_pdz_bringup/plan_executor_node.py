"""Replays a Fabrica motion plan (planning/run_planning.sh's logs/<exp>/... output, here
logs/plumbers_block_sim) on the dual-arm KUKA, via the standard FollowJointTrajectory action --
no custom control interfaces.

ONE node for every rig. What differs between Gazebo, mock, and real FRI hardware is expressed
entirely through CLI args (which the bringup launch files set), NOT a separate code path:

  --controller-topology per-arm|combined   per-arm (default, Gazebo/pdz): one action client per
      arm at joint_trajectory_controller_lbr_{one,two} (each also carries its arm's pdz gripper
      driving joint -- dual_arm_pdz_gazebo_controllers.yaml). combined (real/mock FRI): a single
      14-joint joint_trajectory_controller (dual_arm_controllers.yaml); every goal names all 14
      joints, the idle arm pinned to its measured /joint_states position.
  --arm-control joint_trajectory|cartesian_impedance   cartesian_impedance replays against the
      per-arm effort-interface cartesian_impedance_lbr_{one,two} instead of the position JTCs
      (see dual_arm_*_cartesian_impedance_controllers.yaml). These controllers are the same
      build on Gazebo and the real rig, so impedance mode needs no --controller-topology switch
      -- it is always per-arm. (Variable-per-phase-gain impedance lives in
      variable_impedance_plan_executor_node.py.)
  --gripper-backend none|sim|servo   none (default): arm-only, gripper entries logged+skipped.
      sim (Gazebo): the simulated pdz finger joint, driven through a single-joint
      FollowJointTrajectory goal (open_ratio -> metres). servo (real hardware): the
      servo_gripper_julien service gripper -- there is NO sim/mock variant, so this actuates
      real hardware even in an arm-mock dry run.
  --visualize-held-parts --world <name>   Gazebo-only: teleport the currently-held part to match
      logs/<exp>/traj.npy via gz-sim's /world/<world>/set_pose at 30 Hz alongside each arm goal.
  --use-sim-time   set the node's use_sim_time param (Gazebo; the launch also brings a /clock
      bridge). Off = wall clock (real/mock hardware).
  --on-failure raise|continue   raise (default): stop the whole plan on any rejected goal or
      non-SUCCESSFUL result -- the right default for real hardware. continue: log and go on.
      GOAL_TOLERANCE_VIOLATED under --arm-control cartesian_impedance is always tolerated
      (expected for a compliant arm), regardless of this flag.

hardware_plan_executor_node.py is a thin shim over this file that just prepends the real/mock
FRI defaults (--controller-topology combined --robot-name lbr_dual_arm).

Design notes carried over from the two predecessor nodes:
  - motion.pkl's entries [motion_type, body_type, value, active_part, description] form ONE
    strict global timeline (only one arm ever moves at a time), so this node walks the list in
    order, sending one goal at a time and blocking on its result.
  - motion_type ('move'/'hold') is a role, not a robot name. hold ~= lbr_one, move ~= lbr_two
    (ROLE_TO_ARM) -- lbr_dual_arm.xacro's base joints put lbr_one at y=-0.42, lbr_two at y=+0.42.
  - Arm paths are Fabrica-resampled at a fixed 30 fps; each waypoint is 1/30 s apart nominally.
    _timed_waypoints() STRETCHES (never shortens) any segment -- including the synthetic
    "measured current pose -> path[0]" segment at the start of each goal -- whose implied
    per-joint velocity would exceed a safety-margined fraction of the LBR med7's real per-joint
    ceiling, because on real hardware lbr_fri_ros2::CommandGuard throws (crashing the FRI
    realtime thread) the instant a commanded/measured delta implies more than that. Harmless in
    Gazebo: the blend is a no-op except at genuine plan discontinuities.
  - Per-waypoint velocities are set explicitly (central difference over the resulting, possibly
    non-uniform, time grid; one-sided at the first sample, zero at the last so each motion.pkl
    entry decelerates to a stop at its boundary) -- so there is no velocity discontinuity every
    1/30 s.
  - open_ratio -> pdz finger joint position (sim backend): clamp(open_ratio * 0.032, 0, 0.032)
    -- 0.032 m (PDZ_JAW_TRAVEL, per-finger stroke) is the real pdz retargeting formula.
  - Held parts (viz): traj.npy frame 0 is the pre-command initial state, frame (i+1) the state
    after motion.pkl's i-th flattened waypoint/command.
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
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

# servo_gripper_julien's custom requests -- only needed for --gripper-backend servo. Imported
# lazily-tolerant so an arm-only or Gazebo replay doesn't require the gripper workspace on the
# path; __init__ raises a clear error if these are still None when servo is actually selected.
try:
    from servo_gripper_julien.srv import HoldClose, SetPosition
except ImportError:
    HoldClose = None
    SetPosition = None

FPS = 30.0
CM_TO_M = 0.01
GRIPPER_JOINT_MAX_M = 0.032
GRIPPER_OPEN_RATIO_TO_M = 0.032  # confirmed real pdz retargeting formula (PDZ_JAW_TRAVEL)

ARMS = ("lbr_one", "lbr_two")
ROLE_TO_ARM = {"hold": "lbr_one", "move": "lbr_two"}
ARM_JOINTS = {
    "lbr_one": [f"lbr_one_A{i}" for i in range(1, 8)],
    "lbr_two": [f"lbr_two_A{i}" for i in range(1, 8)],
}
# The combined controller's full joint set -- every combined-topology goal names exactly these
# 14 (dual_arm_controllers.yaml's order).
ALL_JOINTS = ARM_JOINTS["lbr_one"] + ARM_JOINTS["lbr_two"]
# Only the left (driving) joint has a command interface -- right_finger_joint is a <mimic>, so
# it is not in whichever controller owns the gripper and naming it would make the goal be
# rejected outright. (sim backend only.)
GRIPPER_JOINTS = {
    "lbr_one": "lbr_one_pdz_gripper_left_finger_joint",
    "lbr_two": "lbr_two_pdz_gripper_left_finger_joint",
}
PART_MODEL_NAMES = {
    "0": "plumbers_block_part0_pipe",
    "1": "plumbers_block_part1_screw_a",
    "2": "plumbers_block_part2_base",
    "3": "plumbers_block_part3_top",
    "4": "plumbers_block_part4_screw_b",
}

# Per-joint hard velocity ceiling (rad/s), order A1..A7, same for both arms -- the LBR med7's
# true max_velocity as wired through lbr_system_interface.xacro into
# lbr_fri_ros2::CommandGuard::command_in_velocity_limits_, which throws std::runtime_error on
# the FRI realtime thread the instant a commanded/measured joint delta implies more than this.
JOINT_MAX_VELOCITY_RAD_S = np.array([
    1.7104226669544429, 1.7104226669544429, 1.7453292519943295,
    2.2689280275926285, 2.4434609527920612, 3.1415926535897931, 3.1415926535897931,
])
# Blended segments target this fraction of the hard ceiling -- margin for command_guard
# comparing *measured* (not commanded) deltas, the joint position filter's smoothing, and this
# node's finite-difference velocities being an approximation of the controller's spline.
BLEND_VELOCITY_SAFETY_FACTOR = 0.3
# Log (not just silently stretch) any segment needing more than this multiple of the nominal
# 1/FPS spacing -- signal that the plan had a real discontinuity, not just blending noise.
BLEND_LOG_THRESHOLD = 1.5

# --- servo_gripper_julien service interface (--gripper-backend servo) --------------------------
GRIPPER_CONTROLLER_NODE = "gripper_controller"
GRIPPER_NAMESPACES = ("left", "right")
GRIPPER_MOTION_TIMEOUT_S = 30.0
GRIPPER_HOMING_SERVICE_TIMEOUT_S = 10.0
# A description=='open' entry at or above this ratio uses the full-open Trigger instead of
# set_position (nothing in plumbers_block does; startup homing does).
GRIPPER_FULL_OPEN_RATIO = 0.98
GRIPPER_HOLD_TORQUE_DEFAULT = 250
GRIPPER_HOLD_TORQUE_MAX = 500
# hold_close returns as soon as the servo STARTS closing; give the jaws this long to actually
# grip before the plan's next arm move carries the part.
GRIPPER_CLOSE_SETTLE_S = 10.0
# Plan role -> gripper SIDE namespace. The two grippers are identical, so this mapping is
# arbitrary -- but it MUST match however the dual-gripper launch assigns left/right to the
# physical arms. Decoupled from ROLE_TO_ARM on purpose.
ROLE_TO_GRIPPER_NS = {"hold": "left", "move": "right"}


def gripper_open_ratio_to_position(open_ratio: float) -> float:
    return float(np.clip(open_ratio * GRIPPER_OPEN_RATIO_TO_M, 0.0, GRIPPER_JOINT_MAX_M))


class PlanExecutor(Node):
    def __init__(self, log_dir: str, *, controller_topology: str = "per-arm",
                 robot_name: str = "lbr_dual_arm_pdz", arm_control: str = "joint_trajectory",
                 gripper_backend: str = "none", gripper_torque: int = GRIPPER_HOLD_TORQUE_DEFAULT,
                 visualize_held_parts: bool = False, world_name: str = "plumbers_block",
                 use_sim_time: bool = False, on_failure: str = "raise"):
        super().__init__("plan_executor_node")

        if controller_topology not in ("per-arm", "combined"):
            raise ValueError(
                f"controller-topology must be per-arm|combined, got {controller_topology!r}")
        if arm_control not in ("joint_trajectory", "cartesian_impedance"):
            raise ValueError(
                f"arm-control must be joint_trajectory|cartesian_impedance, got {arm_control!r}")
        if gripper_backend not in ("none", "sim", "servo"):
            raise ValueError(f"gripper-backend must be none|sim|servo, got {gripper_backend!r}")
        if on_failure not in ("raise", "continue"):
            raise ValueError(f"on-failure must be raise|continue, got {on_failure!r}")

        self._arm_control = arm_control
        self._on_failure = on_failure
        self._visualize_held_parts = visualize_held_parts
        self._gripper_torque = int(np.clip(gripper_torque, 0, GRIPPER_HOLD_TORQUE_MAX))

        # impedance is always per-arm (cartesian_impedance_lbr_{one,two}); --controller-topology
        # only selects the position-JTC wiring.
        self._combined = (arm_control == "joint_trajectory" and controller_topology == "combined")

        if self._combined and gripper_backend == "sim":
            self.get_logger().warn(
                "--gripper-backend sim is meaningless with --controller-topology combined (the "
                "combined joint_trajectory_controller has no gripper joint) -- treating as none.")
            gripper_backend = "none"
        self._gripper_backend = gripper_backend

        if use_sim_time:
            self.set_parameters([Parameter("use_sim_time", value=True)])

        with open(os.path.join(log_dir, "motion.pkl"), "rb") as f:
            self.motion = pickle.load(f)

        # -- live joint feedback (both topologies: measured start positions feed _timed_waypoints)
        self._current_positions = {}
        self.create_subscription(
            JointState, f"/{robot_name}/joint_states", self._joint_state_cb, 10)

        # -- arm action client(s) ---------------------------------------------------------------
        if arm_control == "cartesian_impedance":
            arm_ctrl_name = {a: f"cartesian_impedance_{a}" for a in ARMS}
            gripper_ctrl_name = {a: f"gripper_controller_{a}" for a in ARMS}
        elif self._combined:
            arm_ctrl_name = {a: "joint_trajectory_controller" for a in ARMS}  # shared client
            gripper_ctrl_name = dict(arm_ctrl_name)
        else:
            arm_ctrl_name = {a: f"joint_trajectory_controller_{a}" for a in ARMS}
            gripper_ctrl_name = dict(arm_ctrl_name)

        clients_by_ctrl = {}

        def _client(ctrl, require=True, timeout_sec=20.0):
            if ctrl not in clients_by_ctrl:
                c = ActionClient(
                    self, FollowJointTrajectory,
                    f"/{robot_name}/{ctrl}/follow_joint_trajectory")
                self.get_logger().info(f"Waiting for {ctrl} action server...")
                if not c.wait_for_server(timeout_sec=timeout_sec):
                    if require:
                        raise RuntimeError(
                            f"{ctrl} action server not available after {timeout_sec:.0f}s at "
                            f"/{robot_name}/{ctrl}/follow_joint_trajectory. Check the bringup "
                            f"launch's --controller-topology / --arm-control match this node's, "
                            f"and that the controller spawned -- `ros2 control list_controllers`.")
                    self.get_logger().warn(
                        f"{ctrl} action server not available after {timeout_sec:.0f}s -- "
                        f"running without it; gripper commands skipped (held-part bookkeeping "
                        f"still works).")
                    clients_by_ctrl[ctrl] = None
                    return None
                clients_by_ctrl[ctrl] = c
            return clients_by_ctrl[ctrl]

        self._arm_clients = {a: _client(arm_ctrl_name[a]) for a in ARMS}

        # -- gripper wiring -------------------------------------------------------------------
        self._gripper_clients = {a: None for a in ARMS}
        if gripper_backend == "sim":
            self._gripper_clients = {
                a: _client(gripper_ctrl_name[a], require=False, timeout_sec=5.0) for a in ARMS}
        elif gripper_backend == "servo":
            self._setup_servo_grippers()

        # -- held-part visualization (Gazebo only) ------------------------------------------
        self.traj = None
        self._set_pose_client = None
        if visualize_held_parts:
            from ros_gz_interfaces.srv import SetEntityPose  # lazy: gz-only dependency

            self.traj = np.load(os.path.join(log_dir, "traj.npy"), allow_pickle=True)
            self._world_name = world_name
            self._set_pose_client = self.create_client(
                SetEntityPose, f"/world/{world_name}/set_pose")
            if not self._set_pose_client.wait_for_service(timeout_sec=10.0):
                self.get_logger().warn(
                    f"/world/{world_name}/set_pose not available -- held parts will not visually "
                    "follow the gripper. Continuing (arm/gripper motion unaffected).")

        # -- impedance debug topic (6D task-space error, for goal-tolerance reports) ---------
        self._cart_err = {}
        if arm_control == "cartesian_impedance":
            for a in ARMS:
                self.create_subscription(
                    Float64MultiArray,
                    f"/{robot_name}/cartesian_impedance_{a}/data_impedance",
                    lambda msg, arm=a: self._cart_err.__setitem__(arm, list(msg.data)),
                    10)

        # -- wait for initial joint feedback ------------------------------------------------
        # Wait for both arms' joints regardless of topology -- _timed_waypoints() needs the
        # moving arm's measured start pose, and the combined goal also pins the idle arm.
        self.get_logger().info("Waiting for initial /joint_states...")
        while rclpy.ok() and not all(j in self._current_positions for j in ALL_JOINTS):
            rclpy.spin_once(self, timeout_sec=0.5)

        self.frame_idx = 0
        self.held_part = {"hold": None, "move": None}
        self._last_feedback = None

    # =====================================================================================
    # servo gripper backend (servo_gripper_julien)
    # =====================================================================================
    def _setup_servo_grippers(self):
        if HoldClose is None or SetPosition is None:
            raise RuntimeError(
                "servo_gripper_julien.srv (HoldClose / SetPosition) could not be imported -- "
                "source the gripper workspace (setup_gripper_env.sh) before "
                "--gripper-backend servo.")
        self._gripper_calibrate_clients = {}
        self._gripper_open_clients = {}
        self._gripper_set_position_clients = {}
        self._gripper_hold_close_clients = {}
        self._gripper_stop_clients = {}
        for ns in GRIPPER_NAMESPACES:
            base = f"/{ns}/{GRIPPER_CONTROLLER_NODE}"
            self._gripper_calibrate_clients[ns] = self.create_client(Trigger, f"{base}/calibrate")
            self._gripper_open_clients[ns] = self.create_client(Trigger, f"{base}/open")
            self._gripper_set_position_clients[ns] = self.create_client(
                SetPosition, f"{base}/set_position")
            self._gripper_hold_close_clients[ns] = self.create_client(
                HoldClose, f"{base}/hold_close")
            self._gripper_stop_clients[ns] = self.create_client(Trigger, f"{base}/stop")

        self.get_logger().info(
            f"Waiting for {GRIPPER_CONTROLLER_NODE} calibrate / open / set_position / hold_close "
            f"/ stop on /{GRIPPER_NAMESPACES[0]} and /{GRIPPER_NAMESPACES[1]}...")
        for ns in GRIPPER_NAMESPACES:
            base = f"/{ns}/{GRIPPER_CONTROLLER_NODE}"
            for service, client in (
                    ("calibrate", self._gripper_calibrate_clients[ns]),
                    ("open", self._gripper_open_clients[ns]),
                    ("set_position", self._gripper_set_position_clients[ns]),
                    ("hold_close", self._gripper_hold_close_clients[ns]),
                    ("stop", self._gripper_stop_clients[ns])):
                if not client.wait_for_service(timeout_sec=GRIPPER_HOMING_SERVICE_TIMEOUT_S):
                    raise RuntimeError(
                        f"{base}/{service} not available -- is {GRIPPER_CONTROLLER_NODE} "
                        "running for both sides (source setup_gripper_env.sh, then the "
                        "dual-gripper launch)? `ros2 service list -t | grep gripper_controller`; "
                        f"if the live namespaces are not /{GRIPPER_NAMESPACES[0]} and "
                        f"/{GRIPPER_NAMESPACES[1]}, update GRIPPER_CONTROLLER_NODE / "
                        "GRIPPER_NAMESPACES / ROLE_TO_GRIPPER_NS.")

        # Home both grippers -- ONE cycle per gripper, each stage on both sides at once:
        # calibrate (sweeps the stroke so set_position has open/closed encoder limits), then
        # open (the plan's own starting state). run() then skips the plan's own init entries.
        self.get_logger().info("Homing both grippers before the plan: calibrate, then open...")
        self._call_gripper_all(
            self._gripper_calibrate_clients, Trigger.Request(), "calibrate", "home")
        self._call_gripper_all(self._gripper_open_clients, Trigger.Request(), "open", "home")

    def _spin_sleep(self, seconds: float) -> None:
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, end - time.time())))

    def _call_gripper_all(self, clients_by_ns: dict, request, label: str, context: str) -> None:
        tag = f"{label} ({context})" if context else label
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
                    f"[{ns}] {result.message if result is not None else 'service call timed out'}")
            else:
                self.get_logger().info(f"[{ns}] gripper {tag}: {result.message}")
        if failures:
            raise RuntimeError(f"gripper {tag} failed -- " + "; ".join(failures))

    def _call_gripper_trigger(self, gripper: str, client, service_name: str,
                              context: str = "") -> None:
        tag = f"{service_name} ({context})" if context else service_name
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=GRIPPER_MOTION_TIMEOUT_S)
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError(
                f"[{gripper}] gripper {tag} failed: "
                f"{result.message if result is not None else 'service call timed out'}")
        self.get_logger().info(f"[{gripper}] gripper {tag}: {result.message}")

    def _call_set_position(self, gripper: str, open_ratio: float, context: str) -> None:
        position = float(np.clip(1.0 - open_ratio, 0.0, 1.0))  # srv: 0.0=open .. 1.0=closed
        req = SetPosition.Request()
        req.position = position
        future = self._gripper_set_position_clients[gripper].call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=GRIPPER_MOTION_TIMEOUT_S)
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError(
                f"[{gripper}] gripper set_position({position:.3f}) ({context}) failed: "
                f"{result.message if result is not None else 'service call timed out'}")
        self.get_logger().info(
            f"[{gripper}] gripper set_position({position:.3f}, open_ratio={open_ratio:.3f}, "
            f"{context}): {result.message}")

    def _call_hold_close(self, gripper: str, context: str) -> None:
        req = HoldClose.Request()
        req.torque_limit = self._gripper_torque
        future = self._gripper_hold_close_clients[gripper].call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=GRIPPER_MOTION_TIMEOUT_S)
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError(
                f"[{gripper}] gripper hold_close ({context}) failed: "
                f"{result.message if result is not None else 'service call timed out'}")
        self.get_logger().info(
            f"[{gripper}] gripper hold_close ({context}, torque_limit={self._gripper_torque}): "
            f"{result.message}")

    def _release_grippers(self, reason: str) -> None:
        """~/stop on both grippers (best effort). Used on an aborted run so the servos aren't
        left gripping. No-op unless the servo backend is active."""
        if self._gripper_backend != "servo":
            return
        self.get_logger().warn(f"{reason} -- calling ~/stop on both grippers to release torque")
        for ns in GRIPPER_NAMESPACES:
            try:
                self._call_gripper_trigger(ns, self._gripper_stop_clients[ns], "stop", reason)
            except Exception as exc:  # noqa: BLE001 -- best-effort cleanup
                self.get_logger().error(f"[{ns}] gripper stop during abort failed: {exc}")

    def _gripper_close_is_grasp(self, idx: int, role: str) -> bool:
        """A description=='close' entry is a real grasp (needs ~/hold_close) if either: (a) an
        arm segment before the next gripper entry names an active_part; or (b) a later gripper
        entry for the SAME role is a description=='open'. Neither -> a non-grasp park close."""
        tail = self.motion[idx + 1:]
        for _mt, body_type, _value, part, _desc in tail:
            if body_type == "gripper":
                break
            if body_type == "arm" and part is not None:
                return True
        for mt, body_type, _value, _part, desc in tail:
            if body_type == "gripper" and mt == role and desc == "open":
                return True
        return False

    # =====================================================================================
    # held-part following (viz)
    # =====================================================================================
    def _joint_state_cb(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            if name in ARM_JOINTS["lbr_one"] or name in ARM_JOINTS["lbr_two"]:
                self._current_positions[name] = position

    def _publish_held_part_pose(self):
        if self._set_pose_client is None or self.traj is None:
            return
        from scipy.spatial.transform import Rotation  # lazy: gz-only dependency

        from ros_gz_interfaces.srv import SetEntityPose
        for role, part_id in self.held_part.items():
            if part_id is None or self.frame_idx >= len(self.traj):
                continue
            T = self.traj[self.frame_idx][f"part{part_id}"]
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

    # =====================================================================================
    # goal-tolerance violation reporting
    # =====================================================================================
    def _cartesian_error_str(self, arm: str):
        d = self._cart_err.get(arm)
        if not d or len(d) < 6:
            return None
        px, py, pz, rx, ry, rz = d[:6]
        pos_mm = np.array([px, py, pz]) * 1000.0
        rot_deg = np.degrees([rx, ry, rz])
        trans_norm_mm = float(np.linalg.norm(pos_mm))
        rot_norm_deg = float(np.linalg.norm(rot_deg))
        sixd_norm = float(np.linalg.norm([px, py, pz, rx, ry, rz]))
        return (
            "cartesian error (impedance target - EE, base frame): "
            f"|trans|={trans_norm_mm:.3f} mm [dx={pos_mm[0]:+.3f}, dy={pos_mm[1]:+.3f}, "
            f"dz={pos_mm[2]:+.3f} mm]; |rot|={rot_norm_deg:.3f} deg [rx={rot_deg[0]:+.3f}, "
            f"ry={rot_deg[1]:+.3f}, rz={rot_deg[2]:+.3f} deg]; 6D vector (m,rad)=["
            f"{px:+.5f}, {py:+.5f}, {pz:+.5f}, {rx:+.5f}, {ry:+.5f}, {rz:+.5f}] "
            f"||6D||={sixd_norm:.5f}")

    def _goal_tol_violation_report(self, arm, joint_names, points, result) -> str:
        parts = []
        fb = getattr(self, "_last_feedback", None)
        target = dict(zip(joint_names, points[-1].positions)) if points else {}
        if fb is not None and len(fb.actual.positions):
            fb_names = list(fb.joint_names) or list(joint_names)
            actual = dict(zip(fb_names, fb.actual.positions))
            miss = {n: target[n] - actual[n] for n in target if n in actual}
            if miss:
                pairs = sorted(miss.items(), key=lambda p: abs(p[1]), reverse=True)
                max_abs = max(abs(v) for v in miss.values())
                per_joint = ", ".join(
                    f"{n}={v:+.5f} rad ({np.degrees(v):+.3f} deg)" for n, v in pairs)
                parts.append(
                    f"goal tolerance violation (commanded final point - measured): "
                    f"max |miss|={max_abs:.5f} rad ({np.degrees(max_abs):.3f} deg); "
                    f"per joint [{per_joint}]")
            track = list(fb.error.positions)
            if track and any(abs(e) > 1e-4 for e in track):
                te = max(abs(e) for e in track)
                parts.append(
                    f"setpoint tracking error at abort: max |err|={te:.5f} rad "
                    f"({np.degrees(te):.3f} deg)")
        else:
            parts.append("no trajectory feedback received -- per-joint miss unavailable")
        cart = self._cartesian_error_str(arm)
        if cart:
            parts.append(cart)
        err_str = "" if result is None else (result.result.error_string or "").strip()
        if err_str:
            parts.append(f"controller: {err_str}")
        return "; ".join(parts)

    # =====================================================================================
    # trajectory timing / execution
    # =====================================================================================
    def _timed_waypoints(self, moving_arm: str, current_position: np.ndarray, path: np.ndarray):
        """Per-waypoint (time_from_start, velocity) for `path`, stretching (never shortening)
        the nominal 1/FPS spacing of any segment -- including the synthetic current-pose ->
        path[0] one -- whose implied per-joint velocity would exceed
        JOINT_MAX_VELOCITY_RAD_S * BLEND_VELOCITY_SAFETY_FACTOR."""
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
                f"[{moving_arm}] blending oversized jump before waypoint {i} ({joint}: "
                f"{np.degrees(gaps[i, np.argmax(per_joint_dt[i])]):+.1f} deg) -- stretching this "
                f"segment to {seg_dt[i] * 1000:.0f} ms (nominal {nominal_dt * 1000:.0f} ms)")

        times = np.cumsum(seg_dt)
        velocities = np.zeros_like(path)
        if n > 1:
            velocities[1:-1] = (path[2:] - path[:-2]) / (times[2:] - times[:-2])[:, np.newaxis]
            velocities[0] = (path[1] - path[0]) / seg_dt[1]
        # velocities[-1] stays 0.0: decelerate to a stop at this entry's goal boundary.
        return times, velocities

    def _run_goal(self, arm: str, action_client, joint_names, points, *,
                  n_frames_to_advance: int = 0, tolerate_goal_tol: bool = False):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        goal.trajectory.points = points
        goal.goal_time_tolerance.sec = 5

        self._last_feedback = None
        send_future = action_client.send_goal_async(
            goal, feedback_callback=lambda msg: setattr(self, "_last_feedback", msg.feedback))
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            if self._on_failure == "raise":
                raise RuntimeError(f"[{arm}] goal rejected by the controller")
            self.get_logger().error(f"[{arm}] goal rejected")
            return
        result_future = goal_handle.get_result_async()

        # Tick frame_idx + held-part pose at ~30 Hz while the controller executes, so a
        # teleported part stays in sync with the arm's real progress. Viz only.
        if self._visualize_held_parts and n_frames_to_advance > 0:
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
            code = None if result is None else result.result.error_code
            report = self._goal_tol_violation_report(arm, joint_names, points, result)
            if tolerate_goal_tol and code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
                self.get_logger().warn(
                    f"[{arm}] goal tolerance not met (impedance control, expected) -- "
                    f"continuing. {report}")
            elif self._on_failure == "raise":
                raise RuntimeError(
                    f"[{arm}] trajectory execution failed (error_code={code}). {report}")
            else:
                self.get_logger().error(
                    f"[{arm}] trajectory execution failed (error_code={code}). {report}")

    # =====================================================================================
    # plan walk
    # =====================================================================================
    def _send_arm_path(self, role: str, path: np.ndarray, description: str, active_part):
        moving_arm = ROLE_TO_ARM[role]

        # Refresh /joint_states so the positions below are measured, not a cached copy of what a
        # prior goal commanded (real/mock joints settle a few tenths of a degree off a setpoint;
        # _timed_waypoints() needs the true gap to path[0], else the controller snaps across the
        # settle error in one tick).
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.02)
        current_position = np.array([self._current_positions[j] for j in ARM_JOINTS[moving_arm]])

        idle_positions = None
        if self._combined:
            idle_arm = "lbr_two" if moving_arm == "lbr_one" else "lbr_one"
            idle_positions = [self._current_positions[j] for j in ARM_JOINTS[idle_arm]]

        times, velocities = self._timed_waypoints(moving_arm, current_position, path)

        points = []
        for q, v, t in zip(path, velocities, times):
            p = JointTrajectoryPoint()
            if self._combined:
                if moving_arm == "lbr_one":
                    p.positions = [float(x) for x in q] + idle_positions
                    p.velocities = [float(x) for x in v] + [0.0] * 7
                else:
                    p.positions = idle_positions + [float(x) for x in q]
                    p.velocities = [0.0] * 7 + [float(x) for x in v]
            else:
                p.positions = [float(x) for x in q]
                p.velocities = [float(x) for x in v]
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t % 1.0) * 1e9)
            points.append(p)

        joint_names = ALL_JOINTS if self._combined else ARM_JOINTS[moving_arm]
        msg = f"[{moving_arm}] {description} ({len(path)} waypoints, part={active_part})"
        if self._combined:
            msg += f", holding {idle_arm} at its current position"
        self.get_logger().info(msg)
        self._run_goal(moving_arm, self._arm_clients[moving_arm], joint_names, points,
                       n_frames_to_advance=len(path),
                       tolerate_goal_tol=(self._arm_control == "cartesian_impedance"))
        # Deliberately NOT caching path[-1] -- the next _send_arm_path() re-reads live
        # /joint_states so the real settle error is visible to _timed_waypoints() and blended.

    def _send_gripper_command(self, role: str, open_ratio: float, description: str,
                              active_part, is_grasp: bool):
        arm = ROLE_TO_ARM[role]
        if self._gripper_backend == "servo":
            self._send_gripper_command_servo(role, open_ratio, description, active_part, is_grasp)
        elif self._gripper_backend == "sim":
            self._send_gripper_command_sim(arm, open_ratio, description)
        else:
            self.get_logger().info(
                f"[{arm}] gripper {description} (open_ratio={open_ratio:.3f}, part={active_part}) "
                f"skipped -- --gripper-backend none")

        # Held-part bookkeeping (all backends) -- drives the traj.npy teleport when viz is on.
        if description == "close" and active_part is not None:
            self.held_part[role] = active_part
            self.get_logger().info(f"[{arm}] now holding part {active_part}")
        elif description == "open" and self.held_part[role] is not None:
            self.get_logger().info(f"[{arm}] released part {self.held_part[role]}")
            self.held_part[role] = None

    def _send_gripper_command_sim(self, arm: str, open_ratio: float, description: str):
        position = gripper_open_ratio_to_position(open_ratio)
        client = self._gripper_clients[arm]
        if client is None:
            self.get_logger().warn(
                f"[{arm}] gripper {description} (open_ratio={open_ratio:.3f} -> "
                f"{position * 1000:.1f}mm) skipped -- no gripper controller available")
            return
        p = JointTrajectoryPoint()
        p.positions = [position]  # left driving joint only -- right is a mimic
        p.time_from_start.sec = 0
        p.time_from_start.nanosec = int(0.5e9)
        self.get_logger().info(
            f"[{arm}] gripper {description} open_ratio={open_ratio:.3f} "
            f"-> {position * 1000:.1f}mm")
        self._run_goal(arm, client, [GRIPPER_JOINTS[arm]], [p], n_frames_to_advance=1)

    def _send_gripper_command_servo(self, role: str, open_ratio: float, description: str,
                                    active_part, is_grasp: bool):
        gripper = ROLE_TO_GRIPPER_NS[role]
        if description == "close" and is_grasp:
            self.get_logger().info(
                f"[{gripper}] gripper close (part={active_part}, open_ratio={open_ratio:.3f}) "
                f"-> ~/hold_close torque_limit={self._gripper_torque} [grasp]")
            self._call_hold_close(gripper, description)
            self._spin_sleep(GRIPPER_CLOSE_SETTLE_S)
        elif description == "open" and open_ratio >= GRIPPER_FULL_OPEN_RATIO:
            self.get_logger().info(
                f"[{gripper}] gripper open (open_ratio={open_ratio:.3f}) -> ~/open [full open]")
            self._call_gripper_trigger(
                gripper, self._gripper_open_clients[gripper], "open", description)
        else:
            self.get_logger().info(
                f"[{gripper}] gripper {description} (part={active_part}, "
                f"open_ratio={open_ratio:.3f}) -> ~/set_position [no torque]")
            self._call_set_position(gripper, open_ratio, description)

    def run(self):
        for idx, (motion_type, body_type, value, active_part, description) in enumerate(
                self.motion):
            role = motion_type
            if body_type == "arm":
                self._send_arm_path(role, np.asarray(value), description, active_part)
            elif body_type == "gripper":
                if description == "init" and self._gripper_backend == "servo":
                    # Startup homing already established a known open state -- replaying the
                    # plan's own init entries here just homes the grippers again.
                    self.get_logger().info(
                        f"[{ROLE_TO_GRIPPER_NS[role]}] gripper init entry skipped -- startup "
                        "homing already established a known state")
                    continue
                is_grasp = (description == "close" and self._gripper_close_is_grasp(idx, role))
                self._send_gripper_command(role, float(value), description, active_part, is_grasp)
            else:
                self.get_logger().warn(f"Unknown body_type {body_type}, skipping")
        self.get_logger().info("Plan complete.")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "log_dir", nargs="?", default=os.path.expanduser("~/Fabrica/logs/plumbers_block_sim"),
        help="directory containing motion.pkl (and traj.npy for --visualize-held-parts) "
             "(default: %(default)s)")
    parser.add_argument(
        "--controller-topology", choices=["per-arm", "combined"], default="per-arm",
        help="per-arm (default, Gazebo): joint_trajectory_controller_lbr_{one,two}. combined "
             "(real/mock FRI): one 14-joint joint_trajectory_controller.")
    parser.add_argument(
        "--robot-name", default="lbr_dual_arm_pdz",
        help="controller_manager / joint_states namespace (default: %(default)s; the hardware "
             "shim uses lbr_dual_arm).")
    parser.add_argument(
        "--arm-control", choices=["joint_trajectory", "cartesian_impedance"],
        default="joint_trajectory",
        help="cartesian_impedance replays against per-arm cartesian_impedance_lbr_{one,two} "
             "(always per-arm, no --controller-topology effect).")
    parser.add_argument(
        "--gripper-backend", choices=["none", "sim", "servo"], default="none",
        help="none (default): arm-only. sim (Gazebo): simulated pdz finger joint. servo (real "
             "hardware): servo_gripper_julien services -- REAL hardware, no sim variant; needs "
             "setup_gripper_env.sh sourced.")
    parser.add_argument(
        "--gripper", action="store_true",
        help="back-compat alias for --gripper-backend servo (when --gripper-backend is left at "
             "none).")
    parser.add_argument(
        "--gripper-torque", type=int, default=GRIPPER_HOLD_TORQUE_DEFAULT,
        metavar=f"0..{GRIPPER_HOLD_TORQUE_MAX}",
        help="torque_limit for every grasp hold_close (servo backend only; default %(default)s).")
    parser.add_argument(
        "--visualize-held-parts", action="store_true",
        help="Gazebo only: teleport the held part to match traj.npy via /world/<world>/set_pose.")
    parser.add_argument(
        "--world", default="plumbers_block",
        help="gz-sim world name for --visualize-held-parts (default: %(default)s).")
    parser.add_argument("--use-sim-time", action="store_true",
                        help="set the node's use_sim_time param (Gazebo).")
    parser.add_argument(
        "--on-failure", choices=["raise", "continue"], default="raise",
        help="raise (default): stop the plan on any rejected/failed goal. continue: log and go "
             "on. GOAL_TOLERANCE_VIOLATED under cartesian_impedance is always tolerated.")
    return parser


def main(args=None):
    rclpy.init(args=args)
    cli_args = remove_ros_args(args=sys.argv)[1:] if args is None else list(args)
    parsed = _build_parser().parse_args(cli_args)

    gripper_backend = parsed.gripper_backend
    if parsed.gripper and gripper_backend == "none":
        gripper_backend = "servo"

    node = PlanExecutor(
        parsed.log_dir,
        controller_topology=parsed.controller_topology,
        robot_name=parsed.robot_name,
        arm_control=parsed.arm_control,
        gripper_backend=gripper_backend,
        gripper_torque=parsed.gripper_torque,
        visualize_held_parts=parsed.visualize_held_parts,
        world_name=parsed.world,
        use_sim_time=parsed.use_sim_time,
        on_failure=parsed.on_failure,
    )
    try:
        node.run()
    except BaseException as exc:  # incl. KeyboardInterrupt -- don't leave the servos gripping
        node._release_grippers(f"run aborted ({type(exc).__name__})")
        raise
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
