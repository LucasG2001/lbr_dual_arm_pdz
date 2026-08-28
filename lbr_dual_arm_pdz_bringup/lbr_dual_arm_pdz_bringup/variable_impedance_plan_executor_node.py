"""Replays a Fabrica motion plan (see plan_executor_node.py's docstring for motion.pkl's
format) on the dual-arm KUKA under Cartesian-impedance control, VARYING the task-space
stiffness per motion phase: stiff while free-flying and while approaching a pre-grasp /
pre-insertion pose, soft along the approach/insertion axis while actually mating, stiff
again while carrying. This is a third executor alongside:

  - plan_executor_node.py           -- Gazebo, position OR cartesian_impedance, fixed gains
  - hardware_plan_executor_node.py  -- real/mock FRI, one combined joint_trajectory_controller

and it targets the SAME cartesian_impedance_controller as plan_executor_node.py's
`--arm-control cartesian_impedance` path (per-arm cartesian_impedance_lbr_one/_two, effort
interface, FollowJointTrajectory -- the patched idra-lab/ros2_effort_controller in
~/franka_ros2_ws/src/kuka_lbr_control). The user has confirmed the Gazebo and the real-rig
controller are the same build, so this node is rig-agnostic: point it at whichever
controller_manager namespace is up (Gazebo gazebo.launch.py arm_control:=cartesian_impedance,
or the real cartesian_impedance.launch.py).

======================================================================================
HOW THE PHASES / STIFFNESS SCHEDULE ARE DERIVED  (nothing in motion.pkl is labelled with
these -- they are reconstructed here)
======================================================================================
motion.pkl is the flat [motion_type, body_type, value, active_part, description] timeline.
For every ARM entry this node assigns one or more (sub-path, TAG) pairs:

  init              -- description=='init'. Sent as a linear constant-velocity joint blend
                       from the arm's MEASURED current pose to rest_q (see _init_waypoints).
  pregrasp_approach -- the LEADING part of a switch/transport that is immediately followed
                       (before the same arm moves again) by a real-grasp gripper 'close'
                       (classified by _gripper_close_is_grasp(), same logic as the hardware
                       node). Stiff: nail the pre-grasp pose.
  grasp_approach    -- the last N_PREGRASP_WP waypoints of that same switch/transport, i.e.
                       roughly the final D_PREGRASP_M metres of straight-line descent onto
                       the grasp. Soft (esp. Z): comply while closing the last gap.
                       (The gripper 'close' entry itself sends no arm goal -- the arm just
                       holds here at this soft stiffness while the jaws close.)
  lift              -- the first N_LIFT_WP waypoints (~D_LIFT_M m) of the transport that
                       carries the part right after a real grasp. Stiffness ramped back up.
  transport_carry   -- the rest of any part-carrying transport. Stiff: track the plan
                       rigidly despite the payload.
  insertion         -- description=='assembly'. Soft along the insertion axis (see below),
                       stiff perpendicular. This is the compliant mate.
  retract           -- a part=None transport right after this arm released a part. Medium.
  free              -- everything else (returns to rest, re-orientation switches). Stiff.

DIRECTIONAL SOFTENING.  The cartesian_impedance_controller applies stiffness.trans_*/rot_*
in the END-EFFECTOR frame -- `compliance_ref_link` in the yaml is DECLARED BUT ITS
IMPLEMENTATION IS COMMENTED OUT (effort_controller_base.cpp ~L156-188); the control loop
always does displayInBaseLink(m_cartesian_stiffness, m_end_effector_link). Fabrica builds
every pickup as a top-down straight line and every insertion in plumbers_block as a
world-axis-aligned straight line, and orients the tool approach axis (EE +Z) along that
line. So softening stiffness.trans_z (EE frame) IS "soft along the approach/insertion
axis" for every grasp_approach and insertion segment in this plan -- no world<->EE rotation
math, no set_cartesian_impedance 6x6 service. If you retarget to a plan whose tool is NOT
aligned with the mate axis, that assumption breaks; the post-segment Cartesian-error log
(below) is what would show it (lateral compliance where you didn't want it).

RE-GAIN DWELLS.  The controller EMA-blends applied gains toward the parameter target at
impedance_blend_alpha (0.1 -> ~30 ms to 95% at 1 kHz). A gain change only produces a force
transient if K*error changes while it blends, i.e. only if the arm is moving / holding a
pose error during the blend. So whenever the tag (hence the stiffness) changes, this node:
  1. finishes the previous sub-goal (arm at rest),
  2. pushes the 6 stiffness params in one set_parameters call,
  3. holds still for DWELL_S (>> blend settle) -- spin, no motion,
  4. sends the next sub-goal.
Cost ~DWELL_S per tag change (~12-16 in the plan). This is the "simple, per-tag, sent
once, controller blends" scheme.

HOLDER ARM.  Between its own manoeuvres the hold arm (lbr_one) is not sent trajectories --
with command_current_configuration:true the controller holds it against a FIXED impedance
target at whatever stiffness its parameters currently say. This node:
  - sets it to HOLDER_STIFF at startup and re-asserts HOLDER_STIFF after any hold-role
    manoeuvre that leaves it holding station;
  - raises it to HOLDER_BRACE_STIFF for the duration of the move arm's insertion segments,
    then restores it;
  - keeps a null-space posture on it (nullspace_desired_configuration = its current nominal
    joint config, nullspace_stiffness = HOLDER_NULLSPACE_STIFFNESS), refreshed every time
    that nominal config changes, so the elbow can't wander while idle-holding;
  - re-commands it the NOMINAL joint config (a short 1-point goal, not its measured/sagged
    pose) right before each move-arm insertion, to re-seat the spring and clear stiction
    slop before the insertion reaction loads it.
A fixed target + finite stiffness + no integral term still leaves a standing, load-
proportional pose error under the held assembly's weight (= wrench / K); removing THAT would
need feed-forward of the predicted deflection, which this node does not do. Per the user's
brief: "holder stays where it is, just increase stiffness" -- plus the null-space / re-seat
mitigations above for the non-recovering part.

GOAL TOLERANCE.  Under compliance the controller aborts a FollowJointTrajectory with
GOAL_TOLERANCE_VIOLATED whenever a joint doesn't settle inside trajectory_default_goal_
tolerance (0.02 rad) within the goal-time window -- expected on soft tags. For tags in
TOLERATE_TAGS this node logs the full miss and continues; for every other tag, or any other
error_code, or a rejected goal, it RAISES and stops the plan (real-hardware default, same
as hardware_plan_executor_node.py). Either way, on any non-SUCCESS result the log carries:
  - per-joint (commanded final point - measured) in rad and deg, plus the max,
  - the running setpoint-tracking error at abort,
  - the 6D task-space error (|trans| mm, |rot| deg, and the full [x y z rx ry rz] vector)
    from the arm's cartesian_impedance_<arm>/data_impedance topic,
  - the controller's own error_string.
For the compliant tags the same joint+Cartesian residual is ALSO logged after a SUCCESSFUL
segment, so you always see how far the compliant arm actually ended from the nominal goal.

GRIPPER.  Identical opt-in --gripper handling to hardware_plan_executor_node.py: the real
servo_gripper_julien services (binary open / set_position / hold_close / stop), per-side
namespaces, startup homing, real grasps via hold_close, everything else via set_position,
~/stop on abort. Without --gripper the grippers are untouched and plan gripper entries are
logged and skipped. See hardware_plan_executor_node.md for the service interface.

WAYPOINT TIMING.  Plain time_from_start = (i+1)/FPS per sub-path (like plan_executor_node.py).
The hardware node's _timed_waypoints velocity blending is NOT reproduced: that guarded
lbr_fri_ros2::CommandGuard's POSITION-mode per-joint velocity ceiling; in effort/impedance
mode the command interface is torque, rate-limited by the controller's delta_tau_max, and a
position jump becomes a bounded spring force (max_impedance_force / max_impedance_torque),
not a guard trip. The 'init' sub-goal instead blends linearly (constant joint velocity,
INIT_MOVE_SPEED_RAD_S on the fastest joint, waypoint count scaled by distance) from the
measured current pose to rest_q, so a rig parked far from Fabrica's rest_q neither lurches
nor implies an arbitrary speed.

Usage:
    # bring up cartesian_impedance on both arms first, e.g.
    ros2 launch lbr_dual_arm_pdz_bringup gazebo.launch.py arm_control:=cartesian_impedance
    #   or the real rig:  ros2 launch lbr_dual_arm_bringup cartesian_impedance.launch.py
    source setup_gripper_env.sh                       # only if using --gripper
    python3 variable_impedance_plan_executor_node.py [log_dir] [robot_name] \
        [--gripper] [--gripper-torque N] [--dwell S] [--no-holder-brace]
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
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

# servo_gripper_julien custom srvs -- only needed for --gripper; tolerate-import so an
# arm-only run doesn't require the gripper workspace on the path (see __init__).
try:
    from servo_gripper_julien.srv import HoldClose, SetPosition
except ImportError:
    HoldClose = None
    SetPosition = None

FPS = 30.0

ROLE_TO_ARM = {'hold': 'lbr_one', 'move': 'lbr_two'}
HOLD_ARM = 'lbr_one'
ARM_JOINTS = {
    'lbr_one': [f'lbr_one_A{i}' for i in range(1, 8)],
    'lbr_two': [f'lbr_two_A{i}' for i in range(1, 8)],
}

# ======================================================================================
# TUNING KNOBS -- the whole variable-impedance schedule lives here.
# Stiffness is (trans_x, trans_y, trans_z) N/m and (rot_x, rot_y, rot_z) Nm/rad, in the
# END-EFFECTOR frame. trans_z is the tool approach/insertion axis for every grasp and
# insertion in plumbers_block (see module docstring).
# ======================================================================================
STIFFNESS_TABLE = {
    'init':              ((1500.0, 1500.0, 1500.0), (75.0, 75.0, 75.0)),
    'free':              ((2000.0, 2000.0, 2000.0), (100.0, 100.0, 100.0)),
    'pregrasp_approach': ((2500.0, 2500.0, 2500.0), (100.0, 100.0, 100.0)),
    'grasp_approach':    ((1200.0, 1200.0,  300.0), (40.0, 40.0, 40.0)),
    'lift':              ((2000.0, 2000.0, 2000.0), (60.0, 60.0, 60.0)),
    'transport_carry':   ((2000.0, 2000.0, 2000.0), (100.0, 100.0, 100.0)),
    'insertion':         ((2000.0, 2000.0,  150.0), (75.0, 75.0, 75.0)),
    'retract':           ((1500.0, 1500.0, 1500.0), (75.0, 75.0, 75.0)),
}
HOLDER_STIFF = ((3000.0, 3000.0, 3000.0), (280.0, 280.0, 280.0))
HOLDER_BRACE_STIFF = ((3600.0, 3600.0, 3600.0), (330.0, 330.0, 330.0))
# The hold arm holds station between its own manoeuvres via command_current_configuration
# (fixed impedance target), so a constant unmodelled disturbance -- the held assembly's
# weight, the other arm's insertion reaction -- leaves a standing pose error (= wrench / K,
# no integral term) plus non-recovering joint stiction / null-space wander. Two mitigations,
# both aimed at the hold arm only:
#   * a null-space posture (nullspace_desired_configuration = the hold arm's current nominal
#     joint config, nullspace_stiffness = HOLDER_NULLSPACE_STIFFNESS) so the elbow can't
#     migrate while idle-holding -- pushed whenever the nominal config changes. Dormant
#     during the hold arm's own trajectories (the controller uses trajectory_nullspace_
#     stiffness + the planned config then).
#   * re-commanding the hold arm its NOMINAL joint config (not its measured, sagged pose) as
#     a short 1-point goal right before each move-arm insertion, to re-energise the spring
#     toward the intended pose and pull back any stiction slop before the insertion pushes
#     on it. Does NOT remove the payload steady-state error -- feed-forward would be needed
#     for that.
HOLDER_NULLSPACE_STIFFNESS = 20.0
HOLDER_RECOMMAND_S = 1.0

# Tags whose FollowJointTrajectory GOAL_TOLERANCE_VIOLATED result is expected (compliant
# arm) -- log the miss and continue instead of raising.
TOLERATE_TAGS = {'grasp_approach', 'insertion'}

# Straight-line standoff distances used to SPLIT an existing resampled path (no IK -- see
# "path-split" in the design discussion). Converted to a waypoint count via the planner's
# nominal segment speed (planning/run_motion_plan.py max_speed: 4 cm/s switch & transport,
# resampled at 30 fps -> ~1.33 mm per waypoint). If a path is shorter than the split count
# the node clamps to half the path and warns.
D_PREGRASP_M = 0.05
D_LIFT_M = 0.03
V_SEGMENT_M_S = 0.04
N_PREGRASP_WP = int(np.ceil(D_PREGRASP_M * FPS / V_SEGMENT_M_S))   # ~38
N_LIFT_WP = int(np.ceil(D_LIFT_M * FPS / V_SEGMENT_M_S))           # ~23

# Hold-still time after a stiffness change, so the controller's EMA gain blend finishes
# with the arm stationary (no K*error transient). >> the ~30 ms blend settle.
DWELL_S = 0.3
# The 'init' sub-goal is a linear (constant-velocity) joint-space blend from the arm's
# MEASURED current pose to Fabrica's rest_q: this many rad/s on the fastest-moving joint,
# with the waypoint count scaled by distance (further start -> more waypoints, same speed).
INIT_MOVE_SPEED_RAD_S = 0.2
INIT_MIN_WAYPOINTS = 2

# --- servo_gripper_julien service interface (see hardware_plan_executor_node.md) ---------
GRIPPER_CONTROLLER_NODE = 'gripper_controller'
GRIPPER_NAMESPACES = ('left', 'right')
GRIPPER_MOTION_TIMEOUT_S = 30.0
GRIPPER_HOMING_SERVICE_TIMEOUT_S = 10.0
GRIPPER_FULL_OPEN_RATIO = 0.98
GRIPPER_HOLD_TORQUE_DEFAULT = 250
GRIPPER_HOLD_TORQUE_MAX = 500
GRIPPER_CLOSE_SETTLE_S = 10.0
# Plan role -> gripper SIDE namespace. Must match however the dual-gripper launch assigns
# left/right to the physical arms (the two grippers are identical, so the mapping itself is
# arbitrary). Decoupled from ROLE_TO_ARM on purpose.
ROLE_TO_GRIPPER_NS = {'hold': 'left', 'move': 'right'}


class VariableImpedancePlanExecutor(Node):
    def __init__(self, log_dir, robot_name='lbr_dual_arm', gripper=False,
                 gripper_torque=GRIPPER_HOLD_TORQUE_DEFAULT, dwell_s=DWELL_S,
                 holder_brace=True):
        super().__init__('variable_impedance_plan_executor_node')
        self._robot_name = robot_name
        self._dwell_s = float(dwell_s)
        self._holder_brace = bool(holder_brace)
        self._gripper_enabled = gripper
        self._gripper_torque = int(np.clip(gripper_torque, 0, GRIPPER_HOLD_TORQUE_MAX))
        if gripper and self._gripper_torque != gripper_torque:
            self.get_logger().warn(
                f'--gripper-torque {gripper_torque} out of range 0..{GRIPPER_HOLD_TORQUE_MAX}, '
                f'clamped to {self._gripper_torque}')

        with open(os.path.join(log_dir, 'motion.pkl'), 'rb') as f:
            self.motion = pickle.load(f)

        self._last_feedback = None
        self._cart_err = {}   # arm -> latest data_impedance list (data[0:6] = 6D pose error)
        self._cur_stiff = {}  # arm -> ((trans3), (rot3)) currently commanded
        self._current_positions = {}  # joint name -> latest measured position (for 'init' blend)
        self._holder_nominal_q = None  # hold arm's intended joint config while holding station
        self.create_subscription(
            JointState, f'/{robot_name}/joint_states', self._joint_state_cb, 10)

        # Per-arm FollowJointTrajectory clients + stiffness parameter clients + the
        # controller's live task-space error topic.
        self._arm_clients = {}
        self._param_clients = {}
        for arm in ('lbr_one', 'lbr_two'):
            ctrl = f'cartesian_impedance_{arm}'
            self._arm_clients[arm] = ActionClient(
                self, FollowJointTrajectory,
                f'/{robot_name}/{ctrl}/follow_joint_trajectory')
            self._param_clients[arm] = self.create_client(
                SetParameters, f'/{robot_name}/{ctrl}/set_parameters')
            self.create_subscription(
                Float64MultiArray, f'/{robot_name}/{ctrl}/data_impedance',
                lambda msg, a=arm: self._cart_err.__setitem__(a, list(msg.data)), 10)

        for arm in ('lbr_one', 'lbr_two'):
            ctrl = f'cartesian_impedance_{arm}'
            self.get_logger().info(f'Waiting for {ctrl} action server...')
            if not self._arm_clients[arm].wait_for_server(timeout_sec=20.0):
                raise RuntimeError(
                    f'/{robot_name}/{ctrl}/follow_joint_trajectory not available -- is the '
                    f'cartesian_impedance controller for {arm} spawned? '
                    f'`ros2 control list_controllers`')
            self.get_logger().info(f'Waiting for {ctrl}/set_parameters service...')
            if not self._param_clients[arm].wait_for_service(timeout_sec=20.0):
                raise RuntimeError(
                    f'/{robot_name}/{ctrl}/set_parameters not available -- cannot set '
                    f'stiffness at runtime.')

        self.get_logger().info('Waiting for initial /joint_states...')
        all_arm_joints = ARM_JOINTS['lbr_one'] + ARM_JOINTS['lbr_two']
        deadline = time.time() + 20.0
        while rclpy.ok() and not all(j in self._current_positions for j in all_arm_joints):
            if time.time() > deadline:
                raise RuntimeError(
                    f'/{robot_name}/joint_states did not report all 14 arm joints within 20s '
                    f'(have {sorted(self._current_positions)}) -- is joint_state_broadcaster up?')
            rclpy.spin_once(self, timeout_sec=0.2)

        # Baseline gains: hold arm firm, move arm nominal. First real sub-goal adjusts.
        self._apply_stiffness('lbr_one', HOLDER_STIFF, 'startup (hold arm baseline)')
        self._apply_stiffness('lbr_two', STIFFNESS_TABLE['free'], 'startup (move arm baseline)')

        self._setup_gripper(robot_name)
        self._schedule = self._build_schedule()
        self.get_logger().info(
            f'Built schedule: {len(self._schedule)} steps from {len(self.motion)} motion.pkl '
            f'entries (dwell {self._dwell_s:.2f}s, holder brace '
            f'{"on" if self._holder_brace else "off"}).')

    # ---------------------------------------------------------------- stiffness / dwell ---
    def _apply_stiffness(self, arm, stiff, context):
        """Push the 6 stiffness params in one set_parameters call (rcl_interfaces/srv/
        SetParameters -- rclpy has no AsyncParameterClient on Humble). Raises on failure.
        The controller re-reads stiffness.* every control cycle (updateGainsFromParameters)
        and EMA-blends the applied gains toward them; _ensure_stiffness() dwells after."""
        trans, rot = stiff
        req = SetParameters.Request()
        req.parameters = [
            Parameter('stiffness.trans_x', Parameter.Type.DOUBLE, float(trans[0])).to_parameter_msg(),
            Parameter('stiffness.trans_y', Parameter.Type.DOUBLE, float(trans[1])).to_parameter_msg(),
            Parameter('stiffness.trans_z', Parameter.Type.DOUBLE, float(trans[2])).to_parameter_msg(),
            Parameter('stiffness.rot_x', Parameter.Type.DOUBLE, float(rot[0])).to_parameter_msg(),
            Parameter('stiffness.rot_y', Parameter.Type.DOUBLE, float(rot[1])).to_parameter_msg(),
            Parameter('stiffness.rot_z', Parameter.Type.DOUBLE, float(rot[2])).to_parameter_msg(),
        ]
        future = self._param_clients[arm].call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()
        if res is None or len(res.results) != 6 or not all(r.successful for r in res.results):
            raise RuntimeError(
                f'[{arm}] set stiffness failed ({context}): '
                f'{[r.reason for r in res.results] if res is not None else "timed out"}')
        self._cur_stiff[arm] = (tuple(float(v) for v in trans), tuple(float(v) for v in rot))
        self.get_logger().info(
            f'[{arm}] stiffness -> trans={tuple(trans)} N/m rot={tuple(rot)} Nm/rad ({context})')

    def _ensure_stiffness(self, arm, stiff, context):
        """Set + dwell only if it actually differs from what's currently commanded."""
        key = (tuple(float(v) for v in stiff[0]), tuple(float(v) for v in stiff[1]))
        if self._cur_stiff.get(arm) == key:
            return
        self._apply_stiffness(arm, stiff, context)
        self.get_logger().info(f'[{arm}] dwelling {self._dwell_s:.2f}s for gain blend...')
        self._spin_sleep(self._dwell_s)

    def _set_nullspace(self, arm, q, ns_stiffness, context):
        """Push nullspace_desired_configuration (7 values) + nullspace_stiffness together.
        Only meaningful while the arm is idle-holding -- during a FollowJointTrajectory the
        controller uses trajectory_nullspace_stiffness + the planned config instead."""
        q = [float(v) for v in q]
        if len(q) != 7:
            self.get_logger().warn(f'[{arm}] nullspace config has {len(q)} values, not 7 -- skipped')
            return
        req = SetParameters.Request()
        req.parameters = [
            Parameter('nullspace_desired_configuration',
                      Parameter.Type.DOUBLE_ARRAY, q).to_parameter_msg(),
            Parameter('nullspace_stiffness',
                      Parameter.Type.DOUBLE, float(ns_stiffness)).to_parameter_msg(),
        ]
        future = self._param_clients[arm].call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()
        if res is None or not all(r.successful for r in res.results):
            raise RuntimeError(
                f'[{arm}] set nullspace failed ({context}): '
                f'{[r.reason for r in res.results] if res is not None else "timed out"}')
        self.get_logger().info(
            f'[{arm}] nullspace posture set (stiffness={ns_stiffness}, {context})')

    def _recommand_holder(self, context):
        """Send the hold arm a short 1-point goal at its NOMINAL joint config (not its
        measured, possibly-sagged pose) to re-energise the impedance spring toward the
        intended pose and pull back stiction slop. Best-effort: tolerates a goal-tolerance
        miss (small corrective move, must not abort the plan)."""
        if self._holder_nominal_q is None:
            return
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in self._holder_nominal_q]
        p.time_from_start.sec = int(HOLDER_RECOMMAND_S)
        p.time_from_start.nanosec = int((HOLDER_RECOMMAND_S % 1.0) * 1e9)
        self.get_logger().info(f'[{HOLD_ARM}] re-commanding nominal hold config ({context})')
        self._run_goal(HOLD_ARM, [p], tolerate_goal_tol=True)

    def _update_holder_nominal(self, q, context):
        """Record the hold arm's new intended holding config and refresh its nullspace
        posture to match. Called after every hold-role arm sub-goal."""
        self._holder_nominal_q = [float(v) for v in q]
        self._set_nullspace(HOLD_ARM, self._holder_nominal_q,
                            HOLDER_NULLSPACE_STIFFNESS, context)

    def _spin_sleep(self, seconds):
        """Sleep while still spinning, so topics/futures keep flowing (gain blend, gripper
        service results, data_impedance, joint_states)."""
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, end - time.time())))

    def _joint_state_cb(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in ARM_JOINTS['lbr_one'] or name in ARM_JOINTS['lbr_two']:
                self._current_positions[name] = position

    def _init_waypoints(self, arm, target_q):
        """Linear constant-velocity joint-space blend from the arm's MEASURED current pose
        to target_q (Fabrica's rest_q). Waypoint count scales with the largest per-joint
        angular distance at INIT_MOVE_SPEED_RAD_S, so a further start just gets more
        waypoints at the same speed. Returns (positions list, times list)."""
        for _ in range(5):  # refresh so current_position is measured, not stale
            rclpy.spin_once(self, timeout_sec=0.02)
        start = np.array([self._current_positions[j] for j in ARM_JOINTS[arm]])
        target = np.asarray(target_q, dtype=float)
        max_delta = float(np.max(np.abs(target - start)))
        duration = max(max_delta / INIT_MOVE_SPEED_RAD_S, INIT_MIN_WAYPOINTS / FPS)
        n = max(INIT_MIN_WAYPOINTS, int(np.ceil(duration * FPS)))
        fracs = np.linspace(1.0 / n, 1.0, n)  # skip the start sample, end exactly at target
        positions = [(start + f * (target - start)).tolist() for f in fracs]
        times = [(i + 1) * duration / n for i in range(n)]
        self.get_logger().info(
            f'[{arm}] init blend: max |delta|={np.degrees(max_delta):.1f} deg -> {n} wp over '
            f'{duration:.1f}s at ~{INIT_MOVE_SPEED_RAD_S} rad/s')
        return positions, times

    # ------------------------------------------------------------------- error reporting ---
    def _cartesian_error_str(self, arm):
        """6D task-space error (impedance target - EE, base frame) from the arm's last
        cartesian_impedance_<arm>/data_impedance sample, or None."""
        d = self._cart_err.get(arm)
        if not d or len(d) < 6:
            return None
        px, py, pz, rx, ry, rz = d[:6]
        pos_mm = np.array([px, py, pz]) * 1000.0
        rot_deg = np.degrees([rx, ry, rz])
        return (
            'cartesian error (impedance target - EE, base frame): '
            f'|trans|={np.linalg.norm(pos_mm):.3f} mm [dx={pos_mm[0]:+.3f}, dy={pos_mm[1]:+.3f}, '
            f'dz={pos_mm[2]:+.3f}]; |rot|={np.linalg.norm(rot_deg):.3f} deg '
            f'[rx={rot_deg[0]:+.3f}, ry={rot_deg[1]:+.3f}, rz={rot_deg[2]:+.3f}]; '
            f'6D (m,rad)=[{px:+.5f}, {py:+.5f}, {pz:+.5f}, {rx:+.5f}, {ry:+.5f}, {rz:+.5f}]')

    def _residual_report(self, arm, joint_names, points, result):
        """Per-joint (commanded final point - measured) + setpoint tracking error + 6D
        Cartesian error + controller error_string. `result` may be None (post-success
        residual log) -- then the controller string is skipped."""
        parts = []
        fb = self._last_feedback
        target = dict(zip(joint_names, points[-1].positions)) if points else {}
        if fb is not None and len(fb.actual.positions):
            fb_names = list(fb.joint_names) or list(joint_names)
            actual = dict(zip(fb_names, fb.actual.positions))
            miss = {n: target[n] - actual[n] for n in target if n in actual}
            if miss:
                pairs = sorted(miss.items(), key=lambda p: abs(p[1]), reverse=True)
                max_abs = max(abs(v) for v in miss.values())
                per_joint = ', '.join(
                    f'{n}={v:+.5f} rad ({np.degrees(v):+.3f} deg)' for n, v in pairs)
                parts.append(
                    f'joint miss (commanded final - measured): max |miss|={max_abs:.5f} rad '
                    f'({np.degrees(max_abs):.3f} deg); per joint [{per_joint}]')
            track = list(fb.error.positions)
            if track and any(abs(e) > 1e-4 for e in track):
                te = max(abs(e) for e in track)
                parts.append(
                    f'setpoint tracking error: max |err|={te:.5f} rad ({np.degrees(te):.3f} deg)')
        else:
            parts.append('no trajectory feedback received -- per-joint miss unavailable')
        cart = self._cartesian_error_str(arm)
        if cart:
            parts.append(cart)
        err_str = '' if result is None else (result.result.error_string or '').strip()
        if err_str:
            parts.append(f'controller: {err_str}')
        return '; '.join(parts)

    # ------------------------------------------------------------------------ goal exec ---
    def _run_goal(self, arm, points, tolerate_goal_tol):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS[arm]
        goal.trajectory.points = points
        goal.goal_time_tolerance.sec = 5

        self._last_feedback = None
        send_future = self._arm_clients[arm].send_goal_async(
            goal, feedback_callback=lambda m: setattr(self, '_last_feedback', m.feedback))
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f'[{arm}] goal rejected by cartesian_impedance_{arm}')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is not None and \
                result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return
        code = None if result is None else result.result.error_code
        report = self._residual_report(arm, ARM_JOINTS[arm], points, result)
        if tolerate_goal_tol and code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            self.get_logger().warn(
                f'[{arm}] goal tolerance not met under compliance (expected for this tag) '
                f'-- continuing. {report}')
            return
        raise RuntimeError(
            f'[{arm}] trajectory execution failed (error_code={code}). {report}')

    def _send_arm_subgoal(self, role, sub_path, tag, description, active_part):
        arm = ROLE_TO_ARM[role]
        brace = (self._holder_brace and tag == 'insertion' and role == 'move')
        if brace:
            self._ensure_stiffness(HOLD_ARM, HOLDER_BRACE_STIFF,
                                   f'brace for {arm} insertion (part={active_part})')
            # Re-seat the hold arm on its intended pose (correct stiction slop / any
            # elastic sag recovered by the higher brace gain) before the insertion loads it.
            self._recommand_holder(f'pre-{arm}-insertion (part={active_part})')

        self._ensure_stiffness(arm, STIFFNESS_TABLE[tag],
                               f'{tag} <- {description} (part={active_part})')

        if tag == 'init':
            # Constant-velocity blend from the measured current pose to rest_q, rather than
            # a single far waypoint at a fixed time (which would imply an arbitrary speed).
            wp_positions, wp_times = self._init_waypoints(arm, np.asarray(sub_path)[-1])
        else:
            wp_positions = [[float(x) for x in q] for q in sub_path]
            wp_times = [(i + 1) / FPS for i in range(len(sub_path))]

        points = []
        for q, t in zip(wp_positions, wp_times):
            p = JointTrajectoryPoint()
            p.positions = [float(x) for x in q]
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t % 1.0) * 1e9)
            points.append(p)

        self.get_logger().info(
            f'[{arm}] {tag} <- {description} ({len(points)} wp, part={active_part})')
        self._run_goal(arm, points, tolerate_goal_tol=(tag in TOLERATE_TAGS))

        if tag in TOLERATE_TAGS:
            self.get_logger().info(
                f'[{arm}] post-{tag} residual -- {self._residual_report(arm, ARM_JOINTS[arm], points, None)}')

        if brace:
            self._ensure_stiffness(HOLD_ARM, HOLDER_STIFF,
                                   f'release brace after {arm} insertion')

        if role == 'hold':
            # The hold arm just finished a sub-goal -- its intended holding config is this
            # segment's last waypoint. Record it and refresh the null-space posture.
            self._update_holder_nominal(
                wp_positions[-1], f'after {tag} <- {description}')
            # Re-assert firm gains after any manoeuvre that leaves it holding station
            # (its stiffness was just set to this segment's tag value).
            if tag in ('transport_carry', 'lift', 'retract', 'free'):
                self._ensure_stiffness(HOLD_ARM, HOLDER_STIFF,
                                       'hold arm back to firm hold after manoeuvre')

    # --------------------------------------------------------------- schedule building ---
    def _gripper_close_is_grasp(self, idx, role):
        """description=='close' at motion[idx] is a real (torque) grasp iff a part is
        carried right after it, or the same role reopens later (regrasp). Same logic as
        hardware_plan_executor_node.py."""
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

    def _approach_precedes_grasp(self, idx, role):
        """motion[idx] is a switch/transport arm entry: is the next same-role gripper
        event (before this arm moves again) a real-grasp 'close'?"""
        for j in range(idx + 1, len(self.motion)):
            mt, bt, _v, _p, desc = self.motion[j]
            if bt == 'arm' and mt == role:
                return False
            if bt == 'gripper' and mt == role and desc == 'close':
                return self._gripper_close_is_grasp(j, role)
        return False

    def _grasp_precedes_carry(self, idx, role):
        """motion[idx] is a part-carrying transport: was the closest preceding same-role
        gripper entry a real-grasp 'close'?"""
        for j in range(idx - 1, -1, -1):
            mt, bt, _v, _p, desc = self.motion[j]
            if bt == 'arm' and mt == role:
                return False
            if bt == 'gripper' and mt == role:
                return desc == 'close' and self._gripper_close_is_grasp(j, role)
        return False

    def _follows_release(self, idx, role):
        """Closest preceding same-role gripper entry is an 'open' (this arm just let go)."""
        for j in range(idx - 1, -1, -1):
            mt, bt, _v, _p, desc = self.motion[j]
            if bt == 'arm' and mt == role:
                return False
            if bt == 'gripper' and mt == role:
                return desc == 'open'
        return False

    def _tag_arm_entry(self, idx, role, path, description, active_part):
        """-> list of (sub_path ndarray, tag). Splits pre-grasp approaches and post-grasp
        lifts off the existing resampled path (no IK)."""
        if description == 'init':
            return [(path, 'init')]
        if description == 'assembly':
            return [(path, 'insertion')]
        # switch | transport
        if self._approach_precedes_grasp(idx, role):
            n = min(N_PREGRASP_WP, max(1, len(path) // 2))
            if n >= len(path):
                self.get_logger().warn(
                    f'{role} {description} entry {idx}: only {len(path)} wp, shorter than the '
                    f'{N_PREGRASP_WP}-wp pre-grasp split -- whole segment tagged grasp_approach')
                return [(path, 'grasp_approach')]
            return [(path[:-n], 'pregrasp_approach'), (path[-n:], 'grasp_approach')]
        if active_part is not None and self._grasp_precedes_carry(idx, role):
            n = min(N_LIFT_WP, max(1, len(path) // 2))
            if n >= len(path):
                return [(path, 'lift')]
            return [(path[:n], 'lift'), (path[n:], 'transport_carry')]
        if active_part is not None:
            return [(path, 'transport_carry')]
        if self._follows_release(idx, role):
            return [(path, 'retract')]
        return [(path, 'free')]

    def _build_schedule(self):
        """Flatten motion.pkl into ('arm', role, sub_path, tag, desc, part) and
        ('gripper', role, open_ratio, desc, part, is_grasp) steps."""
        steps = []
        for idx, (mt, bt, value, active_part, description) in enumerate(self.motion):
            if bt == 'arm':
                path = np.asarray(value, dtype=float)
                if path.ndim == 1:
                    path = path[np.newaxis, :]
                for sub_path, tag in self._tag_arm_entry(idx, mt, path, description, active_part):
                    steps.append(('arm', mt, sub_path, tag, description, active_part))
            elif bt == 'gripper':
                is_grasp = (description == 'close'
                            and self._gripper_close_is_grasp(idx, mt))
                steps.append(('gripper', mt, float(value), description, active_part, is_grasp))
            else:
                self.get_logger().warn(f'Unknown body_type {bt} at entry {idx}, skipping')
        return steps

    # ---------------------------------------------------------------------- gripper ------
    def _setup_gripper(self, robot_name):
        self._gripper_calibrate_clients = {}
        self._gripper_open_clients = {}
        self._gripper_set_position_clients = {}
        self._gripper_hold_close_clients = {}
        self._gripper_stop_clients = {}
        if not self._gripper_enabled:
            self.get_logger().info(
                'Gripper actuation disabled (pass --gripper to enable) -- arm trajectory '
                'only, plan gripper entries will be skipped.')
            return
        if HoldClose is None or SetPosition is None:
            raise RuntimeError(
                'servo_gripper_julien.srv (HoldClose / SetPosition) could not be imported -- '
                'source the gripper workspace (setup_gripper_env.sh) before --gripper.')
        for ns in GRIPPER_NAMESPACES:
            base = f'/{ns}/{GRIPPER_CONTROLLER_NODE}'
            self._gripper_calibrate_clients[ns] = self.create_client(Trigger, f'{base}/calibrate')
            self._gripper_open_clients[ns] = self.create_client(Trigger, f'{base}/open')
            self._gripper_set_position_clients[ns] = self.create_client(
                SetPosition, f'{base}/set_position')
            self._gripper_hold_close_clients[ns] = self.create_client(
                HoldClose, f'{base}/hold_close')
            self._gripper_stop_clients[ns] = self.create_client(Trigger, f'{base}/stop')
        self.get_logger().info(
            f'Waiting for {GRIPPER_CONTROLLER_NODE} calibrate/open/set_position/hold_close/stop '
            f'on /{GRIPPER_NAMESPACES[0]} and /{GRIPPER_NAMESPACES[1]}...')
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
                        f'{base}/{service} not available -- is {GRIPPER_CONTROLLER_NODE} '
                        f'running for both sides? Adjust GRIPPER_CONTROLLER_NODE / '
                        f'GRIPPER_NAMESPACES / ROLE_TO_GRIPPER_NS if the live namespaces differ.')
        # One home cycle per gripper: ~/calibrate sweeps the stroke (this both brings the
        # jaws to a known state and gives ~/set_position its open/closed encoder limits --
        # the plan's first gripper entry is a set_position), then ~/open sets the plan's own
        # starting state. No separate hold_close+settle here -- the calibrate sweep already
        # drives the jaws closed, so adding hold_close homed/closed each gripper twice.
        self.get_logger().info('Homing both grippers (calibrate, then open)...')
        self._call_gripper_all(self._gripper_calibrate_clients, Trigger.Request(), 'calibrate')
        self._call_gripper_all(self._gripper_open_clients, Trigger.Request(), 'open')

    def _call_gripper_all(self, clients_by_ns, request, label):
        futures = {ns: c.call_async(request) for ns, c in clients_by_ns.items()}
        deadline = time.time() + GRIPPER_MOTION_TIMEOUT_S
        while rclpy.ok() and not all(f.done() for f in futures.values()):
            if time.time() >= deadline:
                break
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, deadline - time.time())))
        failures = []
        for ns, future in futures.items():
            result = future.result() if future.done() else None
            if result is None or not result.success:
                failures.append(f'[{ns}] '
                                f'{result.message if result is not None else "timed out"}')
            else:
                self.get_logger().info(f'[{ns}] gripper {label} (home): {result.message}')
        if failures:
            raise RuntimeError(f'gripper {label} (home) failed -- ' + '; '.join(failures))

    def _call_gripper_trigger(self, ns, client, service_name, context=''):
        tag = f'{service_name} ({context})' if context else service_name
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=GRIPPER_MOTION_TIMEOUT_S)
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError(
                f'[{ns}] gripper {tag} failed: '
                f'{result.message if result is not None else "timed out"}')
        self.get_logger().info(f'[{ns}] gripper {tag}: {result.message}')

    def _call_set_position(self, ns, open_ratio, context):
        position = float(np.clip(1.0 - open_ratio, 0.0, 1.0))
        req = SetPosition.Request()
        req.position = position
        future = self._gripper_set_position_clients[ns].call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=GRIPPER_MOTION_TIMEOUT_S)
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError(
                f'[{ns}] gripper set_position({position:.3f}) ({context}) failed: '
                f'{result.message if result is not None else "timed out"}')
        self.get_logger().info(
            f'[{ns}] gripper set_position({position:.3f}, open_ratio={open_ratio:.3f}, '
            f'{context}): {result.message}')

    def _call_hold_close(self, ns, context):
        req = HoldClose.Request()
        req.torque_limit = self._gripper_torque
        future = self._gripper_hold_close_clients[ns].call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=GRIPPER_MOTION_TIMEOUT_S)
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError(
                f'[{ns}] gripper hold_close ({context}) failed: '
                f'{result.message if result is not None else "timed out"}')
        self.get_logger().info(
            f'[{ns}] gripper hold_close ({context}, torque_limit={self._gripper_torque}): '
            f'{result.message}')

    def _release_grippers(self, reason):
        if not self._gripper_enabled:
            return
        self.get_logger().warn(f'{reason} -- ~/stop on both grippers to release torque')
        for ns in GRIPPER_NAMESPACES:
            try:
                self._call_gripper_trigger(ns, self._gripper_stop_clients[ns], 'stop', reason)
            except Exception as exc:  # noqa: BLE001 -- best-effort cleanup
                self.get_logger().error(f'[{ns}] gripper stop during abort failed: {exc}')

    def _send_gripper_command(self, role, open_ratio, description, active_part, is_grasp):
        ns = ROLE_TO_GRIPPER_NS[role]
        if not self._gripper_enabled:
            self.get_logger().info(
                f'[{ns}] gripper {description} (part={active_part}) skipped -- no --gripper')
            return
        if description == 'close' and is_grasp:
            self.get_logger().info(
                f'[{ns}] gripper close (part={active_part}, open_ratio={open_ratio:.3f}) -> '
                f'~/hold_close torque_limit={self._gripper_torque} [grasp]')
            self._call_hold_close(ns, description)
            self._spin_sleep(GRIPPER_CLOSE_SETTLE_S)
        elif description == 'open' and open_ratio >= GRIPPER_FULL_OPEN_RATIO:
            self._call_gripper_trigger(
                ns, self._gripper_open_clients[ns], 'open', description)
        else:
            self.get_logger().info(
                f'[{ns}] gripper {description} (part={active_part}, open_ratio={open_ratio:.3f}) '
                f'-> ~/set_position [no torque]')
            self._call_set_position(ns, open_ratio, description)

    # --------------------------------------------------------------------------- run ----
    def run(self):
        for step in self._schedule:
            if step[0] == 'arm':
                _, role, sub_path, tag, desc, part = step
                self._send_arm_subgoal(role, sub_path, tag, desc, part)
            else:
                _, role, open_ratio, desc, part, is_grasp = step
                if desc == 'init' and self._gripper_enabled:
                    self.get_logger().info(
                        f'[{ROLE_TO_GRIPPER_NS[role]}] gripper init entry skipped -- startup '
                        'homing already established a known state')
                    continue
                self._send_gripper_command(role, open_ratio, desc, part, is_grasp)
                # Once the hold arm has clamped a part, make it firm again -- it was left
                # at grasp_approach softness by its approach and (unlike the move arm) has
                # no lift segment after to ramp it back up before it must hold station.
                if desc == 'close' and is_grasp and role == 'hold':
                    self._ensure_stiffness(HOLD_ARM, HOLDER_STIFF,
                                           'hold arm firm after grasping')
        self.get_logger().info('Plan complete.')


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('log_dir', nargs='?',
                        default=os.path.expanduser('~/Fabrica/logs/plumbers_block_sim'),
                        help='directory containing motion.pkl (default: %(default)s)')
    parser.add_argument('robot_name', nargs='?', default='lbr_dual_arm',
                        help='controller_manager namespace (default: %(default)s)')
    parser.add_argument('--gripper', action='store_true',
                        help='home and actuate the servo_gripper_julien grippers (see module '
                             'docstring); without it, arm trajectory only.')
    parser.add_argument('--gripper-torque', type=int, default=GRIPPER_HOLD_TORQUE_DEFAULT,
                        metavar=f'0..{GRIPPER_HOLD_TORQUE_MAX}',
                        help='hold_close torque_limit (default %(default)s).')
    parser.add_argument('--dwell', type=float, default=DWELL_S,
                        help='hold-still seconds after each stiffness change (default %(default)s)')
    parser.add_argument('--no-holder-brace', action='store_true',
                        help='do not raise the hold arm to HOLDER_BRACE_STIFF during the move '
                             "arm's insertions")
    cli_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:] if args is None else args
    parsed = parser.parse_args(cli_args)

    node = VariableImpedancePlanExecutor(
        parsed.log_dir, robot_name=parsed.robot_name, gripper=parsed.gripper,
        gripper_torque=parsed.gripper_torque, dwell_s=parsed.dwell,
        holder_brace=not parsed.no_holder_brace)
    try:
        node.run()
    except BaseException as exc:  # incl. KeyboardInterrupt -- don't leave servos gripping
        node._release_grippers(f'run aborted ({type(exc).__name__})')
        raise
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
