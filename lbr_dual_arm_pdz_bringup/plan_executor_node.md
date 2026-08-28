# plan_executor_node.py

> **Just want the commands?** See **[`QUICKSTART.md`](QUICKSTART.md)** — copy-paste bringup +
> executor invocations for every rig. This doc is the full reference.

Replays a Fabrica `motion.pkl` plan on the dual-arm KUKA via the standard
`FollowJointTrajectory` action — **one node for Gazebo, mock, and real FRI hardware**. What
differs between the rigs is expressed entirely through CLI args that the bringup launch files
set; there is no per-rig code path.

`hardware_plan_executor_node.py` is now a **thin shim** that just prepends the real/mock FRI
defaults (`--controller-topology combined --robot-name lbr_dual_arm`) and forwards everything
else. `variable_impedance_plan_executor_node.py` is unchanged — it is the advanced
variable-per-phase-gain impedance variant and is already rig-agnostic.

---

## Why one node can serve every rig (and why it couldn't before)

Before this merge there were two divergent nodes, and you could **not** just launch Gazebo and
run the "hardware" one (or vice versa):

1. **Different action servers.** The hardware node opened one client at
   `/lbr_dual_arm/joint_trajectory_controller/follow_joint_trajectory` (14-joint goals).
   Gazebo (`dual_arm_pdz_gazebo_controllers.yaml`) spawns
   `joint_trajectory_controller_lbr_one` / `_two` under `/lbr_dual_arm_pdz` — the combined
   action doesn't exist there, so `wait_for_server()` blocks forever. The reverse fails the
   same way.
2. **Namespace.** Gazebo deliberately uses `robot_name:=lbr_dual_arm_pdz` (a stale RSP on this
   workstation squats `/lbr_dual_arm/robot_description`, which `ros_gz_sim create -topic`
   latched — see `launch/gazebo.launch.py`). Hardware/mock use `lbr_dual_arm`.
3. **Gripper is a different mechanism.** Gazebo: a simulated pdz finger joint over
   `FollowJointTrajectory`. Hardware: real servo hardware via `servo_gripper_julien` services
   (`calibrate`/`open`/`set_position`/`hold_close`/`stop`, custom srv, startup homing, hold
   torque, `~/stop` on abort). No mock/sim variant exists.
4. **Held-part visualization is Gazebo-only** — `ros_gz_interfaces/SetEntityPose` on
   `/world/<world>/set_pose`, plus `traj.npy`, plus simulated part models.
5. **Timing / safety semantics.** Gazebo tolerated fixed `i/30 s` timing, positions only,
   log-and-continue on failure. Real hardware needs explicit per-waypoint velocities +
   `_timed_waypoints()` velocity blending (the FRI `CommandGuard` **crashes the hardware
   interface** otherwise — see "Waypoint timing and blending"), always-14-joint goals, and
   raise-on-failure.
6. **Clock.** Gazebo needs `use_sim_time=True` + a `/clock` bridge; hardware is wall-clock.

The merged node keeps the **hardware-safe core** (a strict superset — the velocity blend is a
no-op in Gazebo except at genuine plan discontinuities) and selects (2)–(6) from args.

---

## Arg matrix

| arg | default | meaning |
|---|---|---|
| `log_dir` (positional) | `~/Fabrica/logs/plumbers_block_sim` | dir with `motion.pkl` (+ `traj.npy` when visualizing) |
| `--controller-topology` | `per-arm` | `per-arm`: one action client per arm at `joint_trajectory_controller_lbr_{one,two}` (each also carries its arm's pdz gripper joint). `combined`: one 14-joint `joint_trajectory_controller`; every goal names all 14 joints, idle arm pinned to measured `/joint_states`. |
| `--robot-name` | `lbr_dual_arm_pdz` | `controller_manager` / `joint_states` namespace. Shim → `lbr_dual_arm`. |
| `--arm-control` | `joint_trajectory` | `cartesian_impedance` → replay against per-arm `cartesian_impedance_lbr_{one,two}` (effort). Always per-arm — `--controller-topology` has no effect. Gripper then goes to a separate per-arm `gripper_controller_lbr_*`. |
| `--gripper-backend` | `none` | `none`: arm-only, gripper entries logged + skipped. `sim`: simulated pdz finger joint (single-joint `FollowJointTrajectory`, `open_ratio*0.032 m`). `servo`: `servo_gripper_julien` services — **real hardware, no sim variant**; needs `setup_gripper_env.sh` sourced. |
| `--gripper` | off | back-compat alias for `--gripper-backend servo` (only when `--gripper-backend` is left at `none`). |
| `--gripper-torque` | `250` | `hold_close` `torque_limit` for grasps (servo backend only), 0..500. |
| `--visualize-held-parts` | off | Gazebo only: teleport the held part to match `traj.npy` via `/world/<world>/set_pose` at 30 Hz alongside each arm goal. |
| `--world` | `plumbers_block` | gz-sim world name for the teleport service. |
| `--use-sim-time` | off | set the node's `use_sim_time` param (Gazebo). |
| `--on-failure` | `raise` | `raise`: stop the plan on any rejected/failed goal. `continue`: log and go on. `GOAL_TOLERANCE_VIOLATED` under `cartesian_impedance` is **always** tolerated. |

### Per-rig invocations

```bash
# real FRI hardware (combined JTC), arm + real servo grippers
ros2 launch lbr_dual_arm_bringup hardware.launch.py
source setup_gripper_env.sh
ros2 run lbr_dual_arm_pdz_bringup hardware_plan_executor_node --gripper

# mock hardware, arm only
ros2 launch lbr_dual_arm_bringup mock.launch.py
ros2 run lbr_dual_arm_pdz_bringup hardware_plan_executor_node

# Gazebo, position JTC, sim gripper + held-part viz
ros2 launch lbr_dual_arm_pdz_bringup gazebo.launch.py
ros2 run lbr_dual_arm_pdz_bringup plan_executor_node \
  --robot-name lbr_dual_arm_pdz --gripper-backend sim \
  --visualize-held-parts --use-sim-time --on-failure continue
#   ... or let the launch start it:  gazebo.launch.py run_executor:=true

# Gazebo, Cartesian impedance
ros2 launch lbr_dual_arm_pdz_bringup gazebo.launch.py arm_control:=cartesian_impedance
ros2 run lbr_dual_arm_pdz_bringup plan_executor_node \
  --robot-name lbr_dual_arm_pdz --arm-control cartesian_impedance \
  --gripper-backend sim --visualize-held-parts --use-sim-time
```

Only the **bringup launch + args** change between rigs; the executor code path is identical.

---

## Waypoint timing and blending (all rigs)

`_timed_waypoints()` replaces any fixed `1/FPS` spacing with per-segment blending: for every
step — **including the synthetic "measured current pose → `path[0]`" step at the start of each
goal** — it computes the time that step needs so no joint's implied velocity exceeds
`JOINT_MAX_VELOCITY_RAD_S * BLEND_VELOCITY_SAFETY_FACTOR` (30 % of the real FRI ceiling), and
uses `max(nominal_dt, required_dt)` — it only ever **stretches**, never shortens the plan's
30 fps cadence. Per-waypoint velocities are then finite-differenced over the resulting
(possibly non-uniform) grid.

### Why this is mandatory on hardware

The old Gazebo node gave each goal's first point only `1/FPS` (33 ms), assuming the arm was
already at `path[0]`. Nothing guarantees that — the very first goal starts from wherever
`/joint_states` is, and consecutive same-arm entries in `plumbers_block_sim/motion.pkl` are not
always continuous (entries 21 / 28 / 38 land 4.9° / 1.3° / 6.0° off). Dividing a 6° gap by
33 ms implies ~3.2 rad/s, past joint A3's real limit.
`lbr_fri_ros2::CommandGuard::command_in_velocity_limits_` (`command_guard.cpp:45`) compares
consecutive **measured** joint positions against each joint's `max_velocity` and throws
`std::runtime_error("Invalid command.")` on the FRI realtime thread
(`position_command.cpp:58-63`) — bringing down the whole hardware interface.

### Why it's harmless in Gazebo

Ordinary within-path motion peaks at ~0.30 rad/s across the whole plan, well under every
limit, so the stretch only fires at the three real discontinuities (verified: they land at
exactly 0.524 rad/s = 30 % of A3's 1.745 rad/s hard limit after blending). Setting explicit
velocities also removes the velocity discontinuity every 1/30 s that positions-only leaves.
`BLEND_LOG_THRESHOLD` (1.5×) means any stretched-beyond-noise segment is warned with the
joint, degree gap, and new duration.

### Follow-up: holder-arm snap at goal starts

`_send_arm_path()` deliberately does **not** cache `path[-1]` after a goal — it spins
`/joint_states` and re-reads the measured pose before the next goal, so the few-tenths-of-a-
degree settle error is a normal blended segment, not a fast snap the controller covers in one
tick. (This was most visible in the holder arm's first ~10–20 s.)

### What this does NOT fix

Velocity only. No acceleration/jerk limits, no optimal-time re-derivation, no Cartesian-space
safety (collisions, workspace) — the plan's / operator's responsibility, unchanged.

---

## `--arm-control cartesian_impedance`: goal-tolerance violation reporting

The `cartesian_impedance_controller` aborts a goal with `GOAL_TOLERANCE_VIOLATED` routinely —
a compliant arm rarely settles *every* joint inside `trajectory_default_goal_tolerance` within
the goal-time window. `_run_goal()` tolerates that (warn + continue) and calls
`_goal_tol_violation_report()`, which quantifies the miss from up to three parts:

**a) Joint-space goal miss** — `commanded final point − measured`, per joint, worst-first, in
rad and deg. Source: the goal's last `JointTrajectoryPoint.positions` vs. the last
`FollowJointTrajectory` feedback's `actual.positions` (name-keyed, so a joint-ordering
mismatch can't misalign them; `feedback_callback` caches `self._last_feedback`).
*Not* the feedback `error` field — that is the running setpoint-tracking error, ~1e-5 rad
under impedance control, and does not explain the abort. It is printed only as a secondary
line and only when some component exceeds 1e-4 rad.

**b) 6D Cartesian error** — no FK in this node (`kdl_parser_py` isn't installed). Instead it
subscribes to `/<robot_name>/cartesian_impedance_<arm>/data_impedance`
(`std_msgs/Float64MultiArray`, len 14), whose `data[0:6]` is `motion_error = [target − EE]` in
the base frame: `[0:3]` position xyz (m), `[3:6]` rotation as a Rodrigues vector (rad). The
controller drives its target from the sampled joint trajectory via FK, so at a segment's end
`data[0:6]` **is** the Cartesian goal error. Reported as `|trans|` mm + `dx/dy/dz`, `|rot|`
deg + `rx/ry/rz`, and the raw 6-vector with `||6D||` (mixed m + rad — use the split values for
anything quantitative).

**c) Controller `error_string`** — appended verbatim when the controller sets one (the gz
controller currently does not).

Any non-`GOAL_TOLERANCE_VIOLATED` failure, or `GOAL_TOLERANCE_VIOLATED` outside impedance
mode, gets the same report but obeys `--on-failure` (default `raise`).

---

## `--gripper-backend servo`: `servo_gripper_julien` service interface

Drives the **`servo_gripper_julien`** package (node `gripper_controller`, workspace on the
gripper machine). Namespaced by **side** — `/left/gripper_controller/…` and
`/right/gripper_controller/…`; this node maps `hold`→`left`, `move`→`right`
(`ROLE_TO_GRIPPER_NS`). If the physical grippers are swapped, swap that mapping (or the
launch), not the node.

### Setup

```bash
source ./setup_gripper_env.sh
ros2 service list -t | grep gripper_controller   # confirm the namespaces
```

The node fails fast at startup (`RuntimeError`) if any of
`/{left,right}/gripper_controller/{calibrate,open,set_position,hold_close,stop}` is missing, or
if the `servo_gripper_julien` interfaces aren't importable.

### Services used

| Service | Type | Blocking? | Effect |
|---|---|---|---|
| `~/calibrate` | `std_srvs/srv/Trigger` | blocks (sweeps the stroke) | learn the open/closed encoder limits; `~/set_position` needs it first |
| `~/open` | `std_srvs/srv/Trigger` | blocks until the jaws reach the open endpoint | full open |
| `~/set_position` | `servo_gripper_julien/srv/SetPosition` (`position` float, 0.0=open..1.0=closed) | blocks | drive to a normalized target and stop, no holding torque |
| `~/hold_close` | `servo_gripper_julien/srv/HoldClose` (`torque_limit` uint16 0..500) | **returns immediately**; torque persists until `~/open`/`~/set_position`/`~/stop` | start closing, then hold at `torque_limit` |
| `~/stop` | `std_srvs/srv/Trigger` | immediate | drop holding torque without opening |

`open_ratio` (0=closed..1=open) → `~/set_position` wants `1 - open_ratio` (0.0=open..1.0=closed),
clamped. Routing is on the entry's `description`, not a ratio threshold:

- **`close` + real grasp** → `~/hold_close` at `--gripper-torque` (default 250), then
  `_spin_sleep(GRIPPER_CLOSE_SETTLE_S)` (10 s) before the plan's next move. "Real grasp" per
  `_gripper_close_is_grasp()`: a part is carried right after this close, **or** this grip is
  reopened later for the same role (covers a regrasp).
- **`close`, not a grasp** → `~/set_position(1 - open_ratio)` (the plan's final park close).
- **`open`, `open_ratio ≥ 0.98`** → `~/open` (full-open Trigger). Startup homing hits this;
  nothing in `plumbers_block` does.
- **`open`, below that** → `~/set_position(1 - open_ratio)` (pre-grasp widen, release).

**Startup homing** (before the plan): `~/calibrate` then `~/open`, each stage on both sides at
once. Because this establishes a known state, `run()` **skips** the plan's own
`description=='init'` gripper entries. **On abort** (any exception, incl. Ctrl-C): `main()`
calls `~/stop` on both grippers.

**Safety:** the servo gripper is real hardware always — there is no mock/sim variant, so
gripper motion happens for real even during an arm-mock dry run.

---

## Held-part visualization (`--visualize-held-parts`, Gazebo)

Re-uses `logs/<exp>/traj.npy` — Fabrica's own rendered simulation, which already computed the
correct `part_i` pose for every frame including while carried — and teleports the held part via
gz-sim's `/world/<world>/set_pose` (`ros_gz_interfaces/srv/SetEntityPose`) at the same 30 Hz
cadence as the arm trajectory. `traj.npy` frame 0 is the pre-command initial state; frame
`(i+1)` is the state after `motion.pkl`'s `i`-th flattened waypoint/command.
`ros_gz_interfaces` and `scipy` are imported lazily so a pure-hardware box without them can
still run the shim.
