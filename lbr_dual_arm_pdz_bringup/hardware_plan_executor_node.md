# hardware_plan_executor_node.py

Replays a Fabrica `motion.pkl` plan on the real/mock dual-arm KUKA via the standard
`joint_trajectory_controller`/`FollowJointTrajectory` action. This is the hardware counterpart of
`plan_executor_node.py`, which does the same job against the Gazebo/pdz-gripper simulation. Both
files live in `lbr_dual_arm_pdz_bringup/lbr_dual_arm_pdz_bringup/`.

## Why a separate script from the Gazebo version

The two rigs differ enough in controller topology and available services that a single
parameterized node wasn't worth the branching. Concretely:

| | `plan_executor_node.py` (Gazebo) | `hardware_plan_executor_node.py` (real/mock) |
|---|---|---|
| Controllers | Two `JointTrajectoryController`s, one per arm (`joint_trajectory_controller_lbr_one/_two`), each also claiming its arm's pdz gripper driving joint | One combined 14-joint `joint_trajectory_controller` for both arms (`dual_arm_controllers.yaml`) |
| Partial goals | `allow_partial_joints_goal: true` (Gazebo controllers yaml) -- a goal can name just the joints it moves | `allow_partial_joints_goal: true` in `dual_arm_controllers.yaml` too, but this node still always sends all 14 joint names/positions per goal (moving arm's path + idle arm held at its last known real position) since it only has the one combined action client |
| Gripper | Simulated pdz finger joint, driven through the *same* `FollowJointTrajectory` action as the arm (`GRIPPER_JOINTS`, `gripper_open_ratio_to_position()`) | Real servo hardware via **`servo_gripper_julien`** (node `gripper_controller_julien`, workspace `~/Workspaces/servo_test`), **binary open / close-and-hold only** -- no fine width control, no calibration. Services namespaced by **arm**: `/lbr_one/gripper_controller_julien/…`, `/lbr_two/gripper_controller_julien/…` (see "Gripper service interface" below for the exact commands). `motion.pkl`'s `open_ratio` is thresholded (`> GRIPPER_OPEN_RATIO_THRESHOLD` → `~/open`, else `~/hold_close` at `--gripper-torque` N, default 250). `~/open` blocks until the open endpoint; `~/hold_close` returns immediately, so the node then sleeps `GRIPPER_CLOSE_SETTLE_S` (6.0 s). Opt-in via `--gripper` (needs `setup_gripper_env.sh` sourced); without it, gripper entries are logged and skipped, arm-only replay. On an aborted run, `main()` calls `~/stop` on both grippers. **The gripper servos are real hardware even in an arm-mock dry run** -- there is no mock/sim variant. |
| Held-part visualization | Teleports the currently-held part to match `traj.npy`'s rendered pose via `/world/<world>/set_pose` (`SetEntityPose`), at 30 Hz alongside each goal | None -- no such service, no simulated part, on real/mock hardware |
| Arm start positions | N/A (each arm has its own action client/controller, no shared goal) | Reads **both** arms' actual current positions from `/joint_states` before every new goal (`_joint_state_cb`), rather than assuming either is still exactly where a prior goal commanded it -- real/mock joints settle slightly off a setpoint (see "Follow-up (2026-08-27)" below) |
| Failure handling | Logs and continues on a rejected goal or failed result | Raises immediately, stopping the whole plan -- continuing to send goals to real hardware after an unexplained failure is not an acceptable default |
| Velocities | Not set on trajectory points (positions only) -- controller/PID free to interpolate | Set explicitly per waypoint (see "Waypoint timing and blending" below) -- needed because there's no PID slack to hide a velocity discontinuity, and the crash this doc is about lives exactly here |

## Gripper service interface

`--gripper` drives the **`servo_gripper_julien`** package (node `gripper_controller_julien`,
workspace `~/Workspaces/servo_test` on the gripper machine). It is **binary only** -- open, or
close-and-hold at a torque limit. There is no width control and no calibration. This replaced the
older `servo_gripper` package (`~/open` / `~/close` `Trigger`, namespaced `/left` `/right`); the
node no longer talks to that.

### Setup

Source the gripper workspace in the terminal that runs the executor (and in any terminal you call
the services from by hand):

```bash
source ./setup_gripper_env.sh
ros2 service list -t | grep gripper_controller_julien   # confirm the namespaces
```

The dual-gripper launch namespaces every service by **arm name** -- `/lbr_one/...` and
`/lbr_two/...` (a single-gripper launch omits the prefix: `/gripper_controller_julien/...`). The
node maps `hold`→`lbr_one`, `move`→`lbr_two` (`ROLE_TO_ARM`). If the physical grippers are
swapped between USB adapters, swap `ROLE_TO_ARM` (or the launch), not the node name.

### The three services the node uses

| Service | Type | Blocking? | Effect |
|---|---|---|---|
| `~/open` | `std_srvs/srv/Trigger` | blocks until the jaws reach the open endpoint | full open |
| `~/hold_close` | `servo_gripper_julien/srv/HoldClose` (`torque_limit` uint16 0..500) | **returns immediately** -- torque persists until `~/open`, `~/close` or `~/stop` | start closing, then hold at `torque_limit` |
| `~/stop` | `std_srvs/srv/Trigger` | immediate | drop holding torque **without** opening |

All three return `success` (bool) + `message`. The node raises and stops the whole plan on any
non-success or timeout (`GRIPPER_MOTION_TIMEOUT_S` = 30 s), same as for a failed arm goal. Because
`~/hold_close` returns before the jaws have actually closed, after every close the node sleeps
`GRIPPER_CLOSE_SETTLE_S` (6.0 s) before the plan's next move.

`motion.pkl` `open_ratio` (0 = closed, 1 = open) is thresholded: `> 0.5` → `~/open`, else
`~/hold_close`. The hold torque is `--gripper-torque N` (0..500, default
`GRIPPER_HOLD_TORQUE_DEFAULT` = 250).

**Startup homing** (before the plan): `~/hold_close` on both arms → settle → `~/open` on both
arms. This is a "known physical state" step only (no calibration to run). Because it already does
this, `run()` skips the plan's own `description=='init'` gripper entries (`motion.pkl` entries
2-3) -- otherwise the grippers visibly home twice.

**On abort** (any exception, incl. Ctrl-C): `main()` calls `~/stop` on both grippers so the
servos aren't left gripping. A normal plan completion leaves the grippers as its last entries
commanded (for `plumbers_block`, both `hold_close` on the finished assembly).

### Calling the services by hand

Single-gripper launch:

```bash
# Open
ros2 service call /gripper_controller_julien/open std_srvs/srv/Trigger "{}"

# Close and hold at torque_limit 250
ros2 service call /gripper_controller_julien/hold_close \
  servo_gripper_julien/srv/HoldClose "{torque_limit: 250}"

# Release holding torque without opening
ros2 service call /gripper_controller_julien/stop std_srvs/srv/Trigger "{}"
```

Dual-gripper launch (what the executor targets) -- `lbr_one` = left, `lbr_two` = right:

```bash
# --- open ---
ros2 service call /lbr_one/gripper_controller_julien/open std_srvs/srv/Trigger "{}"
ros2 service call /lbr_two/gripper_controller_julien/open std_srvs/srv/Trigger "{}"

# --- close and hold (torque_limit 0..500) ---
ros2 service call /lbr_one/gripper_controller_julien/hold_close \
  servo_gripper_julien/srv/HoldClose "{torque_limit: 250}"
ros2 service call /lbr_two/gripper_controller_julien/hold_close \
  servo_gripper_julien/srv/HoldClose "{torque_limit: 250}"

# --- stop (release holding torque, no open) ---
ros2 service call /lbr_one/gripper_controller_julien/stop std_srvs/srv/Trigger "{}"
ros2 service call /lbr_two/gripper_controller_julien/stop std_srvs/srv/Trigger "{}"
```

See also `docs/gripper_service_commands.md` (rig owner's copy of the same commands).

## Waypoint timing and blending

### The bug (2026-08-26)

Running the node crashed immediately with "unreasonably high commanded velocities." Root cause,
found by statically reading the code and `motion.pkl` (no nodes launched):

`_send_arm_path()` gave every new goal's **first** trajectory point only `1/FPS` (33 ms) to be
reached:

```python
p.time_from_start.sec = int((i + 1) / FPS)   # i=0 -> 1/30 s
```

That assumes the moving arm is already sitting at `path[0]` when the goal starts. Nothing
guarantees that:

- At the very first goal of the whole plan, "current position" comes from live `/joint_states` at
  node startup -- there's no check that the physical/mock rig happens to be parked at Fabrica's
  assumed starting pose.
- Between consecutive same-arm entries later in the plan, the plan itself isn't always
  continuous. In `~/Fabrica/logs/plumbers_block_sim/motion.pkl`, entry 21 (`lbr_two transport`,
  right after the part-1 assembly + gripper-open at entries 19-20) starts ~4.9 deg away from
  where entry 19 left that arm; entry 38 (same pattern, part 4) is ~6.0 deg off. Dividing either
  gap by the 33 ms budget implies **2.5-3.2 rad/s**, well past joint A3's real hardware ceiling.

That ceiling is real and enforced by the FRI driver itself, not just a planning-time nicety:
`lbr_fri_ros2::CommandGuard::command_in_velocity_limits_`
(`lbr_fri_ros2/src/command_guard.cpp:45`) compares consecutive measured joint positions against
each joint's `max_velocity` (wired in from `lbr_system_interface.xacro`, the LBR med7's true
per-joint limit -- 98-180 deg/s depending on joint). A violation throws
`std::runtime_error("Invalid command.")` right on the FRI realtime thread
(`lbr_fri_ros2/src/interfaces/position_command.cpp:58-63`), which brings down the whole hardware
interface -- i.e. the crash.

This code path was copied from `plan_executor_node.py`, where it's harmless: Gazebo spawns the
robot already at `path[0]` of the very first entry, no `.velocities` are set (the controller/PID
freely interpolates), and there's no hardware safety cutoff to trip.

### The fix

`_timed_waypoints()` replaces the fixed `1/FPS` spacing with per-segment blending: for every
step -- including the synthetic "current position -> `path[0]`" step at the start of each goal --
it computes the time that step would need so that no joint's implied velocity exceeds
`JOINT_MAX_VELOCITY_RAD_S * BLEND_VELOCITY_SAFETY_FACTOR` (70% of the real FRI ceiling), and uses
`max(nominal_dt, required_dt)` -- i.e. it only ever *stretches* a step, never shortens the plan's
own 30 fps cadence. Velocities are then recomputed by finite difference over the resulting
(possibly non-uniform) time grid instead of a fixed `dt`.

Because the stretch only kicks in where a real jump exists, ordinary within-path motion (verified
statically to peak at 0.30 rad/s across the whole `plumbers_block_sim` plan, well under every
joint's limit) is untouched -- goals still execute at their original cadence except at genuine
discontinuities, which are now blended and logged (`BLEND_LOG_THRESHOLD` -- a step needing more
than 1.5x the nominal duration warns with the joint, degree gap, and new duration) instead of
being sent as-is and crashing the hardware interface.

`BLEND_VELOCITY_SAFETY_FACTOR` (0.3) is margin below the hard ceiling, not the ceiling itself --
`command_in_velocity_limits_` compares *measured* (not commanded) position deltas, there's an
exponential smoothing filter on the command before it reaches that check
(`joint_position_filter_` in `position_command.cpp`), and this node's finite-difference
velocities are an approximation of the controller's actual spline, not identical to it. Verified
against `plumbers_block_sim/motion.pkl`: the known discontinuities (entries 21, 28, and 38) land
exactly at the 30%-of-ceiling safe velocity (0.524 rad/s vs. joint A3's 1.745 rad/s hard limit)
after blending, instead of the original 2.54/0.68/3.17 rad/s. Entry 28's ~1.3deg gap (0.68 rad/s
implied) doesn't cross the hard limit itself but now gets stretched too at this tighter margin --
expected, and harmless: it just adds ~10ms to that one goal's start.

### Follow-up (2026-08-27): holder-arm snap at goal starts

First run *with grippers* (`--gripper`) surfaced two more issues, both in `hardware_plan_executor_node.py` (not the plan data):

1. **Grippers home twice.** `__init__` homes both grippers at startup (a close + settle + open cycle). The plan's opening `description=='init'` gripper entries (`motion.pkl` entries 2-3: `open_ratio 0.5` → close, entry 4 → open) then repeat that. `run()` now skips `description=='init'` gripper entries when `--gripper` is set. (Under the later `servo_gripper_julien` rewrite the startup close is `~/hold_close` — see "Gripper service interface" — but the double-home logic is unchanged.)

2. **Discontinuity in the holder arm's first ~10-20 s.** Not a plan discontinuity -- every consecutive `hold`-arm entry in `plumbers_block_sim/motion.pkl` is continuous to <0.01°; the only real jumps are `move`-arm entries 21/28/38, already handled above. The cause: `_send_arm_path()` overwrote `self._current_positions[<moving arm>]` with the commanded `path[-1]` after each goal and read that back on the next goal, instead of `/joint_states`. Real/mock joints settle a few tenths of a degree off the setpoint, so `_timed_waypoints()` saw a ~0 gap to the next `path[0]`, gave it the nominal 33 ms with a non-zero `velocities[0]`, and the controller covered the real settle error in one tick -- a fast snap at the *start* of the goal. The holder arm's first motions (entries 1→5→7, ≈10 s then ≈16 s) are where this was most visible. Fix: drop the `path[-1]` cache, spin `/joint_states` before reading `current_position`, so the settle error is a normal blended segment like any other jump.

### What this does NOT fix

`_timed_waypoints()` only guards against exceeding the *velocity* limit implied by position
deltas. It does not re-derive an optimal-time trajectory, does not consider acceleration/jerk
limits, and does not know anything about Cartesian-space safety (collisions, workspace limits) --
those are unchanged from before this fix and are the plan's / operator's responsibility as
before.
