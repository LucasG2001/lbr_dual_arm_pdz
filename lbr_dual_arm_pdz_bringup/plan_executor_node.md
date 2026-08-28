# plan_executor_node.py

Replays a Fabrica `motion.pkl` plan on the Gazebo dual-arm KUKA + pdz-gripper rig via the
standard `FollowJointTrajectory` action. The Gazebo counterpart of
`hardware_plan_executor_node.py` (real/mock hardware); both live in
`lbr_dual_arm_pdz_bringup/lbr_dual_arm_pdz_bringup/`. See that file's `.md` for the
controller-topology comparison table.

This doc covers three changes made **2026-08-27**, all prompted by an impedance-control
replay run:

```bash
python3 ~/franka_ros2_ws/src/lbr_fri_ros2_stack/lbr_demos/lbr_dual_arm/lbr_dual_arm_pdz_bringup/\
lbr_dual_arm_pdz_bringup/plan_executor_node.py --arm-control cartesian_impedance
```

Only `plan_executor_node.py` was touched. One new import (`std_msgs.msg.Float64MultiArray`).
No `package.xml` / `setup.py` change — the node is run directly with `python3`.

---

## 1. Goal-tolerance violations now emit the violation amount, not just a warning

### Before

Under `--arm-control cartesian_impedance` the `cartesian_impedance_controller` aborts a goal
with `GOAL_TOLERANCE_VIOLATED` routinely — a compliant arm rarely settles *every* joint
inside `trajectory_default_goal_tolerance` within the goal-time window. `_run_goal()` caught
that and logged a bare:

```
[lbr_one] goal tolerance not met (impedance control, expected) -- continuing
```

with no numbers, so there was no way to tell a 2 mm miss from a 5 cm one from the log.

### After

`_run_goal()` now calls `_goal_tol_violation_report(arm, joint_names, points, result)`, which
builds a quantified line from up to three parts:

**a) Joint-space goal miss** — `commanded final point − measured`, per joint, sorted
worst-first, in rad and deg, plus `max |miss|`.

- Source: the goal's last `JointTrajectoryPoint.positions` (what we commanded) vs. the last
  `FollowJointTrajectory` feedback's `actual.positions`. Name-keyed (`dict(zip(...))`) so a
  feedback/command joint-ordering mismatch can't misalign them.
- A `feedback_callback` on `send_goal_async` caches the last feedback in `self._last_feedback`
  (reset to `None` at the start of every goal).
- **Why not the feedback `error` field:** `error` is `desired − actual` = the *running
  setpoint-tracking* error. Under impedance control the interpolated setpoint chases the
  measured state, so it sits around 1e-5 rad and **does not explain the abort** — this was the
  original ~`0.00001 rad` red herring. It is still printed, but only as a secondary
  `setpoint tracking error at abort: ...` line and only when some component exceeds 1e-4 rad.

**b) 6D Cartesian error** — see §2.

**c) Controller `error_string`** — `result.result.error_string`, appended verbatim when the
controller sets one (the gz `cartesian_impedance_controller` currently does not).

Non-impedance failures, and any non-`GOAL_TOLERANCE_VIOLATED` failure, get the same report on
an `error`-level log instead of `warn`, and include `error_code=<n>`.

### Example

```
[lbr_two] goal tolerance not met (impedance control, expected) -- continuing. goal
tolerance violation (commanded final point - measured): max |miss|=0.04812 rad (2.757
deg); per joint [lbr_two_A4=+0.04812 rad (+2.757 deg), lbr_two_A6=-0.02110 rad (-1.209
deg), ...]; cartesian error (impedance target - EE, base frame): |trans|=2.140 mm
[dx=+1.200, dy=-1.550, dz=+0.600 mm]; |rot|=0.905 deg [rx=+0.100, ry=-0.850, rz=+0.150
deg]; 6D vector (m,rad)=[+0.00120, -0.00155, +0.00060, +0.00175, -0.01484, +0.00262]
||6D||=0.01520
```

---

## 2. 6D Cartesian error, sourced from the controller's debug topic

No FK is done in this node: `kdl_parser_py` is not installed on this machine, and `PyKDL`
alone would need a hand-built KDL tree from the URDF.

Instead the node subscribes to the impedance controller's own debug topic:

`/<robot_name>/cartesian_impedance_<arm>/data_impedance`
(`std_msgs/Float64MultiArray`, length 14, published every control cycle by
`src/kuka_lbr_control/controllers/cartesian_impedance_controller/src/cartesian_impedance_controller.cpp`,
around line 715).

Layout used here — `data[0:6]` = `motion_error` = `[target − EE]` in the **base frame**:

| index | meaning |
|---|---|
| `[0:3]` | position error xyz, metres |
| `[3:6]` | rotation error as a Rodrigues (axis·angle) vector xyz, radians |

The controller drives its target frame from the sampled joint trajectory via FK, so at a
segment's end `data[0:6]` **is** the Cartesian goal error. (`computeMotionError()` clamps it
controller-side at `|pos| ≤ 1 m` / `|rot| ≤ 1 rad` — never reached at goal-tolerance scale.)

### Wiring

- `__init__` creates the two subscriptions (`cartesian_impedance_lbr_one` /
  `cartesian_impedance_lbr_two`) **only when `arm_control == 'cartesian_impedance'`**, caching
  the latest `list(msg.data)` per arm in `self._cart_err`.
- `_cartesian_error_str(arm)` formats:
  - `|trans|` = ‖position‖ in mm, plus `dx/dy/dz` mm
  - `|rot|` = ‖rotation‖ in deg, plus `rx/ry/rz` deg
  - the raw 6-vector `(m,rad)` and `||6D||` — the Euclidean norm of that mixed-unit vector
    (this is the "6D" number; the split `|trans|` / `|rot|` are the physically meaningful
    magnitudes)
- Returns `None` (line silently omitted) if no `data_impedance` sample has arrived for that
  arm yet, or for gripper goals (position controller — no such topic).

---

## 3. Gripper controller wait is non-fatal when running in Gazebo

### Before

`_client()` did a hard 20 s `wait_for_server` for *every* controller and
`raise RuntimeError` if the server was absent. Under `--arm-control cartesian_impedance` the
gripper is a **separate** `gripper_controller_lbr_{one,two}` (position controller), so a
Gazebo bringup that didn't spawn those made the whole node abort before executing anything.

### After

`_client(ctrl, require=True, timeout_sec=20.0)`:

- **Arm** clients: unchanged — `require=True`, 20 s, still `raise` on timeout (a missing arm
  controller almost always means the node's `--arm-control` doesn't match the bringup's
  `arm_control` launch arg).
- **Gripper** clients: created with `require=False, timeout_sec=5.0`. On timeout `_client`
  logs a warning, stores `None`, and returns `None`.
- `_send_gripper_command()` checks for `None`: logs
  `gripper <desc> ... skipped -- no gripper controller available` and returns **without
  sending a goal**, but still updates `self.held_part` — so the `traj.npy` held-part teleport
  following (`/world/<world>/set_pose`) is unaffected.

In the default `--arm-control joint_trajectory` mode the gripper joint is carried by the same
`joint_trajectory_controller_lbr_*` as the arm, so the gripper "client" resolves to the
already-created (required) arm client and this path never triggers.

---

## Caveats / not done

- Both the joint-space miss and the 6D error read the **last** feedback / **last**
  `data_impedance` sample. If the controller stops publishing a beat before the abort they
  lag the truly-settled state slightly — still real mm/deg-scale numbers, not zero.
- The configured `trajectory_default_goal_tolerance` value is not read, so the report shows
  the raw miss, not `miss − tolerance`.
- `||6D||` mixes metres and radians by construction — use `|trans|` (mm) and `|rot|` (deg)
  for anything quantitative.
- Everything here is impedance-path only; `--arm-control joint_trajectory` behaviour is
  unchanged apart from the (never-triggered) gripper-client fallthrough.
