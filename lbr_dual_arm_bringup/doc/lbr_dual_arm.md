# lbr_dual_arm_bringup

Launch package for the dual-arm `iiwa7` demo on `humble`.

## Overview: two processes, one match to keep

Every MoveIt session for this rig is **two independent launches** that have to
agree with each other:

1. **A bring-up launch** (`mock.launch.py`, `hardware.launch.py`,
   `cartesian_impedance.launch.py`, ...). It starts `robot_state_publisher`
   and the `ros2_control_node` (the `controller_manager`), then spawns the
   `ros2_control` controllers. This is what decides the robot's **command
   interface** (`position` vs. `effort`/torque, via `lbr_dual_arm.xacro`'s
   `command_mode` arg) and **which controllers are running**.

2. **`move_group.launch.py`** (this package). It starts MoveIt's `move_group`
   node, and optionally RViz. Its `control_mode` argument selects the **MoveIt
   controller-manager config**
   (`lbr_dual_arm_moveit_config/config/moveit_<control_mode>_controllers.yaml`),
   which tells `move_group` *which* `FollowJointTrajectory` action(s) to push
   planned trajectories onto.

The planned trajectory only executes if the action name that `move_group` was
configured with (process 2) matches a controller that the bring-up actually
spawned (process 1). Pick a row from the table below and use both commands
from that row.

## The MoveIt controller managers

All configs use the same plugin,
`moveit_simple_controller_manager/MoveItSimpleControllerManager`. Only the
controller list and joint mapping differ.

### `moveit_controllers.yaml` — `control_mode:=""` (the default)

One controller, `joint_trajectory_controller`, spanning all 14 joints, action
namespace `follow_joint_trajectory`. This is the normal **position-control**
path. Pairs with the `position`-interface `joint_trajectory_controller` from
`lbr_dual_arm_description/ros2_control/dual_arm_controllers.yaml`.

### `moveit_cartesian_impedance_controllers.yaml` — `control_mode:=cartesian_impedance`

Two controllers, `cartesian_impedance_lbr_one` and
`cartesian_impedance_lbr_two`, 7 joints each, each exposing its own
`follow_joint_trajectory` action. Pairs with the `effort`-interface
`cartesian_impedance_controller` instances from
`dual_arm_cartesian_impedance_controllers.yaml` (torque command mode). Each
instance samples the incoming joint trajectory, drives its Cartesian task
target through FK, and uses the trajectory as the nullspace target (see
`CartesianImpedanceController::updateTrajectoryExecution()`). There is no
single 14-joint controller here because the underlying `effort_controller_base`
kinematic chain is single-chain, so each arm needs its own controller/action.

> **Note:** `mode` (mock vs. hardware URDF) and `control_mode` (which MoveIt
> controllers YAML) are **independent** arguments. `control_mode` does not
> change the URDF, and `mode` does not change the execution backend.

> **Note:**
> `lbr_dual_arm_moveit_config/config/moveit_admittance_controllers.yaml` exists
> but is not wired to a `control_mode` choice, and
> `lbr_dual_arm_moveit_config/launch/move_group.launch.py` is an older,
> standalone entry point (different arg name, `use_cartesian_impedance`) that
> the bring-up flow does not use. Always launch MoveIt through **this
> package's** `move_group.launch.py`.

## Bring-up scripts and their MoveIt pairing

| Bring-up launch (terminal 1) | URDF `mode` / `command_mode` | `ros2_control` config | Controllers spawned | MoveIt launch (terminal 2) |
| --- | --- | --- | --- | --- |
| `mock.launch.py` | mock / position | `dual_arm_controllers.yaml` | `joint_state_broadcaster`, `joint_trajectory_controller` | `move_group.launch.py` (defaults: `mode:=mock`, `control_mode:=""`) |
| `hardware.launch.py` | hardware / position | `dual_arm_controllers.yaml` | `joint_state_broadcaster`, `joint_trajectory_controller` | `move_group.launch.py mode:=hardware` |
| `cartesian_impedance.launch.py` | hardware / torque | `dual_arm_cartesian_impedance_controllers.yaml` | `joint_state_broadcaster`, `cartesian_impedance_lbr_one`, `cartesian_impedance_lbr_two` | `move_group.launch.py mode:=hardware control_mode:=cartesian_impedance` |
| `calibration.launch.py` | hardware / torque | `dual_arm_gravity_compensation_controllers.yaml` | `joint_state_broadcaster`, `gravity_compensation_lbr_one` / `_two` | not a MoveIt backend — hand-guiding only |
| `admittance.launch.py` | hardware / position | `dual_arm_admittance_controllers.yaml` | per arm: `lbr_state_broadcaster_*`, `lbr_joint_position_command_controller_*` | not a MoveIt backend — an external admittance client streams joint-position setpoints |

(`lbr_dual_arm_pdz_bringup/launch/gazebo.launch.py` is a separate Gazebo path
with its own combined `gz_ros2_control` controller_manager and an
`arm_control:=joint_trajectory|cartesian_impedance` arg; trajectories there are
replayed by `plan_executor_node`, not `move_group`. See
[Gazebo (gz-sim) simulation](#gazebo-gz-sim-simulation) below.)

## Normal position control

Mock (no hardware needed):

Terminal 1:

```bash
ros2 launch lbr_dual_arm_bringup mock.launch.py
```

Terminal 2:

```bash
ros2 launch lbr_dual_arm_bringup move_group.launch.py \
    rviz:=true
```

Hardware:

Terminal 1:

```bash
ros2 launch lbr_dual_arm_bringup hardware.launch.py
# optional: arms:=lbr_one  (or lbr_two) to bring up a single real arm,
# the other is loaded as a mock component
```

Terminal 2:

```bash
ros2 launch lbr_dual_arm_bringup move_group.launch.py \
    mode:=hardware \
    rviz:=true
```

`control_mode` is left at its default (`""`), so `move_group` loads
`moveit_controllers.yaml` and executes onto the 14-joint
`joint_trajectory_controller`.

## Cartesian impedance control

Hardware only (the bring-up switches the arms to torque command mode; there is
currently no mock variant).

Terminal 1:

```bash
ros2 launch lbr_dual_arm_bringup cartesian_impedance.launch.py
```

Terminal 2:

```bash
ros2 launch lbr_dual_arm_bringup move_group.launch.py \
    mode:=hardware \
    control_mode:=cartesian_impedance \
    rviz:=true
```

Now `move_group` loads `moveit_cartesian_impedance_controllers.yaml` and splits
each plan across `cartesian_impedance_lbr_one/follow_joint_trajectory` and
`cartesian_impedance_lbr_two/follow_joint_trajectory`. The arms track the plan
with a compliant, task-space PD response; stiffness / damping / nullspace gains
are adjustable at runtime (see the header of
`dual_arm_cartesian_impedance_controllers.yaml` and
`cartesian_impedance_dual_arm.py` in `Masterthesis-vision`).

## Gazebo (gz-sim) simulation

The Gazebo path lives in a **different package**, `lbr_dual_arm_pdz_bringup`,
and does **not** use `move_group` or any `moveit_*_controllers.yaml`. It brings
up the dual-arm rig with the **pdz gripper** (not the Y-gripper) inside gz-sim,
spawns the `plumbers_block` fixture + parts, and replays a pre-computed Fabrica
`motion.pkl` plan onto the arms through the plain `FollowJointTrajectory`
action.

### How it differs from the mock/hardware paths

| | mock / hardware | Gazebo (`gazebo.launch.py`) |
| --- | --- | --- |
| `controller_manager` | standalone `ros2_control_node` started by the launch | one combined **`gz_ros2_control`** plugin instance, comes up with the `<gazebo><plugin>` — there is no `ros2_control_node` |
| Namespace (`robot_name`) | `lbr_dual_arm` | `lbr_dual_arm_pdz` (deliberately different — avoids colliding with long-running `robot_state_publisher` on `lbr_dual_arm`) |
| URDF | `mode:=mock` / `mode:=hardware`, `use_gripper:=true` (Y-gripper) | `mode:=gazebo gripper:=pdz` |
| Controller-mode arg | `control_mode` (on `move_group.launch.py`) | `arm_control` (on `gazebo.launch.py`) |
| Execution | MoveIt `move_group` plans + executes | `plan_executor_node` replays a fixed `motion.pkl` timeline, one goal at a time |
| Controllers | one combined 14-joint controller (position path) | **two per-arm** controller instances (see below) |

### `arm_control` — the Gazebo equivalent of `control_mode`

`arm_control:=joint_trajectory` (default)
: Config `dual_arm_pdz_gazebo_controllers.yaml`. Two position
  `joint_trajectory_controller_lbr_one` / `_lbr_two`, each claiming its arm's
  7 joints **plus its own pdz gripper driving joint** — so one action client
  per arm serves both arm and gripper goals.

`arm_control:=cartesian_impedance`
: Config `dual_arm_gazebo_cartesian_impedance_controllers.yaml`. Two effort
  `cartesian_impedance_lbr_one` / `_lbr_two` (the same patched KUKA controller
  as the hardware path — samples the joint trajectory, drives the Cartesian
  target through FK, uses it as the nullspace target), **plus** two separate
  position `gripper_controller_lbr_one` / `_lbr_two` for the finger joints, so
  arm goals and gripper goals go to different clients. This YAML also flips
  `command_current_configuration` to `false` and `compensate_gravity` to
  `true` versus the hardware YAML — and `compensate_gravity: true` **must
  match the world's gravity** or the arms drift.

The URDF is passed the matching `arm_control:=<value>` automatically;
`arm_control:=cartesian_impedance` also forces the arm joints to an
effort-only command interface.

### Running it

Terminal 1 — position path (default):

```bash
ros2 launch lbr_dual_arm_pdz_bringup gazebo.launch.py
```

Terminal 1 — Cartesian-impedance path:

```bash
ros2 launch lbr_dual_arm_pdz_bringup gazebo.launch.py \
    arm_control:=cartesian_impedance
```

Terminal 2 — replay the plan (the `--arm-control` value **must match**
terminal 1's `arm_control`):

```bash
# position path
ros2 run lbr_dual_arm_pdz_bringup plan_executor_node

# cartesian-impedance path
ros2 run lbr_dual_arm_pdz_bringup plan_executor_node --arm-control cartesian_impedance
```

`plan_executor_node` walks `motion.pkl`'s single global timeline, sending one
`FollowJointTrajectory` goal at a time and blocking on its result, and
teleports each held part to match Fabrica's rendered `traj.npy` pose via
gz-sim's `set_pose` service. The real/mock counterpart is
`hardware_plan_executor_node` (see
`lbr_dual_arm_pdz_bringup/hardware_plan_executor_node.md`), which targets the
combined 14-joint `joint_trajectory_controller` from the hardware path
instead.

### Driving the Gazebo impedance rig with MoveIt instead of `plan_executor_node`

`plan_executor_node` is not the only client the Gazebo impedance rig can take.
With `arm_control:=cartesian_impedance`, each `cartesian_impedance_lbr_one` /
`_lbr_two` instance exposes a `follow_joint_trajectory` action — the *same*
`FollowJointTrajectory` interface (and the same trajectory-sampling / FK /
nullspace behaviour) as on real hardware. So `move_group` can plan and execute
onto the Gazebo arms directly, using the existing
`moveit_cartesian_impedance_controllers.yaml` (whose controller names
`cartesian_impedance_lbr_one` / `_lbr_two` and `action_ns:
follow_joint_trajectory` already match the Gazebo controllers).

Three things have to line up, and they are the only differences from the
hardware recipe:

| What | Hardware | Gazebo |
| --- | --- | --- |
| Namespace | `robot_name:=lbr_dual_arm` (default) | `robot_name:=lbr_dual_arm_pdz` — so MoveIt's action clients resolve to `/lbr_dual_arm_pdz/cartesian_impedance_lbr_one/follow_joint_trajectory`, the real server. The default namespace never connects. |
| Clock | wall time | `use_sim_time:=true` — so trajectory stamps and goal-time tolerance are checked against `/clock`, not wall time. Without it every execution aborts as `GOAL_TOLERANCE_VIOLATED`. |
| SRDF tip links | `use_gripper:=true` → Y-gripper TCP | `use_gripper:=false` → tips fall back to `lbr_*_link_ee`. The SRDF's Y-gripper tip/collision links do not exist in the pdz URDF, so `use_gripper:=true` fails to load the groups. |

Terminal 1:

```bash
ros2 launch lbr_dual_arm_pdz_bringup gazebo.launch.py \
    arm_control:=cartesian_impedance
```

Wait until both `cartesian_impedance_lbr_one` and `_lbr_two` report `active`
(`ros2 control list_controllers -c /lbr_dual_arm_pdz/controller_manager`).

Terminal 2 — MoveIt (replaces `plan_executor_node`):

```bash
ros2 launch lbr_dual_arm_bringup move_group.launch.py \
    mode:=gazebo \
    use_gripper:=false \
    control_mode:=cartesian_impedance \
    robot_name:=lbr_dual_arm_pdz \
    use_sim_time:=true \
    rviz:=true
```

Plan for the `arm_one`, `arm_two` or `both_arms` group and hit Execute (or send
a `MoveGroup` goal): `move_group` splits the trajectory across
`cartesian_impedance_lbr_one/follow_joint_trajectory` and
`cartesian_impedance_lbr_two/follow_joint_trajectory`, exactly as on hardware.

Caveats specific to this combination:

- **No gripper in the planning model.** `use_gripper:=false` means `move_group`
  plans against a bare flange; the pdz gripper geometry is not collision-checked
  and its finger joints are not actuated by MoveIt (there is no gripper planning
  group). Joint-space planning and execution for the 7 arm joints are
  unaffected — the arm chain is identical across gripper variants.
  - Side effect: Gazebo's `joint_state_broadcaster` still publishes the two
    `lbr_*_pdz_gripper_left_finger_joint` joints on `/joint_states`, which this
    gripper-less model does not contain, so `CurrentStateMonitor` would log
    `Joint '...' not found in model` at ERROR on every message (~50 Hz). It is
    harmless (all 14 arm joints are present; planning and execution work), and
    `move_group.launch.py` now raises the `moveit_robot_model.robot_model`
    logger to `FATAL` whenever `mode:=gazebo` to suppress the flood.
- **`gripper_controller_lbr_one` / `_lbr_two`** (the pdz finger controllers) are
  not in `moveit_cartesian_impedance_controllers.yaml`; drive them separately if
  needed.
- **World gravity vs `compensate_gravity`.** `dual_arm_gazebo_cartesian_impedance_controllers.yaml`
  sets `compensate_gravity: true`, which requires the gz-sim world's gravity to
  be **on**. `worlds/plumbers_block.sdf` currently ships with a "TEMP
  diagnostic" `<gravity>0 0 0</gravity>` — with that, the arms drift upward
  under the uncancelled `+G(q)` term. Restore world gravity (or set
  `compensate_gravity: false`) before expecting MoveIt trajectories to track.
- **Controller gains** still need to be sane for the arm to reach goal
  tolerance; `nullspace_stiffness` starts at `0.0` and
  `trajectory_nullspace_stiffness` at `75.0`, same tuning story as hardware.
- RViz runs on sim time too (`moveit_rviz.launch.py` now forwards
  `use_sim_time`); if its TF still lags, the Gazebo `/clock` bridge in
  `gazebo.launch.py` is what feeds it.

## Shared arguments (keep identical across both terminals)

| Argument | Meaning |
| --- | --- |
| `mode` | `mock` or `hardware`. Selects the URDF's `ros2_control` hardware plugin. Must match between the bring-up and `move_group.launch.py`. |
| `use_gripper` | `true` (default) attaches the Y-gripper to each flange. Must match between the two launches, or the URDF/SRDF `move_group` builds will not match the running robot. |
| `robot_name` | Namespace (default `lbr_dual_arm`) that `robot_state_publisher`, `ros2_control_node` and `move_group` all live under. `move_group` is pushed into it so its default topic/action names resolve to the running hardware. |
| `control_mode` (`move_group.launch.py` only) | `""` (default) → `moveit_controllers.yaml`; `cartesian_impedance` → `moveit_cartesian_impedance_controllers.yaml`. |
| `use_sim_time` (`move_group.launch.py` only) | `false` (default) for mock/hardware. Set `true` for a gz-sim rig (see [Driving the Gazebo impedance rig with MoveIt](#driving-the-gazebo-impedance-rig-with-moveit-instead-of-plan_executor_node)); forwarded to RViz too. |
| `rviz` (`move_group.launch.py` only) | `true` also launches RViz via `lbr_dual_arm_moveit_config/launch/moveit_rviz.launch.py`. |
