# QUICKSTART — bringup + plan executor

Copy-paste commands for every rig. Each block is **two terminals**: one for the bringup
launch, one for the executor. Source the workspace in both:

```bash
source /opt/ros/humble/setup.bash && source ~/franka_ros2_ws/install/setup.bash
```

The plan directory (`motion.pkl`, and `traj.npy` for Gazebo held-part viz) defaults to
`~/Fabrica/logs/plumbers_block_sim`; pass a different one as the first positional arg to the
executor.

| Rig | Bringup launch | Executor |
|---|---|---|
| Gazebo, position | `gazebo.launch.py` | `plan_executor_node` (per-arm) |
| Gazebo, fixed-gain impedance | `gazebo.launch.py arm_control:=cartesian_impedance` | `plan_executor_node --arm-control cartesian_impedance` |
| Gazebo, variable-gain impedance | `gazebo.launch.py arm_control:=cartesian_impedance` | `variable_impedance_plan_executor_node` |
| Mock hardware, position | `lbr_dual_arm_bringup mock.launch.py` | `hardware_plan_executor_node` |
| Real FRI, position | `lbr_dual_arm_bringup hardware.launch.py` | `hardware_plan_executor_node [--gripper]` |
| Real FRI, fixed-gain impedance | `lbr_dual_arm_bringup cartesian_impedance.launch.py` | `hardware_plan_executor_node --arm-control cartesian_impedance` |
| Real FRI, variable-gain impedance | `lbr_dual_arm_bringup cartesian_impedance.launch.py` | `variable_impedance_plan_executor_node` |

Only the bringup launch + args change between rigs — the executor is one node
(`hardware_plan_executor_node` is a thin shim over `plan_executor_node` with the FRI
defaults). Full behaviour / arg matrix: **`plan_executor_node.md`**.

---

## Gazebo — position control

```bash
# terminal 1
ros2 launch lbr_dual_arm_pdz_bringup gazebo.launch.py

# terminal 2
ros2 run lbr_dual_arm_pdz_bringup plan_executor_node \
  --robot-name lbr_dual_arm_pdz \
  --gripper-backend sim \
  --visualize-held-parts --world plumbers_block \
  --use-sim-time \
  --on-failure continue
```

`gazebo.launch.py` args: `robot_name` (default `lbr_dual_arm_pdz`), `arm_control`
(`joint_trajectory` | `cartesian_impedance`), and — to start the executor from the launch
itself — `run_executor:=true` `gripper_backend:=sim|servo|none` `log_dir:=<dir>`:

```bash
ros2 launch lbr_dual_arm_pdz_bringup gazebo.launch.py run_executor:=true
```

(`--visualize-held-parts` needs the gz `/world/<world>/set_pose` service; if it's absent the
executor logs one warning and keeps going — arm/gripper motion is unaffected.)

---

## Gazebo — Cartesian impedance (fixed gains)

```bash
# terminal 1
ros2 launch lbr_dual_arm_pdz_bringup gazebo.launch.py arm_control:=cartesian_impedance

# terminal 2
ros2 run lbr_dual_arm_pdz_bringup plan_executor_node \
  --robot-name lbr_dual_arm_pdz \
  --arm-control cartesian_impedance \
  --gripper-backend sim \
  --visualize-held-parts --world plumbers_block \
  --use-sim-time
```

Impedance is always per-arm (`cartesian_impedance_lbr_{one,two}`) — `--controller-topology`
has no effect. `GOAL_TOLERANCE_VIOLATED` is tolerated automatically (a compliant arm rarely
settles every joint inside the goal tolerance).

---

## Gazebo — variable (per-phase) impedance

```bash
# terminal 1
ros2 launch lbr_dual_arm_pdz_bringup gazebo.launch.py arm_control:=cartesian_impedance

# terminal 2
ros2 run lbr_dual_arm_pdz_bringup variable_impedance_plan_executor_node \
  ~/Fabrica/logs/plumbers_block_sim lbr_dual_arm_pdz \
  --dwell 2.0
```

`variable_impedance_plan_executor_node` positional args: `[log_dir] [robot_name]`
(robot_name **must** be `lbr_dual_arm_pdz` for Gazebo — its default is `lbr_dual_arm`).
Other args: `--gripper` `--gripper-torque N` `--dwell SEC` (hold-still after each stiffness
change) `--no-holder-brace`. It varies task-space stiffness per motion phase; see the module
docstring in `variable_impedance_plan_executor_node.py`.

---

## Mock hardware — position control

```bash
# terminal 1
ros2 launch lbr_dual_arm_bringup mock.launch.py

# terminal 2
ros2 run lbr_dual_arm_pdz_bringup hardware_plan_executor_node
```

`mock.launch.py` args: `ctrl` (default `joint_trajectory_controller`), `robot_name`
(default `lbr_dual_arm`), `use_gripper` (default `true`). Wall clock, one combined 14-joint
`joint_trajectory_controller`. Arm-only (grippers are mock stubs — do not pass `--gripper`).

---

## Real FRI hardware — position control

```bash
# terminal 1
ros2 launch lbr_dual_arm_bringup hardware.launch.py

# terminal 2 — arm only
ros2 run lbr_dual_arm_pdz_bringup hardware_plan_executor_node

# terminal 2 — arm + real servo_gripper_julien grippers
source ~/franka_ros2_ws/src/servo_gripper_julien/setup_gripper_env.sh
# ...and start the dual-gripper launch for /left + /right gripper_controller...
ros2 run lbr_dual_arm_pdz_bringup hardware_plan_executor_node --gripper --gripper-torque 250
```

`hardware.launch.py` args: `ctrl` (default `joint_trajectory_controller`), `robot_name`
(default `lbr_dual_arm`), `use_gripper` (default `true`), `arms` (`both` | `lbr_one` |
`lbr_two` — which arm(s) connect as real FRI; the other is loaded mock).

`--gripper` (= `--gripper-backend servo`) homes + actuates the servo grippers — **real
hardware, no sim variant**, so it moves even in an arm-mock dry run. Needs
`setup_gripper_env.sh` sourced and `gripper_controller` running for both `/left` and
`/right`. On abort the executor calls `~/stop` on both grippers.

Default failure policy is `--on-failure raise` (stop the whole plan on any rejected/failed
goal). `_timed_waypoints()` blends any segment whose implied velocity would trip the FRI
`CommandGuard` — see `plan_executor_node.md` ("Waypoint timing and blending").

---

## Real FRI hardware — Cartesian impedance

Impedance on the real rig has its **own bringup launch** (`cartesian_impedance.launch.py` —
torque-mode twin of `hardware.launch.py`; there is no mock variant):

```bash
# terminal 1
ros2 launch lbr_dual_arm_bringup cartesian_impedance.launch.py

# terminal 2 — fixed gains
ros2 run lbr_dual_arm_pdz_bringup hardware_plan_executor_node \
  --arm-control cartesian_impedance

# terminal 2 — variable per-phase gains  (both positionals default correctly on real hw)
ros2 run lbr_dual_arm_pdz_bringup variable_impedance_plan_executor_node --dwell 2.0
```

`cartesian_impedance.launch.py` args: `robot_name` (default `lbr_dual_arm`), `use_gripper`
(default `true`). Brings up `cartesian_impedance_lbr_one/_two` sequentially (see
`dual_arm_cartesian_impedance_controllers.yaml`). Add `--gripper` to either executor for the
servo grippers (source `setup_gripper_env.sh` first).

---

## Optional — MoveIt / RViz visualization (any rig)

```bash
ros2 launch lbr_dual_arm_moveit_config moveit_rviz.launch.py mode:=mock
#   mode:=mock | hardware | gazebo ,  robot_name:=lbr_dual_arm (or lbr_dual_arm_pdz)
```

Visualization only — the executors talk to the controllers directly, not through move_group.
