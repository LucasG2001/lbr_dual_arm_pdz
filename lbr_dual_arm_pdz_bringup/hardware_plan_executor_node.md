# hardware_plan_executor_node.py

> **Just want the commands?** See **[`QUICKSTART.md`](QUICKSTART.md)**.

**Merged into `plan_executor_node.py`.** This is now a thin shim that runs the unified node
with the real/mock FRI defaults prepended:

```
ros2 run lbr_dual_arm_pdz_bringup hardware_plan_executor_node [args...]
  ==  ros2 run lbr_dual_arm_pdz_bringup plan_executor_node \
        --controller-topology combined --robot-name lbr_dual_arm [args...]
```

i.e. the single combined 14-joint `joint_trajectory_controller` (`dual_arm_controllers.yaml`)
under `/lbr_dual_arm`, wall clock, raise-on-failure. Pass `--gripper` (or
`--gripper-backend servo`) to also home + actuate the `servo_gripper_julien` grippers — source
`setup_gripper_env.sh` first.

See **`plan_executor_node.md`** for everything: the full arg matrix, the "why one node for
every rig" analysis, the waypoint timing/blending writeup (incl. the FRI `CommandGuard` crash
this shim's predecessor was built around), the Cartesian-impedance goal-tolerance reporting,
and the `servo_gripper_julien` service interface.
