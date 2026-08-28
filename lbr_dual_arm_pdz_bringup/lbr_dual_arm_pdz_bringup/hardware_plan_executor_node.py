"""Thin shim: plan_executor_node.py with the real/mock FRI defaults prepended.

Running

    ros2 run lbr_dual_arm_pdz_bringup hardware_plan_executor_node [args...]

is exactly

    ros2 run lbr_dual_arm_pdz_bringup plan_executor_node \
        --controller-topology combined --robot-name lbr_dual_arm [args...]

i.e. the single combined 14-joint joint_trajectory_controller (dual_arm_controllers.yaml)
under /lbr_dual_arm, wall clock, raise-on-failure. Pass --gripper (or --gripper-backend
servo) to also home + actuate the servo_gripper_julien grippers -- source
setup_gripper_env.sh first. Every other flag (--arm-control, --on-failure, --gripper-torque,
--robot-name to override the namespace, ...) is forwarded to plan_executor_node unchanged.

All behaviour, the arg matrix, and the "why one node for every rig" analysis live in
plan_executor_node.py / plan_executor_node.md. There is no logic here.
"""
import sys

from lbr_dual_arm_pdz_bringup.plan_executor_node import main as _unified_main

# Defaults applied only when the caller has not set them explicitly, so overriding e.g.
# --robot-name on the command line still works.
_HARDWARE_DEFAULTS = (
    ("--controller-topology", "combined"),
    ("--robot-name", "lbr_dual_arm"),
)


def main(args=None):
    for flag, value in _HARDWARE_DEFAULTS:
        if flag not in sys.argv:
            sys.argv[1:1] = [flag, value]
    _unified_main(args)


if __name__ == "__main__":
    main()
