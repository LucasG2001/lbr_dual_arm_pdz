lbr_dual_arm_description
========================

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

Frame Conventions
------------------
Each arm's ``iiwa7`` macro (``lbr_description``) defines ``{robot_name}_link_ee``
as a fixed child of ``{robot_name}_link_7``, offset ``0.035 m`` along local Z.
This is the canonical KUKA hardware flange frame.

``y_gripper.xacro``'s ``gripper_mount_joint`` mounts ``{robot_name}_gripper_base_link``
directly on ``{robot_name}_link_ee`` (zero offset, only a ``mount_yaw`` rotation),
so ``gripper_base_link``, ``link_ee``, and the physical flange all coincide.
``gripper_tcp`` then sits ``0.1455 m`` further along local Z from that shared
frame. Do not reparent the gripper mount back onto ``link_7`` directly --
that would reintroduce a ``0.035 m`` gap between where the gripper visually/
physically attaches and the ``link_ee`` frame that ``lbr_ros2_control``'s
force/torque estimator and hand-eye calibration (``arm_one_flange`` /
``arm_two_flange`` MoveIt groups) are calibrated against.

``y_gripper_standalone.xacro`` (the debug-only single-gripper viewer) carries
its own stand-in ``lbr_one_link_7`` / ``lbr_one_link_ee`` pair, at the same
``0.035 m`` offset, so it stays consistent with the real robot.

Customize Robot Placement
-------------------------
#. Open ``urdf/lbr_dual_arm.xacro``.
#. Adjust the fixed joint origins:

    - ``lbr_one_base_joint``
    - ``lbr_two_base_joint``

#. Typical changes are:

    - Increase/decrease spacing between both arms via ``xyz``.
    - Rotate one arm around the base frame via ``rpy``.

Customize Hardware Network Settings
-----------------------------------
#. Open ``ros2_control/lbr_one_system_config.yaml`` and ``ros2_control/lbr_two_system_config.yaml``.
#. Set unique ``port_id`` values for each arm.
#. Set ``remote_host`` per arm according to your network setup.

.. note::
    ``port_id`` must be different for both arms.
