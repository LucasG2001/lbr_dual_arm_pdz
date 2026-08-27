#!/usr/bin/env python3
"""Publishes static box CollisionObjects into the dual-arm KUKA mock MoveIt
planning scene (move_group.launch.py mode:=mock), e.g. a box resting on the
table for collision-aware mock planning.

Object frame/size/pose come entirely from ../config/mock_scene_objects.yaml,
not launch arguments or CLI flags -- by design, so the fixed scene setup is
the same regardless of which launch file starts this node, instead of being
re-specified per invocation. --config only selects *which* YAML file to load.

lbr_dual_arm_bringup is an ament_cmake package with no installed-Python-node
support, so this is a plain script (not a console_script/ros2-run
executable) started via ExecuteProcess from move_group.launch.py -- same
pattern as that launch's own camera-rig publisher precedent in
mock.launch.py (`python3 -m ...`). Because ExecuteProcess doesn't
participate in launch_ros's PushRosNamespace the way a launch_ros Node does,
move_group.launch.py passes the namespace explicitly via
`--ros-args -r __ns:=/<robot_name>` so this node's relative "planning_scene"
publisher still resolves to the same /<robot_name>/planning_scene topic
move_group itself listens on, instead of a bare, unnamespaced one.

Usage:
    ros2 launch lbr_dual_arm_bringup move_group.launch.py mode:=mock
    python3 scripts/publish_mock_scene_objects.py [--config /path/to/other.yaml]
"""
import argparse
import math
import os
import sys

import rclpy
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneWorld
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG_PATH = os.path.join(SCRIPT_DIR, "..", "config", "mock_scene_objects.yaml")

# /planning_scene has no transient-local QoS, so a subscriber that connects
# after the first publish (move_group started later, RViz opened late, ...)
# would otherwise never see these fixed objects -- republish periodically
# instead of once, same reasoning as Masterthesis-vision's
# publish_camera_scene_objects.py.
REPUBLISH_PERIOD_S = 2.0


def _rpy_rad_to_quaternion_xyzw(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _make_box_collision_object(spec: dict, frame_id: str, stamp) -> CollisionObject:
    size = spec['size_m']
    position = spec['position_m']
    roll, pitch, yaw = spec.get('rpy_rad', [0.0, 0.0, 0.0])

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(size[0]), float(size[1]), float(size[2])]

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (
        float(position[0]), float(position[1]), float(position[2])
    )
    qx, qy, qz, qw = _rpy_rad_to_quaternion_xyzw(float(roll), float(pitch), float(yaw))
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw

    obj = CollisionObject()
    obj.header = Header(frame_id=frame_id, stamp=stamp)
    obj.id = spec['id']
    obj.primitives = [primitive]
    obj.primitive_poses = [pose]
    obj.operation = CollisionObject.ADD
    return obj


class MockScenePublisher(Node):
    def __init__(self, config_path: str) -> None:
        super().__init__('mock_scene_object_publisher')

        with open(config_path) as f:
            config = yaml.safe_load(f)
        self._frame_id = config['frame_id']
        self._object_specs = config['objects']

        self._pub = self.create_publisher(PlanningScene, 'planning_scene', 1)
        self.get_logger().info(
            f"Publishing {len(self._object_specs)} mock scene object(s) "
            f"{[spec['id'] for spec in self._object_specs]} in frame "
            f"'{self._frame_id}' from {config_path}"
        )
        self.create_timer(REPUBLISH_PERIOD_S, self._publish)
        self._publish()

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        objects = [
            _make_box_collision_object(spec, self._frame_id, stamp)
            for spec in self._object_specs
        ]
        self._pub.publish(
            PlanningScene(is_diff=True, world=PlanningSceneWorld(collision_objects=objects))
        )


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--config', default=DEFAULT_CONFIG_PATH,
        help='YAML file defining the frame_id and box objects to publish '
             '(default: %(default)s)'
    )
    cli_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:] if args is None else args
    parsed = parser.parse_args(cli_args)

    rclpy.init(args=args)
    node = MockScenePublisher(parsed.config)
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
