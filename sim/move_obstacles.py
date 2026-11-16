#!/usr/bin/env python3
"""
Drive the moving obstacles of a world back and forth along their lanes.

Usage: move_obstacles.py <lanes.yaml>   (with use_sim_time)
Each obstacle starts at its lane's end `a`, faces `b`, and moves forward at its
speed; at either end it reverses. Velocities are in the obstacle's own frame,
which the velocity-control system keeps from turning, so a reversal is a sign
change. Timing follows simulated time.
"""
import sys

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import yaml

lanes = yaml.safe_load(open(sys.argv[1]))['lanes']
rclpy.init()
node = Node('move_obstacles', parameter_overrides=[Parameter('use_sim_time', value=True)])
publishers = [node.create_publisher(Twist, f"/model/{lane['name']}/cmd_vel", 10) for lane in lanes]
periods = [
    ((lane['b'][0] - lane['a'][0]) ** 2 + (lane['b'][1] - lane['a'][1]) ** 2) ** 0.5
    / lane['speed']
    for lane in lanes
]


def tick():
    t = node.get_clock().now().nanoseconds / 1e9
    if t <= 0.0:
        return
    for lane, publisher, period in zip(lanes, publishers, periods):
        forward = int(t / period) % 2 == 0
        msg = Twist()
        msg.linear.x = lane['speed'] if forward else -lane['speed']
        publisher.publish(msg)


node.create_timer(0.05, tick)
rclpy.spin(node)
