"""Waypoint driver for the S2 loop simulation.

The movement engine listens for two plain numbers: an absolute heading on
TURN_TOPIC, then a distance on MOVE_TOPIC. This module turns a list of star
coordinates into that stream of commands.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from s2_loop_sim.constants import (
    ANGULAR_SPEED,
    COMMAND_QUEUE_DEPTH,
    CONTROL_PERIOD,
    LINEAR_SPEED,
    MOVE_TOPIC,
    TURN_TOPIC,
)


def angle_to(robot_x, robot_y, target_x, target_y):
    """Absolute heading, in radians, from the robot to the target point.

    `atan2(y, x)` is the standard way to convert a 2D direction into an angle.
    The movement engine treats 0 radians as facing along +X, with positive
    angles rotating toward +Y.
    """
    return math.atan2(target_y - robot_y, target_x - robot_x)


def distance_to(robot_x, robot_y, target_x, target_y):
    """Straight-line distance, in metres, from the robot to the target point."""
    return math.hypot(target_x - robot_x, target_y - robot_y)


def wait_for_turn(start_angle, target_angle):
    """Seconds the movement engine needs to finish this turn, plus one tick."""
    shortest_turn = math.atan2(
        math.sin(target_angle - start_angle),
        math.cos(target_angle - start_angle),
    )
    return abs(shortest_turn) / ANGULAR_SPEED + CONTROL_PERIOD


def wait_for_drive(distance):
    """Seconds the movement engine needs to finish this drive, plus one tick."""
    return distance / LINEAR_SPEED + CONTROL_PERIOD


class WaypointDriver(Node):
    """Publishes one turn command and one drive command for each star.

    This node keeps the same simple pose estimate as movement_engine.py: it
    starts at (0, 0), facing +X, and assumes each command completes before the
    next one is sent. There is no obstacle handling yet.
    """

    def __init__(self, waypoints):
        super().__init__('waypoint_driver')
        self.waypoints = waypoints
        self.next_waypoint = 0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.turn_publisher = self.create_publisher(
            Float64, TURN_TOPIC, COMMAND_QUEUE_DEPTH)
        self.move_publisher = self.create_publisher(
            Float64, MOVE_TOPIC, COMMAND_QUEUE_DEPTH)

        # Give the movement engine time to create its subscriptions before the
        # first command is published. ROS topics do not hold old messages for
        # future subscribers by default.
        self.create_one_shot_timer(1.0, self.start_next_waypoint)

    def create_one_shot_timer(self, delay, callback):
        """Run `callback` once after `delay` seconds.

        ROS timers repeat until cancelled, unlike a one-time sleep. Cancelling
        before the callback runs keeps command scheduling predictable.
        """
        timer = None

        def run_once():
            timer.cancel()
            callback()

        timer = self.create_timer(delay, run_once)

    def start_next_waypoint(self):
        """Point the robot at the next star, then schedule the drive command."""
        if self.next_waypoint >= len(self.waypoints):
            self.get_logger().info('Visited every waypoint star')
            rclpy.shutdown()
            return

        target_x, target_y = self.waypoints[self.next_waypoint]
        target_yaw = angle_to(self.x, self.y, target_x, target_y)
        distance = distance_to(self.x, self.y, target_x, target_y)

        self.turn_publisher.publish(Float64(data=target_yaw))
        self.create_one_shot_timer(
            wait_for_turn(self.yaw, target_yaw),
            lambda: self.drive_to_waypoint(target_x, target_y, target_yaw,
                                           distance),
        )

    def drive_to_waypoint(self, target_x, target_y, target_yaw, distance):
        """Drive to the already selected star, then schedule the next star."""
        self.move_publisher.publish(Float64(data=distance))

        self.x = target_x
        self.y = target_y
        self.yaw = target_yaw
        self.next_waypoint += 1

        self.create_one_shot_timer(wait_for_drive(distance),
                                   self.start_next_waypoint)


def parse_waypoints(arguments):
    """Convert command-line numbers into [(x, y), ...] waypoint pairs."""
    if len(arguments) % 2 != 0:
        raise ValueError('waypoints must be passed as x y pairs')

    numbers = [float(argument) for argument in arguments]
    return list(zip(numbers[0::2], numbers[1::2]))


def main(args=None):
    """Start the waypoint driver node."""
    rclpy.init(args=args)
    rclpy.spin(WaypointDriver(parse_waypoints(sys.argv[1:])))
    if rclpy.ok():
        rclpy.shutdown()
