#!/usr/bin/env python3
"""Drive the simulated vehicle through each waypoint star in order."""
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
from s2_loop_sim.geometry import angle_to, distance_to


def wait_for_turn(start_angle, target_angle):
    """Seconds needed to finish the turn, plus one control tick."""
    error = math.atan2(
        math.sin(target_angle - start_angle),
        math.cos(target_angle - start_angle),
    )
    return abs(error) / ANGULAR_SPEED + CONTROL_PERIOD


def wait_for_drive(distance):
    """Seconds needed to finish the drive, plus one control tick."""
    return distance / LINEAR_SPEED + CONTROL_PERIOD


def parse_waypoints(arguments):
    """Convert command-line numbers into [(x, y), ...] waypoint pairs."""
    if len(arguments) % 2 != 0:
        raise ValueError('waypoints must be passed as x y pairs')

    numbers = [float(argument) for argument in arguments]
    return list(zip(numbers[0::2], numbers[1::2]))


class WaypointDriver(Node):
    """Turns toward each star, drives to it, then starts the next one.

    The movement engine does not publish pose, so this node keeps the same pose
    estimate locally and waits long enough for each command to finish.
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

        self.create_one_shot_timer(1.0, self.start_next_waypoint)

    def create_one_shot_timer(self, delay, callback):
        """ROS timers repeat by default, so cancel this one before it runs."""
        timer = None

        def run_once():
            timer.cancel()
            callback()

        timer = self.create_timer(delay, run_once)

    def start_next_waypoint(self):
        """Publish the turn command for the next star."""
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
        """Publish the drive command, then queue the next star."""
        self.move_publisher.publish(Float64(data=distance))

        self.x = target_x
        self.y = target_y
        self.yaw = target_yaw
        self.next_waypoint += 1

        self.create_one_shot_timer(wait_for_drive(distance),
                                   self.start_next_waypoint)


def main(args=None):
    """Start the waypoint driver node."""
    waypoint_args = sys.argv[1:] if args is None else args
    rclpy.init(args=args)
    rclpy.spin(WaypointDriver(parse_waypoints(waypoint_args)))
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
