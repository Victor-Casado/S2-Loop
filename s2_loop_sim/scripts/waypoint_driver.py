#!/usr/bin/env python3
"""Drive the simulated vehicle through each waypoint star in order."""
import sys

import rclpy
from rclpy.utilities import remove_ros_args

from s2_loop_sim.waypoint_navigation import WaypointDriver


def parse_waypoints(arguments):
    """Convert command-line numbers into [(x, y), ...] waypoint pairs."""
    if len(arguments) % 2 != 0:
        raise ValueError('waypoints must be passed as x y pairs')

    numbers = [float(argument) for argument in arguments]
    return list(zip(numbers[0::2], numbers[1::2]))


def main(args=None):
    """Start the waypoint driver node."""
    raw_args = sys.argv if args is None else ['waypoint_driver.py', *args]
    waypoint_args = remove_ros_args(args=raw_args)[1:]
    rclpy.init()
    rclpy.spin(WaypointDriver(parse_waypoints(waypoint_args)))
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
