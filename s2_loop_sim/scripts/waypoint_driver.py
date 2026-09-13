#!/usr/bin/env python3
"""Drive the simulated vehicle through each waypoint star in order."""
import sys

import rclpy

from s2_loop_sim.waypoint_navigation import WaypointDriver


def parse_waypoints(arguments):
    """Convert command-line numbers into [(x, y), ...] waypoint pairs."""
    if len(arguments) % 2 != 0:
        raise ValueError('waypoints must be passed as x y pairs')

    numbers = [float(argument) for argument in arguments]
    return list(zip(numbers[0::2], numbers[1::2]))


def main(args=None):
    """Start the waypoint driver node."""
    waypoint_args = sys.argv[1:] if args is None else args
    rclpy.init()
    rclpy.spin(WaypointDriver(parse_waypoints(waypoint_args)))
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
