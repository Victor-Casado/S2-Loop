"""Stateful waypoint navigation for the simulated vehicle."""
import math
import time

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


READY_TO_TURN = 'ready_to_turn'
READY_TO_DRIVE = 'ready_to_drive'


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


class WaypointDriver(Node):
    """ROS node state for driving through waypoint stars.

    The movement engine does not publish pose, so this class keeps the matching
    pose estimate and schedules the next command after the current one should be
    complete.
    """

    def __init__(self, waypoints):
        super().__init__('waypoint_driver')
        self.waypoints = waypoints
        self.next_waypoint = 0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.state = READY_TO_TURN
        self.wait_until = time.monotonic() + 1.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.distance = 0.0

        self.turn_publisher = self.create_publisher(
            Float64, TURN_TOPIC, COMMAND_QUEUE_DEPTH)
        self.move_publisher = self.create_publisher(
            Float64, MOVE_TOPIC, COMMAND_QUEUE_DEPTH)

        self.create_timer(CONTROL_PERIOD, self.step)

    def step(self):
        """Start the next turn or drive once the previous command has finished."""
        if time.monotonic() < self.wait_until:
            return

        if self.state == READY_TO_TURN:
            self.turn_to_next_waypoint()
        else:
            self.drive_to_waypoint()

    def turn_to_next_waypoint(self):
        """Publish the turn command for the next star."""
        if self.next_waypoint >= len(self.waypoints):
            self.get_logger().info('Visited every waypoint star')
            rclpy.shutdown()
            return

        self.target_x, self.target_y = self.waypoints[self.next_waypoint]
        self.target_yaw = angle_to(
            self.x, self.y, self.target_x, self.target_y)
        self.distance = distance_to(
            self.x, self.y, self.target_x, self.target_y)

        self.turn_publisher.publish(Float64(data=self.target_yaw))
        self.state = READY_TO_DRIVE
        self.wait_until = time.monotonic() + wait_for_turn(
            self.yaw, self.target_yaw)

    def drive_to_waypoint(self):
        """Publish the drive command, then queue the next star."""
        self.move_publisher.publish(Float64(data=self.distance))

        self.x = self.target_x
        self.y = self.target_y
        self.yaw = self.target_yaw
        self.next_waypoint += 1
        self.state = READY_TO_TURN
        self.wait_until = time.monotonic() + wait_for_drive(self.distance)
