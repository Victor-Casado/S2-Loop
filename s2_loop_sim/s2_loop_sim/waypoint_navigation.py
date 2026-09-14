import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float64

from s2_loop_sim.constants import (
    COMMAND_QUEUE_DEPTH,
    CONTROL_PERIOD,
    MOVEMENT_DONE_TOPIC,
    MOVE_TOPIC,
    TURN_TOPIC,
)
from s2_loop_sim.geometry import angle_to, distance_to


READY_TO_TURN = 'ready_to_turn'
WAITING_FOR_TURN = 'waiting_for_turn'
WAITING_FOR_DRIVE = 'waiting_for_drive'


class WaypointDriver(Node):
    """ROS node state for driving through waypoint stars.

    The movement engine does not publish pose, so this class keeps the matching
    pose estimate. It waits for movement_engine.py to report that a command is
    done before sending the next one.
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
        self.create_subscription(
            Empty, MOVEMENT_DONE_TOPIC, self.movement_done,
            COMMAND_QUEUE_DEPTH)

        self.create_timer(CONTROL_PERIOD, self.step)

    def step(self):
        """Start the next turn once startup or the previous drive is done."""
        if self.state != READY_TO_TURN or time.monotonic() < self.wait_until:
            return
        if self.turn_publisher.get_subscription_count() == 0:
            return
        if self.move_publisher.get_subscription_count() == 0:
            return

        self.turn_to_next_waypoint()

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

        self.get_logger().info(
            f'Turning toward waypoint {self.next_waypoint + 1}: '
            f'heading={self.target_yaw:.2f} rad')
        self.turn_publisher.publish(Float64(data=self.target_yaw))
        self.state = WAITING_FOR_TURN

    def movement_done(self, _):
        """Send the next command after movement_engine.py finishes one."""
        if self.state == WAITING_FOR_TURN:
            self.drive_to_waypoint()
        elif self.state == WAITING_FOR_DRIVE:
            self.x = self.target_x
            self.y = self.target_y
            self.yaw = self.target_yaw
            self.next_waypoint += 1
            self.state = READY_TO_TURN
            self.wait_until = time.monotonic() + CONTROL_PERIOD

    def drive_to_waypoint(self):
        """Publish the drive command for the already selected star."""
        self.get_logger().info(
            f'Driving to waypoint {self.next_waypoint + 1}: '
            f'distance={self.distance:.2f} m')
        self.move_publisher.publish(Float64(data=self.distance))
        self.state = WAITING_FOR_DRIVE
