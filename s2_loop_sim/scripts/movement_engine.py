#!/usr/bin/env python3
"""Drive the Gazebo vehicle in response to /turnto and /moveforward.

Gazebo is the physics simulator that owns the 3D world. ROS 2 is a separate
message-passing framework, connected to Gazebo by a bridge process. A ROS 2 node
is one participant on that network, and this file is a single node. Nodes talk
over topics, which are named broadcast channels with no reply, and services,
which are request/response calls. Float64, Pose, Entity and SetEntityPose are
classes generated from schema files rather than written by hand.

This node dead reckons. It keeps its own pose, advances it one small step per
tick, and teleports the model to match. Gazebo is never asked where the vehicle
is, because nothing else moves it. Nothing here is physical, so the vehicle will
drive through an obstacle until collision checks exist.
"""
import math

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Float64

from s2_loop_sim.constants import (
    COMMAND_QUEUE_DEPTH,
    CONTROL_PERIOD,
    FLOAT_TOLERANCE,
    METRES_PER_TICK,
    MOVE_TOPIC,
    RADIANS_PER_TICK,
    RIDE_HEIGHT,
    SET_POSE_SERVICE,
    TURN_TOPIC,
    VEHICLE_NAME,
)


def shortest_turn(angle):
    """`angle` folded into [-pi, pi], so a turn never goes the long way round."""
    return math.atan2(math.sin(angle), math.cos(angle))


def capped(amount, limit):
    """`amount` with its magnitude cut down to `limit`, keeping its sign."""
    return math.copysign(min(limit, abs(amount)), amount)


def is_negligible(amount):
    """True when `amount` is rounding dust rather than ground still to cover."""
    return abs(amount) < FLOAT_TOLERANCE


def yaw_to_quaternion(yaw):
    """A rotation about the vertical axis, in the four number form Gazebo wants."""
    return Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))


class MovementEngine(Node):
    """Runs one movement command at a time, a tick at a time.

    A set `target_yaw` means a turn is running and a set `remaining` means a
    drive is; both None means idle. Starting either command abandons the other.
    The pose below starts at the world origin facing +X, which the scene has to
    actually match, since nothing here ever checks.
    """

    def __init__(self):
        super().__init__('movement_engine')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.target_yaw = None
        self.remaining = None

        self.client = self.create_client(SetEntityPose, SET_POSE_SERVICE)

        self.create_subscription(
            Float64, TURN_TOPIC,
            lambda message: self.turnto(message.data), COMMAND_QUEUE_DEPTH)
        self.create_subscription(
            Float64, MOVE_TOPIC,
            lambda message: self.moveforward(message.data), COMMAND_QUEUE_DEPTH)

        self.create_timer(CONTROL_PERIOD, self.step)

    def turnto(self, angle):
        """Rotate in place until facing `angle`. Absolute heading, not relative."""
        self.target_yaw = angle
        self.remaining = None

    def moveforward(self, distance):
        """Drive `distance` metres along the current heading; negative reverses."""
        self.remaining = distance
        self.target_yaw = None

    def step(self):
        """Advance one tick and mirror the result into Gazebo.

        Does nothing at all while the bridge is down, so the pose can never get
        ahead of an update we failed to send.
        """
        if not self.client.service_is_ready():
            return

        if self.target_yaw is not None:
            self.turn_step()
        elif self.remaining is not None:
            self.drive_step()
        else:
            return

        self.publish_pose()

    def turn_step(self):
        """Rotate one tick's worth toward the target heading."""
        error = shortest_turn(self.target_yaw - self.yaw)
        self.yaw += capped(error, RADIANS_PER_TICK)

        if abs(error) <= RADIANS_PER_TICK:
            self.target_yaw = None

    def drive_step(self):
        """Travel one tick's worth along the current heading."""
        distance = capped(self.remaining, METRES_PER_TICK)
        self.x += math.cos(self.yaw) * distance
        self.y += math.sin(self.yaw) * distance

        left = self.remaining - distance
        self.remaining = None if is_negligible(left) else left

    def publish_pose(self):
        """Ask Gazebo to put the model where we now believe it is.

        Asynchronously, because a blocking call would deadlock: the reply can
        only arrive through the same spin loop that is running this callback.
        """
        request = SetEntityPose.Request()
        request.entity = Entity(name=VEHICLE_NAME, type=Entity.MODEL)
        request.pose = Pose(
            position=Point(x=self.x, y=self.y, z=RIDE_HEIGHT),
            orientation=yaw_to_quaternion(self.yaw))

        self.client.call_async(request)


def main():
    """Start the node and hand this thread to ROS.

    Nothing registered in the constructor runs until spin() does. It dispatches
    the timer and the subscriptions one at a time on this thread, so no two
    callbacks overlap and none of the state needs a lock.
    """
    rclpy.init()
    rclpy.spin(MovementEngine())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
