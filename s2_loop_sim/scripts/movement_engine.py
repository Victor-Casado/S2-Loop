#!/usr/bin/env python3
"""Turn /turnto and /moveforward commands into vehicle motion inside Gazebo.

If you have not used ROS 2 or Gazebo before, four things to know first.

Gazebo is the physics simulator that owns the 3D world. ROS 2 is a separate
message-passing framework that processes use to talk to each other, and to
Gazebo through a bridge process.

A ROS 2 node is one participant on that network, usually one process. This file
is a single node.

Nodes talk two ways. A topic is a named broadcast channel with no reply and no
delivery guarantee, such as /turnto. A service is a request/response call to one
named endpoint, such as /world/s2_scene/set_pose.

Messages and services are declared in .msg and .srv schema files and generated
into classes. Float64, Pose, Entity and SetEntityPose below are generated, not
hand-written.

What this node does: it keeps its own belief about where the vehicle is,
advances that belief one small step 20 times a second, and teleports the Gazebo
model to match. Nothing here is physical. No wheels, no forces. The vehicle will
drive straight through an obstacle until collision checks exist.
"""
import math

import rclpy
from rclpy.node import Node  # base class every ROS 2 node inherits from

# geometry_msgs and std_msgs ship with ROS 2. ros_gz_interfaces comes from the
# ROS/Gazebo bridge package and mirrors Gazebo's own types.
from geometry_msgs.msg import Point, Pose, Quaternion
from ros_gz_interfaces.msg import Entity  # names one thing inside a Gazebo world
from ros_gz_interfaces.srv import SetEntityPose  # "put that thing at this pose"
from std_msgs.msg import Float64  # message carrying one float, in .data

from s2_loop_sim.constants import (
    ANGULAR_SPEED,
    ARRIVAL_EPSILON,
    COMMAND_QUEUE_DEPTH,
    CONTROL_PERIOD,
    LINEAR_SPEED,
    MOVE_TOPIC,
    RIDE_HEIGHT,
    SET_POSE_SERVICE,
    TURN_TOPIC,
    VEHICLE_NAME,
)


class MovementEngine(Node):

    def __init__(self):
        # Registers this process on the ROS 2 network under this name. Nothing
        # can publish, subscribe or call before it runs.
        super().__init__('movement_engine')

        # Our own belief about the pose. Gazebo is never asked where the vehicle
        # is; this node is the only thing that moves it, so the two agree as long
        # as every update gets delivered. The zeros assume the model really does
        # start at the origin facing +X.
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # One command at a time. target_yaw set means turning, remaining set
        # means driving, both None means idle.
        self.target_yaw = None
        self.remaining = None

        # Our end of a request/response call. Creating a client neither connects
        # nor blocks; it declares the endpoint and type we intend to call. The far
        # end is the bridge process the launch file starts, which forwards the
        # call into Gazebo.
        self.client = self.create_client(SetEntityPose, SET_POSE_SERVICE)

        # A callback gets the whole message object, so these lambdas unwrap .data
        # down to a plain float.
        self.create_subscription(
            Float64, TURN_TOPIC,
            lambda message: self.turnto(message.data), COMMAND_QUEUE_DEPTH)
        self.create_subscription(
            Float64, MOVE_TOPIC,
            lambda message: self.moveforward(message.data), COMMAND_QUEUE_DEPTH)

        # Call self.step every CONTROL_PERIOD seconds. Registering a timer does
        # not start it; callbacks fire only once rclpy.spin() runs, at the bottom
        # of this file.
        self.create_timer(CONTROL_PERIOD, self.step)

    def turnto(self, angle):
        """Rotate in place until facing `angle`. Absolute heading, not relative."""
        self.target_yaw = angle
        self.remaining = None  # abandons any drive in progress

    def moveforward(self, distance):
        """Drive `distance` metres along the current heading; negative reverses."""
        self.remaining = distance
        self.target_yaw = None  # abandons any turn in progress

    def step(self):
        """One tick of the control loop."""
        # Bridge not up, so do nothing at all. Advancing the pose without being
        # able to send it would leave our belief ahead of the model forever.
        if not self.client.service_is_ready():
            return

        if self.target_yaw is not None:
            self.turn_step()
        elif self.remaining is not None:
            self.drive_step()
        else:
            # Idle. Bail out before publishing, so a parked vehicle is not
            # re-teleported onto the same spot 20 times a second.
            return

        self.publish_pose()

    def turn_step(self):
        """Rotate one tick's worth toward the target heading."""
        # Shortest way round. Pushing the difference through sin/cos and back
        # through atan2 throws away whole turns and folds the result into
        # [-pi, pi], so 170 deg to -170 deg is a +20 deg turn, not -340.
        error = math.atan2(
            math.sin(self.target_yaw - self.yaw),
            math.cos(self.target_yaw - self.yaw))

        change, arrived = self.step_toward(error, ANGULAR_SPEED)
        self.yaw += change

        if arrived:
            self.target_yaw = None

    def drive_step(self):
        """Travel one tick's worth along the current heading."""
        distance, arrived = self.step_toward(self.remaining, LINEAR_SPEED)

        # Projecting onto the heading is what makes "forward" mean forward
        # rather than "along +X".
        self.x += math.cos(self.yaw) * distance
        self.y += math.sin(self.yaw) * distance

        # `arrived` is the exact test. The epsilon catches the float dust that
        # repeated subtraction leaves behind, which the vehicle would otherwise
        # spend a whole extra tick crawling off.
        remaining = self.remaining - distance

        if arrived or abs(remaining) < ARRIVAL_EPSILON:
            self.remaining = None
        else:
            self.remaining = remaining

    @staticmethod
    def step_toward(error, speed):
        """Close `error` as far as one tick at `speed` allows.

        Returns the signed amount to apply, and whether it closes the gap
        completely. Capping at the budget is what keeps the motion to a constant
        speed; returning the whole error on the final tick is what makes the
        vehicle land exactly on target instead of overshooting.
        """
        budget = speed * CONTROL_PERIOD

        if abs(error) <= budget:
            return error, True

        return math.copysign(budget, error), False

    def publish_pose(self):
        """Ask Gazebo to place the model where we now believe it is."""
        request = SetEntityPose.Request()

        # VEHICLE_NAME must match the model name in the world file. MODEL picks
        # the whole model rather than a link or visual inside it.
        request.entity = Entity(name=VEHICLE_NAME, type=Entity.MODEL)

        # Orientation is a quaternion. Rotating about the Z axis alone leaves the
        # x and y terms at their default zero; these two carry the angle. The
        # half-angle is what makes quaternions compose correctly under
        # multiplication.
        request.pose = Pose(
            position=Point(x=self.x, y=self.y, z=RIDE_HEIGHT),
            orientation=Quaternion(
                z=math.sin(self.yaw / 2), w=math.cos(self.yaw / 2)))

        # Fire and forget: returns a Future nobody reads. The blocking call()
        # would deadlock, because the reply can only arrive through the same spin
        # loop that is running this callback.
        self.client.call_async(request)


def main():
    rclpy.init()  # sets up the ROS context, consumes any --ros-args

    # spin() is the event loop. It waits on everything the node registered, runs
    # the ready callbacks one at a time on this thread, and blocks until
    # shutdown. Being single-threaded, callbacks never overlap, which is why none
    # of the state above needs a lock.
    rclpy.spin(MovementEngine())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
