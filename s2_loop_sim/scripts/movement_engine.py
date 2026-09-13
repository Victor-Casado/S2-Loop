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
# geometry_msgs and std_msgs ship with ROS 2. ros_gz_interfaces comes from the
# ROS/Gazebo bridge package and mirrors Gazebo's own types.
from geometry_msgs.msg import Pose  # a position and an orientation
from rclpy.node import Node  # base class every ROS 2 node inherits from
from ros_gz_interfaces.msg import Entity  # names one thing inside a Gazebo world
from ros_gz_interfaces.srv import SetEntityPose  # "put that thing at this pose"
from std_msgs.msg import Float64  # message carrying one float, in .data


class MovementEngine(Node):
    # One tick moves at most 0.025 m, or turns at most 0.0375 rad.
    DT = 0.05  # seconds per tick, so 20 Hz
    LINEAR_SPEED = 0.5  # m/s
    ANGULAR_SPEED = 0.75  # rad/s

    def __init__(self):
        # Registers this process on the ROS 2 network under this name. Nothing
        # can publish, subscribe or call before it runs.
        super().__init__('movement_engine')

        # Our own belief about the pose. Gazebo is never asked where the vehicle
        # is; this node is the only thing that moves it, so the two agree as long
        # as every update below gets delivered. The zeros assume the model really
        # does start at the origin facing +X.
        self.x = self.y = self.yaw = 0.0
        # One command at a time. target_yaw set means turning, remaining set
        # means driving, both None means idle.
        self.target_yaw = self.remaining = None

        # Our end of a request/response call. Creating a client neither connects
        # nor blocks; it declares the endpoint and type we intend to call. The far
        # end is the bridge process the launch file starts, which forwards the
        # call into Gazebo.
        self.client = self.create_client(
            SetEntityPose, '/world/s2_scene/set_pose')
        # Subscriptions. A callback gets the whole message object, so the lambda
        # unwraps .data to a plain float. The trailing 10 is queue depth: hold up
        # to 10 unprocessed messages, then drop the oldest.
        self.create_subscription(
            Float64, '/turnto', lambda command: self.turnto(command.data), 10)
        self.create_subscription(
            Float64, '/moveforward',
            lambda command: self.moveforward(command.data), 10)
        # Call self.step every DT seconds. Registering a timer does not start it.
        # Callbacks fire only once rclpy.spin() runs, at the bottom of this file.
        self.create_timer(self.DT, self.step)

    def turnto(self, angle):
        """Rotate in place until facing `angle`. Absolute heading, not relative."""
        self.target_yaw = angle
        self.remaining = None  # abandons any drive in progress

    def moveforward(self, distance):
        """Drive `distance` metres along the current heading; negative reverses."""
        self.remaining = distance
        self.target_yaw = None  # abandons any turn in progress

    def step(self):
        """One tick: advance the pose by a step, then push it to Gazebo."""
        # Bridge not up, so do nothing. Advancing the pose without being able to
        # send it would leave our belief ahead of the model forever.
        if not self.client.service_is_ready():
            return
        if self.target_yaw is not None:
            # Shortest way round. Pushing the difference through sin/cos and back
            # through atan2 throws away whole turns and folds the result into
            # [-pi, pi], so 170 deg to -170 deg is a +20 deg turn, not -340.
            error = math.atan2(
                math.sin(self.target_yaw - self.yaw),
                math.cos(self.target_yaw - self.yaw))
            # Cap the step at this tick's budget. On the last tick min() picks the
            # leftover error instead, so the vehicle lands exactly on target.
            # copysign puts back the direction abs() stripped.
            change = math.copysign(
                min(self.ANGULAR_SPEED * self.DT, abs(error)), error)
            self.yaw += change
            if abs(error) <= self.ANGULAR_SPEED * self.DT:
                self.target_yaw = None  # this tick used up the whole error
        elif self.remaining is not None:
            distance = math.copysign(
                min(self.LINEAR_SPEED * self.DT, abs(self.remaining)),
                self.remaining)
            # Project onto the current heading. This is what makes "forward" mean
            # forward rather than "along +X".
            self.x += math.cos(self.yaw) * distance
            self.y += math.sin(self.yaw) * distance
            self.remaining -= distance
            # Repeated float subtraction rarely hits exactly 0.0.
            if abs(self.remaining) < 1e-9:
                self.remaining = None
        else:
            # Idle. Bail before building a request, so a parked vehicle is not
            # re-teleported onto the same spot 20 times a second.
            return

        # The rest fills in the generated request object and sends it.
        request = SetEntityPose.Request()
        # 'vehicle' must match the model name in the world file. MODEL picks the
        # whole model rather than a link or visual inside it.
        request.entity = Entity(name='vehicle', type=Entity.MODEL)
        request.pose = Pose()  # zero position, identity rotation (w defaults to 1)
        request.pose.position.x = self.x
        request.pose.position.y = self.y
        request.pose.position.z = 0.125  # half the 0.25 m box height, so it rests on the ground
        # Orientation is a quaternion. Rotating about Z alone leaves the x and y
        # terms at zero; these two carry the angle. The half-angle is what makes
        # quaternions compose correctly under multiplication.
        request.pose.orientation.z = math.sin(self.yaw / 2)
        request.pose.orientation.w = math.cos(self.yaw / 2)
        # Fire and forget: returns a Future nobody reads. The blocking call()
        # would deadlock, because the reply can only arrive through the same spin
        # loop that is running this callback.
        self.client.call_async(request)


def main():
    rclpy.init()  # sets up the ROS context, consumes any --ros-args
    # spin() is the event loop. It waits on everything the node registered, runs
    # the ready callbacks one at a time on this thread, and blocks until
    # shutdown. Single-threaded, so callbacks never overlap, which is why none of
    # the state above needs a lock.
    rclpy.spin(MovementEngine())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
