#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Float64


class MovementEngine(Node):
    DT = 0.05
    LINEAR_SPEED = 0.5
    ANGULAR_SPEED = 0.75

    def __init__(self):
        super().__init__('movement_engine')
        self.x = self.y = self.yaw = 0.0
        self.target_yaw = self.remaining = None
        self.client = self.create_client(
            SetEntityPose, '/world/s2_scene/set_pose')
        self.create_subscription(
            Float64, '/turnto', lambda command: self.turnto(command.data), 10)
        self.create_subscription(
            Float64, '/moveforward',
            lambda command: self.moveforward(command.data), 10)
        self.create_timer(self.DT, self.step)

    def turnto(self, angle):
        self.target_yaw = angle
        self.remaining = None

    def moveforward(self, distance):
        self.remaining = distance
        self.target_yaw = None

    def step(self):
        if not self.client.service_is_ready():
            return
        if self.target_yaw is not None:
            error = math.atan2(
                math.sin(self.target_yaw - self.yaw),
                math.cos(self.target_yaw - self.yaw))
            change = math.copysign(
                min(self.ANGULAR_SPEED * self.DT, abs(error)), error)
            self.yaw += change
            if abs(error) <= self.ANGULAR_SPEED * self.DT:
                self.target_yaw = None
        elif self.remaining is not None:
            distance = math.copysign(
                min(self.LINEAR_SPEED * self.DT, abs(self.remaining)),
                self.remaining)
            self.x += math.cos(self.yaw) * distance
            self.y += math.sin(self.yaw) * distance
            self.remaining -= distance
            if abs(self.remaining) < 1e-9:
                self.remaining = None
        else:
            return

        request = SetEntityPose.Request()
        request.entity = Entity(name='vehicle', type=Entity.MODEL)
        request.pose = Pose()
        request.pose.position.x = self.x
        request.pose.position.y = self.y
        request.pose.position.z = 0.125
        request.pose.orientation.z = math.sin(self.yaw / 2)
        request.pose.orientation.w = math.cos(self.yaw / 2)
        self.client.call_async(request)


def main():
    rclpy.init()
    rclpy.spin(MovementEngine())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
