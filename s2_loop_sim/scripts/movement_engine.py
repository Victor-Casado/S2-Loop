#!/usr/bin/env python3
"""Drive the Gazebo vehicle through waypoint coordinates.

Gazebo is the physics simulator that owns the 3D world. ROS 2 is a separate
message-passing framework, connected to Gazebo by a bridge process. This file
is one ROS 2 node. The SetEntityPose service is a request/response call that
sets the model pose in Gazebo.

This node dead reckons. It keeps its own pose, advances it one small step per
tick, and teleports the model to match. Gazebo is never asked where the vehicle
is, because nothing else moves it. Nothing here is physical, so the vehicle will
drive through an obstacle until collision checks exist.
"""
import math

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.node import Node
from rclpy.parameter import Parameter
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

from s2_loop_sim.constants import (
    CONTROL_PERIOD,
    METRES_PER_TICK,
    RADIANS_PER_TICK,
    RIDE_HEIGHT,
    SDF_VEHICLE_START,
    SET_POSE_SERVICE,
    VEHICLE_NAME,
)
from s2_loop_sim.geometry import (
    angle_to,
    capped,
    distance_to,
    is_negligible,
    shortest_turn,
)
from s2_loop_sim.layout import pair_coordinates


WAYPOINTS_PARAMETER = 'waypoints'

TURNING = 'turning'
DRIVING = 'driving'
DONE = 'done'


def yaw_to_quaternion(yaw):
    """A rotation about the vertical axis, in the four number form Gazebo wants."""
    return Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))


class MovementEngine(Node):
    """Turns toward each waypoint, drives to it, then starts the next one.

    Dead reckoning has to start from wherever the world file actually parked
    the vehicle, and nothing here ever checks, so both sides read the same
    SDF_VEHICLE_START rather than each writing out a zero.
    """

    def __init__(self):
        super().__init__('movement_engine')
        self.waypoints = pair_coordinates(self.declared_waypoints())
        self.next_waypoint = 0

        self.x = SDF_VEHICLE_START.x
        self.y = SDF_VEHICLE_START.y
        self.yaw = SDF_VEHICLE_START.yaw

        self.state = TURNING
        self.target_yaw = None
        self.remaining = None

        self.client = self.create_client(SetEntityPose, SET_POSE_SERVICE)
        self.create_timer(CONTROL_PERIOD, self.step)

    def declared_waypoints(self):
        """The flattened waypoint coordinates the launch file passed in.

        Declared with a type and no default, which is how rclpy says "a double
        array or nothing". Started by hand with no waypoints, the parameter is
        left uninitialised and reading it would raise, so the empty list stands
        in and the node simply finishes with nothing to visit.
        """
        self.declare_parameter(WAYPOINTS_PARAMETER, Parameter.Type.DOUBLE_ARRAY)
        nothing_to_visit = Parameter(
            WAYPOINTS_PARAMETER, Parameter.Type.DOUBLE_ARRAY, [])

        return self.get_parameter_or(WAYPOINTS_PARAMETER, nothing_to_visit).value

    def step(self):
        """Advance one tick and mirror the result into Gazebo.

        Does nothing at all while the bridge is down, so the pose can never get
        ahead of an update we failed to send.
        """
        if not self.client.service_is_ready():
            return

        if self.state == DONE:
            return

        if self.target_yaw is None and self.remaining is None:
            self.start_next_waypoint()
            if self.state == DONE:
                return

        if self.state == TURNING:
            self.turn_step()
        elif self.state == DRIVING:
            self.drive_step()

        self.publish_pose()

    def start_next_waypoint(self):
        """Aim at the next waypoint, or stop when all have been visited."""
        if self.next_waypoint >= len(self.waypoints):
            self.get_logger().info('Visited every waypoint star')
            self.state = DONE
            return

        target_x, target_y = self.waypoints[self.next_waypoint]
        self.target_yaw = angle_to(self.x, self.y, target_x, target_y)
        self.remaining = distance_to(self.x, self.y, target_x, target_y)
        self.state = TURNING

        self.get_logger().info(
            f'Waypoint {self.next_waypoint + 1}: '
            f'turn={self.target_yaw:.2f} rad, drive={self.remaining:.2f} m')

    def turn_step(self):
        """Rotate one tick's worth toward the target heading."""
        error = shortest_turn(self.target_yaw - self.yaw)
        self.yaw += capped(error, RADIANS_PER_TICK)

        if abs(error) <= RADIANS_PER_TICK:
            self.target_yaw = None
            self.state = DRIVING

    def drive_step(self):
        """Travel one tick's worth along the current heading."""
        distance = capped(self.remaining, METRES_PER_TICK)
        self.x += math.cos(self.yaw) * distance
        self.y += math.sin(self.yaw) * distance

        left = self.remaining - distance
        self.remaining = None if is_negligible(left) else left
        if self.remaining is None:
            self.next_waypoint += 1
            self.state = TURNING

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
