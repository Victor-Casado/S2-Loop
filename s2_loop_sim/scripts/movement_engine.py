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
import heapq
import math

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.node import Node
from rclpy.parameter import Parameter
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

from s2_loop_sim.constants import (
    CONTROL_PERIOD,
    GREEN_MODEL,
    GREEN_Z,
    GRID_SPACING,
    METRES_PER_TICK,
    RADIANS_PER_TICK,
    RED_MODEL,
    RED_Z,
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
    nearest_cell,
    shortest_turn,
)
from s2_loop_sim.layout import build_nav_graph, pair_coordinates


WAYPOINTS_PARAMETER = 'waypoints'
FREE_CELLS_PARAMETER = 'free_cells'

SEEKING = 'seeking'
FOLLOWING = 'following'
DONE = 'done'


def rotation(yaw):
    """A rotation about the vertical axis, in the four number form Gazebo wants."""
    return Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))


class MovementEngine(Node):
    """Follow an A* cell path to each waypoint, then start the next one.

    SEEKING plans the next leg and FOLLOWING walks it one cell at a time,
    turning toward each cell then driving to it with the same tick helpers.
    DONE is where it ends up once the list runs out or nothing is left
    reachable.

    Dead reckoning has to start from wherever the world file actually parked
    the vehicle, and nothing here ever checks, so both sides read the same
    SDF_VEHICLE_START rather than each writing out a zero.
    """

    def __init__(self):
        super().__init__('movement_engine')
        self.waypoints = pair_coordinates(self.declared_waypoints())
        self.graph = build_nav_graph(pair_coordinates(self.declared_free_cells()))
        self.next_waypoint = 0
        self.cell_path = []

        self.x = SDF_VEHICLE_START.x
        self.y = SDF_VEHICLE_START.y
        self.yaw = SDF_VEHICLE_START.yaw

        self.state = SEEKING
        self.target_yaw = self.yaw
        self.remaining = 0.0

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

    def declared_free_cells(self):
        """The flattened free grid nodes the launch file passed in."""
        self.declare_parameter(FREE_CELLS_PARAMETER, Parameter.Type.DOUBLE_ARRAY)
        nothing_free = Parameter(
            FREE_CELLS_PARAMETER, Parameter.Type.DOUBLE_ARRAY, [])

        return self.get_parameter_or(FREE_CELLS_PARAMETER, nothing_free).value

    def step(self):
        """Advance one tick and mirror the result into Gazebo.

        Does nothing at all while the bridge is down, so the pose can never get
        ahead of an update we failed to send.
        """
        if not self.client.service_is_ready():
            return

        if self.state == SEEKING:
            self.start_next_waypoint()

        if self.state == DONE:
            return

        if self.state == FOLLOWING:
            error = shortest_turn(self.target_yaw - self.yaw)
            if abs(error) > RADIANS_PER_TICK:
                self.turn_step()
            else:
                self.drive_step()

            if is_negligible(self.remaining):
                self.cell_path.pop(0)
                if self.cell_path:
                    self.aim_at_next_cell()
                else:
                    self.next_waypoint += 1
                    self.state = SEEKING

        self.publish_pose()

    def start_next_waypoint(self):
        """Plan the A* leg to the next waypoint, skipping it when unreachable."""
        while self.next_waypoint < len(self.waypoints):
            target = self.waypoints[self.next_waypoint]
            start = nearest_cell(self.x, self.y, self.graph)
            path = self.astar_path(start, target) if start else None

            if path is None:
                self.get_logger().info(
                    f'Waypoint {self.next_waypoint + 1}: unreachable, skipping')
                self.teleport(RED_MODEL, self.next_waypoint + 1, target, RED_Z)
                self.next_waypoint += 1
                continue

            self.cell_path = path[1:]
            if not self.cell_path:
                self.get_logger().info(
                    f'Waypoint {self.next_waypoint + 1}: already there')
                self.next_waypoint += 1
                continue
            self.aim_at_next_cell()
            self.state = FOLLOWING
            self.get_logger().info(
                f'Waypoint {self.next_waypoint + 1}: '
                f'{len(self.cell_path)} cells via A*')
            self.teleport(GREEN_MODEL, 1, target, GREEN_Z)
            return

        self.get_logger().info('Visited every reachable waypoint star')
        self.state = DONE

    def astar_path(self, start, goal):
        """The shortest cell path from `start` to `goal`, or None when unreachable.

        Every edge costs one and the grid is axis aligned, so the Manhattan
        distance in steps is the heuristic. Either endpoint missing from the
        graph means an obstacle sits on it, and there is nothing to search.
        """
        if start not in self.graph or goal not in self.graph:
            return None

        def steps(cell):
            return ((abs(cell[0] - goal[0]) + abs(cell[1] - goal[1]))
                    / GRID_SPACING)

        open_cells = [(steps(start), 0, start)]
        came_from = {start: None}
        cost = {start: 0}

        while open_cells:
            _, current_cost, current = heapq.heappop(open_cells)

            if current == goal:
                path = []
                while current is not None:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            if current_cost > cost[current]:
                continue

            for neighbour in self.graph[current]:
                next_cost = current_cost + 1
                if neighbour not in cost or next_cost < cost[neighbour]:
                    cost[neighbour] = next_cost
                    came_from[neighbour] = current
                    heapq.heappush(open_cells, (next_cost + steps(neighbour),
                                                next_cost, neighbour))

        return None

    def aim_at_next_cell(self):
        """Face the next cell on the leg and measure the drive to it."""
        target_x, target_y = self.cell_path[0]
        self.target_yaw = angle_to(self.x, self.y, target_x, target_y)
        self.remaining = distance_to(self.x, self.y, target_x, target_y)

    def turn_step(self):
        """Rotate one tick's worth toward the target heading."""
        error = shortest_turn(self.target_yaw - self.yaw)
        self.yaw += capped(error, RADIANS_PER_TICK)

    def drive_step(self):
        """Travel one tick's worth along the current heading."""
        distance = capped(self.remaining, METRES_PER_TICK)
        self.x += math.cos(self.yaw) * distance
        self.y += math.sin(self.yaw) * distance

        self.remaining -= distance

    def publish_pose(self):
        """Ask Gazebo to put the model where we now believe it is.

        Asynchronously, because a blocking call would deadlock: the reply can
        only arrive through the same spin loop that is running this callback.
        """
        self.teleport(VEHICLE_NAME, None, (self.x, self.y), RIDE_HEIGHT,
                      yaw=self.yaw)

    def teleport(self, model, index, position, z, yaw=0.0):
        """Ask Gazebo to put one model somewhere, without tracking it.

        The vehicle passes its own name and no index; the waypoint
        overlays pass theirs, so the launch file's `model_index` naming
        lands on the parked copy. Fire and forget, like publish_pose.
        """
        name = VEHICLE_NAME if index is None else f'{model}_{index}'
        request = SetEntityPose.Request()
        request.entity = Entity(name=name, type=Entity.MODEL)
        request.pose = Pose(
            position=Point(x=position[0], y=position[1], z=z),
            orientation=rotation(yaw))

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
