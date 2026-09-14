"""Small geometry helpers for the flat XY simulation world.

Nothing here touches ROS or Gazebo. The movement engine is a thin node wrapped
around these functions, which keeps the arithmetic testable on its own.
"""
import math

from s2_loop_sim.constants import FLOAT_TOLERANCE


def angle_to(robot_x, robot_y, target_x, target_y):
    """Absolute heading, in radians, from the robot to the target point."""
    return math.atan2(target_y - robot_y, target_x - robot_x)


def distance_to(robot_x, robot_y, target_x, target_y):
    """Straight-line distance, in metres, from the robot to the target point."""
    return math.hypot(target_x - robot_x, target_y - robot_y)


def shortest_turn(angle):
    """`angle` folded into [-pi, pi], so a turn never goes the long way round."""
    return math.atan2(math.sin(angle), math.cos(angle))


def capped(amount, limit):
    """`amount` with its magnitude cut down to `limit`, keeping its sign."""
    return math.copysign(min(limit, abs(amount)), amount)


def is_negligible(amount):
    """True when `amount` is rounding dust rather than ground still to cover."""
    return abs(amount) < FLOAT_TOLERANCE
