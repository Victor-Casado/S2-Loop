"""Small geometry helpers for the flat XY simulation world."""
import math


def angle_to(robot_x, robot_y, target_x, target_y):
    """Absolute heading, in radians, from the robot to the target point."""
    return math.atan2(target_y - robot_y, target_x - robot_x)


def distance_to(robot_x, robot_y, target_x, target_y):
    """Straight-line distance, in metres, from the robot to the target point."""
    return math.hypot(target_x - robot_x, target_y - robot_y)
