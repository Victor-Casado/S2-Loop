"""Small geometry helpers for the flat XY simulation world.

Nothing here touches ROS or Gazebo. The movement engine is a thin node wrapped
around these functions, which keeps the arithmetic testable on its own.
"""
import math

from s2_loop_sim.constants import (
    ARENA_HALF_SIZE,
    FLOAT_TOLERANCE,
    GRID_SPACING,
)


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


def nearest_cell(x, y, graph):
    """The graph key closest to `(x, y)`, rounding to the grid then searching out.

    Dead reckoning drifts, so the pose is rarely exactly on a node. Rounding
    lands on the right one in the usual case; the expanding ring covers the
    rest.
    """
    guess = (round(x / GRID_SPACING) * GRID_SPACING,
             round(y / GRID_SPACING) * GRID_SPACING)
    if guess in graph:
        return guess

    reach = int(ARENA_HALF_SIZE / GRID_SPACING)
    for ring in range(1, 2 * reach + 1):
        for step in range(-ring, ring + 1):
            for row in (-ring, ring):
                for candidate in ((guess[0] + step * GRID_SPACING,
                                   guess[1] + row * GRID_SPACING),
                                  (guess[0] + row * GRID_SPACING,
                                   guess[1] + step * GRID_SPACING)):
                    if candidate in graph:
                        return candidate

    return None
