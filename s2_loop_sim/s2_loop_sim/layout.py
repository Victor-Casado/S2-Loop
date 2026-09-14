"""Place the scene's models on the nodes of a grid, one model per node.

Pure arithmetic and randomness, no ROS. The launch file asks for a layout and
turns the result into spawn actions; everything about where things go is
decided here, where it can be exercised without starting Gazebo.
"""
import random
from typing import NamedTuple

from s2_loop_sim.constants import (
    SDF_OBSTACLE_RADIUS,
    SDF_VEHICLE_START,
    VEHICLE_RADIUS,
)


class Circle(NamedTuple):
    """A footprint on the ground plane."""

    x: float
    y: float
    radius: float


class Placement(NamedTuple):
    """One model to create: which kind, which copy of it, where, and how high."""

    model: str
    index: int
    spot: Circle
    z: float


OBSTACLE_MODEL = 'sphere_obstacle'
OBSTACLE_COUNT = 5

WAYPOINT_MODEL = 'waypoint_star'
WAYPOINT_COUNT = 5
WAYPOINT_RADIUS = 0.5
WAYPOINT_Z = 0.06

ARENA_HALF_SIZE = 4.5

# Wider than the two largest footprints put together, with room to spare:
# VEHICLE_RADIUS 0.70 + WAYPOINT_RADIUS 0.50 leaves 0.30 m of clear ground
# between neighbours. So nothing on one node can reach anything on another,
# and no layout needs checking for overlaps.
GRID_SPACING = 1.5

PARKED_VEHICLE = Circle(x=SDF_VEHICLE_START.x, y=SDF_VEHICLE_START.y,
                        radius=VEHICLE_RADIUS)


def free_nodes():
    """Every grid node the arena holds, shuffled, minus the vehicle's own.

    The vehicle is parked on a node rather than beside one, so leaving that
    node out of the pool is all it takes to keep the scene off the bumper.
    """
    reach = int(ARENA_HALF_SIZE / GRID_SPACING)
    nodes = [(step * GRID_SPACING, row * GRID_SPACING)
             for step in range(-reach, reach + 1)
             for row in range(-reach, reach + 1)
             if (step * GRID_SPACING, row * GRID_SPACING)
             != (PARKED_VEHICLE.x, PARKED_VEHICLE.y)]

    random.shuffle(nodes)

    return nodes


def scatter(nodes, model, count, radius, z):
    """`count` copies of `model`, each taking a node off `nodes`."""
    placements = []

    for index in range(1, count + 1):
        x, y = nodes.pop()
        placements.append(Placement(model, index, Circle(x, y, radius), z))

    return placements


def random_layout():
    """A Placement for every model to create, one model per grid node."""
    nodes = free_nodes()

    obstacles = scatter(nodes, OBSTACLE_MODEL, OBSTACLE_COUNT,
                        SDF_OBSTACLE_RADIUS, SDF_OBSTACLE_RADIUS)
    waypoints = scatter(nodes, WAYPOINT_MODEL, WAYPOINT_COUNT,
                        WAYPOINT_RADIUS, WAYPOINT_Z)

    return obstacles + waypoints


def waypoint_coordinates(placements):
    """The waypoint star positions, flattened to [x, y, x, y, ...] metres.

    Flat rather than paired because that is the widest shape a ROS parameter
    can carry: an array has to be all one scalar type. pair_coordinates is the
    other half of the trip.
    """
    coordinates = []
    for placement in placements:
        if placement.model == WAYPOINT_MODEL:
            coordinates += [placement.spot.x, placement.spot.y]

    return coordinates


def pair_coordinates(coordinates):
    """Rebuild [(x, y), ...] from the flattened form waypoint_coordinates makes."""
    if len(coordinates) % 2 != 0:
        raise ValueError('waypoint coordinates must come in x y pairs')

    return list(zip(coordinates[0::2], coordinates[1::2], strict=True))
