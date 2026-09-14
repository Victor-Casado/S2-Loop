"""Scatter the scene's models around the arena without overlapping any of them.

Pure arithmetic and randomness, no ROS. The launch file asks for a layout and
turns the result into spawn actions; everything about where things go is
decided here, where it can be exercised without starting Gazebo.
"""
import math
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
MIN_GAP = 0.3
PARKED_VEHICLE = Circle(x=SDF_VEHICLE_START.x, y=SDF_VEHICLE_START.y,
                        radius=VEHICLE_RADIUS)


def overlaps(spot, other):
    """True when two footprints leave less than MIN_GAP of clear ground between."""
    return (math.hypot(spot.x - other.x, spot.y - other.y)
            <= spot.radius + other.radius + MIN_GAP)


def free_spot(taken, radius):
    """A random footprint in the arena that overlaps nothing in `taken`."""
    while True:
        spot = Circle(random.uniform(-ARENA_HALF_SIZE, ARENA_HALF_SIZE),
                      random.uniform(-ARENA_HALF_SIZE, ARENA_HALF_SIZE),
                      radius)

        if not any(overlaps(spot, other) for other in taken):
            return spot


def scatter(taken, model, count, radius, z):
    """`count` copies of `model`, on ground clear of everything in `taken`.

    Appends what it places to `taken`, so a second call avoids the first.
    """
    placements = []

    for index in range(1, count + 1):
        spot = free_spot(taken, radius)
        taken.append(spot)
        placements.append(Placement(model, index, spot, z))

    return placements


def random_layout():
    """A Placement for every model to create, none of them overlapping."""
    taken = [PARKED_VEHICLE]

    obstacles = scatter(taken, OBSTACLE_MODEL, OBSTACLE_COUNT,
                        SDF_OBSTACLE_RADIUS, SDF_OBSTACLE_RADIUS)
    waypoints = scatter(taken, WAYPOINT_MODEL, WAYPOINT_COUNT,
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
