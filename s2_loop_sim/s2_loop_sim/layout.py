"""Scatter the scene's models around the arena without overlapping any of them.

Pure arithmetic and randomness, no ROS. The launch file asks for a layout and
turns the result into spawn actions; everything about where things go is
decided here, where it can be exercised without starting Gazebo.
"""
import math
import random
from typing import NamedTuple

from s2_loop_sim.constants import SDF_OBSTACLE_RADIUS


class Circle(NamedTuple):
    """A footprint on the ground plane."""

    x: float
    y: float
    radius: float


class Spawn(NamedTuple):
    """A kind of model to scatter around the arena at launch."""

    model: str
    count: int
    radius: float
    z: float


class Placement(NamedTuple):
    """One model to create: which kind, which copy of it, and where."""

    spawn: Spawn
    index: int
    spot: Circle


WAYPOINT_MODEL = 'waypoint_star'

ARENA_HALF_SIZE = 4.5
MIN_GAP = 0.3
PARKED_VEHICLE = Circle(x=0.0, y=0.0, radius=0.5)

SPAWNS = (
    Spawn(model='sphere_obstacle', count=5,
          radius=SDF_OBSTACLE_RADIUS, z=SDF_OBSTACLE_RADIUS),
    Spawn(model=WAYPOINT_MODEL, count=5, radius=0.5, z=0.06),
)


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


def random_layout():
    """A Placement for every model to create, none of them overlapping."""
    taken = [PARKED_VEHICLE]
    placements = []

    for spawn in SPAWNS:
        for index in range(1, spawn.count + 1):
            spot = free_spot(taken, spawn.radius)
            taken.append(spot)
            placements.append(Placement(spawn, index, spot))

    return placements


def waypoint_arguments(placements):
    """Flatten the waypoint star positions into command-line arguments."""
    arguments = []
    for placement in placements:
        if placement.spawn.model == WAYPOINT_MODEL:
            arguments += [str(placement.spot.x), str(placement.spot.y)]

    return arguments
