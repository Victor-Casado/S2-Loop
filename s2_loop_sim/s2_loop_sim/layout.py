"""Place the scene's models on the nodes of a grid, one model per node.

Pure arithmetic and randomness, no ROS. The launch file asks for a layout and
turns the result into spawn actions; everything about where things go is
decided here, where it can be exercised without starting Gazebo.
"""
import random
from typing import NamedTuple

from s2_loop_sim.constants import (
    ARENA_HALF_SIZE,
    GRID_SPACING,
    OBSTACLE_COUNT,
    OBSTACLE_MODEL,
    SDF_OBSTACLE_RADIUS,
    SDF_VEHICLE_START,
    VEHICLE_RADIUS,
    WAYPOINT_COUNT,
    WAYPOINT_MODEL,
    WAYPOINT_RADIUS,
    WAYPOINT_Z,
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

    return obstacles + waypoints + arena_walls()


def arena_walls():
    """One wall obstacle per node of the ring just outside the arena.

    The grid is 9 by 9; the ring at 11 by 11 hems the vehicle in, so
    it can never leave the navigable cells. Past the randomised
    placements, so wall indices never collide with theirs.
    """
    side = int(ARENA_HALF_SIZE / GRID_SPACING) + 1
    index = OBSTACLE_COUNT + WAYPOINT_COUNT + 1
    walls = []

    for step in range(-side, side + 1):
        for row in range(-side, side + 1):
            if abs(step) != side and abs(row) != side:
                continue
            x, y = step * GRID_SPACING, row * GRID_SPACING
            walls.append(Placement(OBSTACLE_MODEL, index,
                                   Circle(x, y, SDF_OBSTACLE_RADIUS),
                                   SDF_OBSTACLE_RADIUS))
            index += 1

    return walls


def model_coordinates(placements, model):
    """The positions of every `model` placement, flattened to [x, y, ...] metres.

    Flat rather than paired because that is the widest shape a ROS parameter
    can carry: an array has to be all one scalar type. pair_coordinates is the
    other half of the trip.
    """
    coordinates = []
    for placement in placements:
        if placement.model == model:
            coordinates += [placement.spot.x, placement.spot.y]

    return coordinates


def free_cell_coordinates(placements):
    """Every grid node without an obstacle on it, flattened to [x, y, ...].

    Waypoint stars stay in: the vehicle has to reach them, so they must be
    traversable. Only obstacle nodes come out, and the arena walls are
    outside the grid anyway. The movement engine rebuilds its nav graph
    from this once at startup.
    """
    reach = int(ARENA_HALF_SIZE / GRID_SPACING)
    blocked = {(placement.spot.x, placement.spot.y)
               for placement in placements
               if placement.model == OBSTACLE_MODEL}

    coordinates = []
    for step in range(-reach, reach + 1):
        for row in range(-reach, reach + 1):
            x, y = step * GRID_SPACING, row * GRID_SPACING
            if (x, y) not in blocked:
                coordinates += [x, y]

    return coordinates


def pair_coordinates(coordinates):
    """Rebuild [(x, y), ...] from the flattened form model_coordinates makes."""
    if len(coordinates) % 2 != 0:
        raise ValueError('waypoint coordinates must come in x y pairs')

    return list(zip(coordinates[0::2], coordinates[1::2], strict=True))


def build_nav_graph(cells):
    """An adjacency list over `cells`: {cell: [up to 4 free neighbours]}.

    A neighbour only counts when it is itself in `cells`, so obstacle nodes
    never become keys and never appear in any neighbour list. Anything absent
    from the graph is untraversable, and A* exits early on it.
    """
    cell_set = set(cells)
    graph = {}

    for x, y in cell_set:
        neighbours = [(x + GRID_SPACING, y), (x - GRID_SPACING, y),
                      (x, y + GRID_SPACING), (x, y - GRID_SPACING)]
        graph[(x, y)] = [neighbour for neighbour in neighbours
                         if neighbour in cell_set]

    return graph
