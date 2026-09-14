"""Every tunable number and shared name in the simulation.

Names beginning SDF_ are copies of values written in the model files under
models/. XML cannot import Python, so changing one means changing the other.
"""
from typing import NamedTuple


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


WORLD_NAME = 's2_scene'
VEHICLE_NAME = 'vehicle'

SET_POSE_SERVICE = f'/world/{WORLD_NAME}/set_pose'
TURN_TOPIC = '/turnto'
MOVE_TOPIC = '/moveforward'
MOVEMENT_DONE_TOPIC = '/movement_done'
COMMAND_QUEUE_DEPTH = 10

SDF_VEHICLE_BODY_HEIGHT = 0.25
SDF_OBSTACLE_RADIUS = 0.45
RIDE_HEIGHT = SDF_VEHICLE_BODY_HEIGHT / 2

CONTROL_PERIOD = 0.05
LINEAR_SPEED = 0.5
ANGULAR_SPEED = 0.75
METRES_PER_TICK = LINEAR_SPEED * CONTROL_PERIOD
RADIANS_PER_TICK = ANGULAR_SPEED * CONTROL_PERIOD
FLOAT_TOLERANCE = 1e-9

ARENA_HALF_SIZE = 4.5
MIN_GAP = 0.3
SPAWN_DELAY = 2.0
PARKED_VEHICLE = Circle(x=0.0, y=0.0, radius=0.5)

SPAWNS = (
    Spawn(model='sphere_obstacle', count=5,
          radius=SDF_OBSTACLE_RADIUS, z=SDF_OBSTACLE_RADIUS),
    Spawn(model='waypoint_star', count=5, radius=0.5, z=0.06),
)
