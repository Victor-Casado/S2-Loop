"""Names and tunable numbers shared across the simulation.

Names beginning SDF_ are copies of values written in the SDF files under
models/ and worlds/. XML cannot import Python, so changing one means changing
the other. Everything on the Python side reads the copy here, so the SDF file
is the only other place any of these numbers appear.

Anything only the scene layout cares about lives in layout.py instead.
"""
import math
from typing import NamedTuple


class Pose2D(NamedTuple):
    """Where something sits on the ground plane, and which way it faces."""

    x: float
    y: float
    yaw: float


WORLD_NAME = 's2_scene'
VEHICLE_NAME = 'vehicle'

SET_POSE_SERVICE = f'/world/{WORLD_NAME}/set_pose'

SDF_VEHICLE_BODY_HEIGHT = 0.25
SDF_VEHICLE_BODY_WIDTH = 0.5
SDF_VEHICLE_NOSE_TIP = 0.65
SDF_OBSTACLE_RADIUS = 0.45

SDF_VEHICLE_START = Pose2D(x=0.0, y=0.0, yaw=0.0)

RIDE_HEIGHT = SDF_VEHICLE_BODY_HEIGHT / 2
VEHICLE_RADIUS = math.hypot(SDF_VEHICLE_NOSE_TIP, SDF_VEHICLE_BODY_WIDTH / 2)

CONTROL_PERIOD = 0.05
LINEAR_SPEED = 0.5
ANGULAR_SPEED = 0.75
METRES_PER_TICK = LINEAR_SPEED * CONTROL_PERIOD
RADIANS_PER_TICK = ANGULAR_SPEED * CONTROL_PERIOD
FLOAT_TOLERANCE = 1e-9

SPAWN_DELAY = 2.0
SPAWN_SETTLE_DELAY = 1.0
MOVEMENT_START_DELAY = SPAWN_DELAY + SPAWN_SETTLE_DELAY
