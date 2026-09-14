"""Names and tunable numbers shared across the simulation.

Names beginning SDF_ are copies of values written in the SDF files under
models/ and worlds/. XML cannot import Python, so changing one means changing
the other. Everything on the Python side reads the copy here, so the SDF file
is the only other place any of these numbers appear.
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

SDF_VEHICLE_BODY_HEIGHT = 0.15
SDF_VEHICLE_BODY_WIDTH = 0.4
SDF_VEHICLE_NOSE_TIP = 0.45
SDF_OBSTACLE_RADIUS = 0.4

SDF_VEHICLE_START = Pose2D(x=0.0, y=0.0, yaw=0.0)

RIDE_HEIGHT = SDF_VEHICLE_BODY_HEIGHT / 2
VEHICLE_RADIUS = math.hypot(SDF_VEHICLE_NOSE_TIP, SDF_VEHICLE_BODY_WIDTH / 2)

CONTROL_PERIOD = 0.05
LINEAR_SPEED = 0.5
ANGULAR_SPEED = 0.75
METRES_PER_TICK = LINEAR_SPEED * CONTROL_PERIOD
RADIANS_PER_TICK = ANGULAR_SPEED * CONTROL_PERIOD
FLOAT_TOLERANCE = 1e-9

MOVEMENT_START_DELAY = 3.0

ARENA_HALF_SIZE = 4.5

# One metre, which is what Gazebo's own ground grid draws, so every model
# lands on a line the viewport already shows. The widest pair on adjacent
# nodes is the vehicle nose (0.45 m reach) plus an obstacle (0.40 m), which
# still leaves clear ground between them, so no layout needs checking for
# overlaps.
GRID_SPACING = 1.0

OBSTACLE_MODEL = 'sphere_obstacle'
OBSTACLE_COUNT = 25

WAYPOINT_MODEL = 'waypoint_star'
WAYPOINT_COUNT = 5
WAYPOINT_RADIUS = 0.4
WAYPOINT_Z = 0.06

# Overlay stars marking the active (green) and unreachable (red) waypoints.
# Concentric with the yellow star: green 0.41 and red 0.42 outer radii, each
# raised a centimetre so stacked polys never z-fight.
GREEN_MODEL = 'waypoint_star_green'
GREEN_Z = 0.07

RED_MODEL = 'waypoint_star_red'
RED_Z = 0.08
