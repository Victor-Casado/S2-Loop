"""Names and tunable numbers shared across the simulation.

Names beginning SDF_ are copies of values written in the model files under
models/. XML cannot import Python, so changing one means changing the other.

Anything only the scene layout cares about lives in layout.py instead.
"""
WORLD_NAME = 's2_scene'
VEHICLE_NAME = 'vehicle'

SET_POSE_SERVICE = f'/world/{WORLD_NAME}/set_pose'

SDF_VEHICLE_BODY_HEIGHT = 0.25
SDF_OBSTACLE_RADIUS = 0.45
RIDE_HEIGHT = SDF_VEHICLE_BODY_HEIGHT / 2

CONTROL_PERIOD = 0.05
LINEAR_SPEED = 0.5
ANGULAR_SPEED = 0.75
METRES_PER_TICK = LINEAR_SPEED * CONTROL_PERIOD
RADIANS_PER_TICK = ANGULAR_SPEED * CONTROL_PERIOD
FLOAT_TOLERANCE = 1e-9

SPAWN_DELAY = 2.0
