"""Every tunable number and shared name in the simulation, in one place.

The launch file and the movement engine both import from here, so a value that
has to agree between them is written once. The SDF model files cannot import
Python, so anything mirrored from them is marked below; change one and you must
change the other.
"""

# --- Names -------------------------------------------------------------------
# These three are load-bearing. WORLD_NAME must match <world name> in
# worlds/s2_scene.sdf, VEHICLE_NAME must match the <name> the world gives the
# included vehicle model, and Gazebo namespaces its services under the world
# name, which is where the middle of SET_POSE_SERVICE comes from.
WORLD_NAME = 's2_scene'
VEHICLE_NAME = 'vehicle'
SET_POSE_SERVICE = f'/world/{WORLD_NAME}/set_pose'

TURN_TOPIC = '/turnto'
MOVE_TOPIC = '/moveforward'

# --- Vehicle geometry --------------------------------------------------------
# Mirrors the <box><size> of the body link in models/vehicle/vehicle.sdf.
# Gazebo poses a model by its centre, so a box sitting on the ground has its
# origin half a body height up.
VEHICLE_BODY_HEIGHT = 0.25
RIDE_HEIGHT = VEHICLE_BODY_HEIGHT / 2

# --- Movement engine ---------------------------------------------------------
# One tick therefore moves at most 0.025 m, or turns at most 0.0375 rad.
CONTROL_PERIOD = 0.05  # seconds between ticks, so 20 Hz
LINEAR_SPEED = 0.5  # m/s
ANGULAR_SPEED = 0.75  # rad/s

# Queue depth for the command subscriptions: hold this many unprocessed
# messages before dropping the oldest.
COMMAND_QUEUE_DEPTH = 10

# Subtracting a tick's travel from the distance left over and over leaves float
# dust behind, so a drive usually ends a fraction of a nanometre short of zero
# rather than on it. Anything under this counts as arrived.
ARRIVAL_EPSILON = 1e-9

# --- Scene layout ------------------------------------------------------------
# Objects spawn at random inside a square this far from the origin in x and y,
# with at least MIN_GAP of clear ground between any two of them.
ARENA_HALF_SIZE = 4.5
MIN_GAP = 0.3

# The vehicle is already parked at the origin when the layout is generated, so
# its footprint is reserved before anything else is placed. The radius is
# deliberately larger than the vehicle to leave it somewhere to set off from.
VEHICLE_FOOTPRINT = (0.0, 0.0, 0.5)

# What to scatter around it. `radius` is the footprint used for overlap
# rejection, `spawn_z` is the height the model is created at: half a diameter
# for the sphere so it rests on the ground, just above it for the flat star.
OBSTACLE_COUNT = 5
OBSTACLE_RADIUS = 0.45  # mirrors <sphere><radius> in models/sphere_obstacle
WAYPOINT_COUNT = 5
WAYPOINT_RADIUS = 0.5  # the star's points reach 0.5 m from its centre

SPAWNS = (
    # (model directory name, how many, footprint radius, z to spawn at)
    ('sphere_obstacle', OBSTACLE_COUNT, OBSTACLE_RADIUS, OBSTACLE_RADIUS),
    ('waypoint_star', WAYPOINT_COUNT, WAYPOINT_RADIUS, 0.06),
)

# How long to wait after starting Gazebo before injecting the models above.
# There is no readiness signal to wait on, so this is a guess; if models are
# ever missing at startup, raise it.
SPAWN_DELAY = 2.0
