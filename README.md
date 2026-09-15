# S2-Loop

A box drives itself around a walled arena in Gazebo, visiting five waypoint
stars and steering around the obstacles on the way.

The layout is shuffled on every launch, so no two runs look alike. The star
the vehicle is heading for turns green. If one gets boxed in by obstacles and
there's no way through, it turns red and the vehicle carries on to the next.

You'll need Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, colcon, and
`ros-jazzy-ros-gz`. The repo root is the colcon workspace, so build from there.

## The Makefile

Each target sources `/opt/ros/jazzy/setup.bash` on its own, so you never have
to remember to.

| Target | What it does |
| --- | --- |
| `make launch` | builds, sources `install/setup.bash`, opens the scene |
| `make build` | `colcon build --packages-select s2_loop_sim` |
| `make run` | another name for `launch` |
| `make doctor` | prints the package prefix and executables, to confirm ROS sees it |
| `make clean` | removes `build/`, `install/`, and `log/` |
| `make tunnel` | prints an SSH port-forward for a remote sim host |

In practice you only type `make launch`, since it rebuilds before it starts.
If you're tunnelling to another machine, name the host in your shell first
with `export TUNNEL_HOST=you@1.2.3.4`.

## Running it on WSL2

This works on WSL with two things in place: software rendering, and an X
server over on the Windows side.

Software rendering comes from `LIBGL_ALWAYS_SOFTWARE=1`, which puts OpenGL on
llvmpipe. Frames come slower that way, but the engine ticks on wall-clock time
rather than render rate, so the vehicle drives exactly the same.

For the X server, run XLaunch from VcXsrv. Choose *Multiple windows*, leave
the display number at **0**, pick *Start no client*, and tick **Disable access
control** so WSL is allowed to connect. Windows will ask about the firewall
somewhere in there; allow it on **private** networks.

Then open a shell and go:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export DISPLAY=$(ip route show default | awk '{print $3}'):0.0   # NAT
# export DISPLAY=127.0.0.1:0.0                                   # mirrored
make launch
```

Both exports have to live in the same shell you run `make` in, since that's
the environment the launch inherits. If you want to check the display on its
own, `xeyes` answers much faster than Gazebo will.

## How it works

`layout.py` shuffles a 9x9 grid of one-metre cells and deals it out: 25
obstacles, 5 stars, and a ring of walls at ±5 m to keep the vehicle inside.
`s2_scene.launch.py` bakes that layout into the world file before handing it
to `gz sim`, brings up the ROS/Gazebo bridge, and starts the engine three
seconds later.

`movement_engine.py` does the driving. It runs A* from whichever cell it's
standing on to the next star, then walks the path a square at a time, turning
to face each cell before driving to it. Only free cells are in the graph, so
any path it finds is already clear of the obstacles. It keeps its own pose,
moves it one tick's worth per step, and teleports the model to match with
`SetEntityPose`, which is why it lands on each cell exactly.

Put the same planner on real hardware and it would send `cmd_vel` to a base,
take its pose from odometry, and build the grid from a live scan.
