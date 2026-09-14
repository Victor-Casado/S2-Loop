# Build recipe for this package. ROS 2 builds with colcon, which walks the
# workspace and runs one of these per package. Nothing here compiles anything:
# the package is Python and XML, so the whole job is putting files where the
# other tools will look for them.
cmake_minimum_required(VERSION 3.8)
project(s2_loop_sim)

# ament_cmake is ROS 2's CMake layer; it defines ament_package() below and sets
# up the install paths. ament_cmake_python adds the one call that installs an
# importable Python package.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Data files go to share/s2_loop_sim/. This is the directory that
# get_package_share_directory('s2_loop_sim') returns at runtime, which is how
# the launch file finds the world and model files.
install(DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

# Installs the s2_loop_sim/ Python package onto the interpreter's path, so that
# `from s2_loop_sim.constants import ...` resolves from anywhere: the launch
# file under share/, the engine under lib/, or a plain python3 session.
ament_python_install_package(${PROJECT_NAME})

# Executables go to lib/s2_loop_sim/, which is where `ros2 run` looks. PROGRAMS
# rather than FILES because it keeps the executable bit, and the script is run
# directly via its shebang rather than handed to an interpreter.
install(PROGRAMS scripts/movement_engine.py scripts/waypoint_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

# Must come last. Writes the marker files that let colcon and ros2 discover this
# package once it is installed.
ament_package()
