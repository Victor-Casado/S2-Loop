# Build recipe for this package. ROS 2 builds with colcon, which walks the
# workspace and runs one of these per package. Nothing here compiles anything:
# the package is Python and XML, so the whole job is copying files into the
# install tree that other tools then look in.
cmake_minimum_required(VERSION 3.8)
project(s2_loop_sim)

# ament_cmake is ROS 2's CMake layer. It defines ament_package() at the bottom
# and sets up the install paths used below.
find_package(ament_cmake REQUIRED)

# Data files go to share/s2_loop_sim/. This is the directory that
# get_package_share_directory('s2_loop_sim') returns at runtime, which is how
# the launch file finds the world and model files.
install(DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

# Executables go to lib/s2_loop_sim/, which is where `ros2 run` looks. PROGRAMS
# rather than FILES because it keeps the executable bit, and the script is run
# directly via its shebang rather than handed to an interpreter.
install(PROGRAMS scripts/movement_engine.py
  DESTINATION lib/${PROJECT_NAME}
)

# Must come last. Writes the marker files that let colcon and ros2 discover
# this package after it is installed.
ament_package()
