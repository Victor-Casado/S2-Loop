SHELL := /bin/bash
.ONESHELL:

PACKAGE := s2_loop_sim
ROS_SETUP := /opt/ros/jazzy/setup.bash

.PHONY: help build launch run clean doctor tunnel

help:
	@printf '%s\n' \
	  'Targets:' \
	  '  make build   - build this ROS workspace' \
	  '  make launch  - build, source install/setup.bash, and launch Gazebo' \
	  '  make run     - same as make launch' \
	  '  make doctor  - show whether ROS can see the package' \
	  '  make clean   - remove build/install/log' \
	  '  make tunnel  - print the browser tunnel command'

build:
	source $(ROS_SETUP)
	colcon build --packages-select $(PACKAGE)

launch: build
	source $(ROS_SETUP)
	source install/setup.bash
	ros2 launch $(PACKAGE) s2_scene.launch.py

run: launch

doctor:
	source $(ROS_SETUP)
	if [ -f install/setup.bash ]; then source install/setup.bash; fi
	ros2 pkg prefix $(PACKAGE)
	ros2 pkg executables $(PACKAGE)

clean:
	rm -rf build install log

tunnel:
	@printf '%s\n' 'ssh -N -L 6080:127.0.0.1:6080 agent@46.62.250.51'
