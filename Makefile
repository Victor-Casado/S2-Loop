SHELL := /bin/bash
.ONESHELL:

PACKAGE := s2_loop_sim
ROS_SETUP := /opt/ros/jazzy/setup.bash

# Whichever machine is running the sim. Override per shell rather than editing
# this line, so a host nobody else can reach does not travel with the repo:
#   export TUNNEL_HOST=you@1.2.3.4     or     make tunnel TUNNEL_HOST=you@1.2.3.4
TUNNEL_HOST ?= user@sim-host
TUNNEL_PORT ?= 6080

.PHONY: help build launch run clean doctor tunnel

help:
	@printf '%s\n' \
	  'Targets:' \
	  '  make build   - build this ROS workspace' \
	  '  make launch  - build, source install/setup.bash, and launch Gazebo' \
	  '  make run     - same as make launch' \
	  '  make doctor  - show whether ROS can see the package' \
	  '  make clean   - remove build/install/log' \
	  '  make tunnel  - print the browser tunnel command (set TUNNEL_HOST)'

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
	@printf '%s\n' 'ssh -N -L $(TUNNEL_PORT):127.0.0.1:$(TUNNEL_PORT) $(TUNNEL_HOST)'
