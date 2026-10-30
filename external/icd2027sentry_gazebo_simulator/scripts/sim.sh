#!/bin/zsh

# Start ign simulation
source install/setup.sh

ros2 launch icd2027sentry_gazebo_simulator icd2027sentry_bringup_sim.launch.py
