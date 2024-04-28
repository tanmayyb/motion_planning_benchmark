#! /bin/bash
colcon build --packages-select benchmark 
source "./install/setup.bash"
ros2 launch benchmark bench.launch.py
