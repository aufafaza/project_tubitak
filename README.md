# project_tubitak

A simple mission planner for TEKNOFEST 2026 fixed-wing category.

# Stack

- Python 3
- OpenCV
- ROS Jammy and Gazebo Fortress (for testing)

# How to run

Make sure to install all dependencies in **requirements.txt**, use your own preferred ROS2 and Gazebo versions, should be compatible with any.

## Run the full mission (single command)

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch uav_sim mission.launch.xml
```

Optional flags:

- `use_map:=true` — open MAVProxy map
- `use_console:=true` — open MAVProxy console
- `use_drop_zones:=false` — skip spawning drop zones

## Launch simulation only (no mission node)

```bash
ros2 launch uav_sim uav_sim.launch.xml
```

## Launch your preferred planner, either MAVProxy or Mission Planner

The port is forwarded from the master to udp:127.0.0.1:14550 for your preferred planner and udp:127.0.0.1:14551 for scripts.

# Notes

This is still work in progress, hence some codes are noodly and needs some tuning. Especially the georeferencing algorithms.
