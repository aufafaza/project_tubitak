# project_tubitak

A simple mission planner for TEKNOFEST 2026 fixed-wing category.

# Stack

- Python 3
- OpenCV
- ROS Jammy and Gazebo Fortress (for testing)

# How to run

Make sure to install all dependencies in **requirements.txt**, use your own preferred ROS2 and Gazebo versions, should be compatible with any.

## Launch the ros2 node and simulation

`ros2 launch uav_sim uav_sim.launch.xml`
If needed, use these flags:

- `use_mavros_pl:=true`
- `use_drop_zones:=true`

## Launch your preferred planner, either MAVProxy or Mission Planner

The port is forwarded from the master to udp:127.0.0.1:14550 for your preferred planner and udp:127.0.0.1:14451 for scripts.

## Launch the algorithm scripts

`python3 src/gz_camera.py` or `python3 src/gz_payload.py`
