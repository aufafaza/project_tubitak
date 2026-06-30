import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    pkg = get_package_share_directory("uav_sim")
    config = os.path.join(pkg, "config", "drop_zone.yaml")

    with open(config) as f:
        p = yaml.safe_load(f)["drop_zone"]

    script = os.path.join(pkg, "scripts", "spawn_drop_zones.sh")

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    "bash",
                    script,
                    str(p["center_x"]),
                    str(p["center_y"]),
                    str(p["half_width"]),
                    str(p["half_height"]),
                ],
                output="screen",
            )
        ]
    )
