import rclpy
import numpy as np
import sys
import os
import time
import threading
from rclpy.node import Node
from std_msgs.msg import Empty
import pymap3d as p3d

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from drone.drone import Drone
import payload.dropper

class RosDropTest(Node):
    def __init__(self):
        super().__init__('payload_drop_test')

        try:
            self.drone = Drone("udpin:localhost:14551")
        except Exception as e:
            self.get_logger().error(f"Could not connect with error code {e}")
            self.drone = None

        self.latest_gps = None
        self.latest_att = None
        self.latest_vel = None

        if self.drone is not None:
            self.telemetry_thread = threading.Thread(target=self._telemetry_updater, daemon=True)
            self.telemetry_thread.start()

        self.home_lat = -35.363261
        self.home_lon = 149.165230
        self.home_alt = 584.0

        self.drop_pub = self.create_publisher(Empty, '/payload/drop_left', 10)
        self.target_ned = np.array([164.33, 85.53])  # N=Y_RED, E=X_RED (Gazebo ENU→NED)
        self.dropped = False
        self.timer = self.create_timer(0.1, self._drop_check)

    def _telemetry_updater(self):
        while rclpy.ok():
            if self.drone is None or self.drone.mavcon is None:
                time.sleep(1.0)
                continue
            try:
                while True:
                    msg = self.drone.mavcon.recv_match(blocking=False)
                    if msg is None:
                        break
                    msg_type = msg.get_type()
                    if msg_type == 'GLOBAL_POSITION_INT':
                        self.latest_gps = {
                            "lat": msg.lat / 1e7,
                            "lon": msg.lon / 1e7,
                            "alt_msl": msg.alt / 1000.0,
                            "alt_rel": msg.relative_alt / 1000.0,
                            "hdg": msg.hdg / 100.0
                        }
                    elif msg_type == 'ATTITUDE':
                        self.latest_att = {
                            "roll": msg.roll,
                            "pitch": msg.pitch,
                            "yaw": msg.yaw
                        }
                    elif msg_type == 'LOCAL_POSITION_NED':
                        self.latest_vel = {
                            "vx": msg.vx,
                            "vy": msg.vy,
                            "vz": msg.vz,
                        }
            except Exception as e:
                self.get_logger().error(f"failed to update telemetry: {e}")
            time.sleep(0.05)

    def _drop_check(self):
        if self.dropped:
            return
        if self.latest_gps is None or self.latest_att is None or self.latest_vel is None:
            return

        n, e, d = p3d.geodetic2ned(
            self.latest_gps['lat'], self.latest_gps['lon'], self.latest_gps['alt_msl'],
            self.home_lat, self.home_lon, self.home_alt
        )
        drone_ned = np.array([n, e, d])
        vx = self.latest_vel['vx']
        vy = self.latest_vel['vy']

        drop_point = payload.dropper.Dropper().compute_drop_point(
            self.target_ned, drone_ned, vx, vy
        )
        if drop_point is None:
            return

        dist = np.linalg.norm([n - drop_point[0], e - drop_point[1]])
        self.get_logger().info(f"dist to drop point: {dist:.2f}m")

        if dist < 3.0:
            self.drop_pub.publish(Empty())
            self.dropped = True
            self.get_logger().info("DROPPED")

def main(args=None):
    rclpy.init(args=args)
    node = RosDropTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
