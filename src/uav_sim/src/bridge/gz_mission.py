import rclpy
import numpy as np
import time
import threading
import os
from rclpy.node import Node
from std_msgs.msg import Empty
import pymap3d as p3d
import sys
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.utilities import ok
import cv2

_src = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.append(os.path.join(_src, 'uav_mission', 'src'))
sys.path.append(os.path.join(_src, 'uav_mission'))
sys.path.append(os.path.join(_src, 'uav_vision', 'src'))
sys.path.append(os.path.join(_src, 'uav_perception', 'src'))

# defined modules
from drone import Drone
import perception.geof as georeference
import dropper
import waypoint as wp
from vision.detection import Detect


#static variables
fx = 381.36
fy = 381.36
cx = 320.0
cy = 240.0
home_lat = -35.363261
home_lon = 149.165230
home_alt = 584.0

DROP_THRESHOLD_M = 10.0

class RosTest(Node):
    def __init__(self):
        super().__init__('mission_tester')

        self.drone = None
        try:
            self.drone = Drone("udpin:localhost:14551")
        except Exception as e:
            self.get_logger().error(f"error connecting with core {e}")

        self.gps = None
        self.att = None
        self.vel = None

        # camera stuff
        self.red_detections = []
        self.blue_detections = []
        self.required_hits = 5
        self.bridge = CvBridge()
        
        self.confirmed_targets = {}
        self.detector = Detect(None, False)
        mavcon = self.drone.mavcon if self.drone is not None else None
        self.wp_planner = wp.WaypointPlanner(mavcon, home_lat, home_lon, home_alt)
        self.mission_uploaded = False
        self._uploading = False
        if self.drone is not None:
            self.telemetry_thread = threading.Thread(target=self._telemetry_updater, daemon=True)
            self.telemetry_thread.start()
        # georeferencing setup
        self.A = georeference.build_intrinsic(fx=fx, fy=fy, cx=cx, cy=cy)
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.home_alt = home_alt
        self.dropped_left = False
        self.dropped_right = False  
        self.timer = self.create_timer(0.1, self._drop_check)

        # sub and pub
        self.subscription_down_camera = self.create_subscription(
            Image,
            '/down_camera/image',
            self._image_callback,
            qos_profile_sensor_data
        )
        self.publisher_payload_left = self.create_publisher(
            Empty,
            '/payload/drop_left',
            10
        )
        self.publisher_payload_right = self.create_publisher(
            Empty,
            '/payload/drop_right',
            10
        )

    def _telemetry_updater(self):
        while ok():
            if self.drone is None or self.drone.mavcon is None:
                time.sleep(1.0)
                continue

            if self._uploading:
                time.sleep(0.05)
                continue

            try:
                msg = self.drone.mavcon.recv_match(blocking=False)
                if msg is None:
                    time.sleep(0.01)
                    continue
                msg_type = msg.get_type()
                if msg_type == 'GLOBAL_POSITION_INT':
                    self.gps = {
                        "lat": msg.lat / 1e7,
                        "lon": msg.lon / 1e7,
                        "alt_msl": msg.alt / 1000.0,
                        "alt_rel": msg.relative_alt / 1000.0,
                        "hdg": msg.hdg / 100.0
                    }
                elif msg_type == 'ATTITUDE':
                    self.att = {
                        "roll": msg.roll,
                        "pitch": msg.pitch,
                        "yaw": msg.yaw
                    }
                elif msg_type == 'LOCAL_POSITION_NED':
                    self.vel = {
                        "vx": msg.vx,
                        "vy": msg.vy,
                        "vz": msg.vz,
                    }
                elif msg_type == 'MISSION_ITEM_REACHED':
                    seq = msg.seq
                    self.get_logger().info(f"waypoint {seq} reached")
                    # seq 2 = red target, seq 4 = blue target
                    if seq == 2 and not self.dropped_left:
                        self.drop_left()
                    elif seq == 4 and not self.dropped_right:
                        self.drop_right()

            except Exception as e:
                self.get_logger().error(f"error in updating telemetry: {e}")

    def _drop_check(self):
        if self.dropped_left and self.dropped_right:
            return
        if self.gps is None or self.vel is None:
            return

        color = 'red' if not self.dropped_left else 'blue'
        if color not in self.confirmed_targets:
            return

        target = self.confirmed_targets[color]

        n, e, d = p3d.geodetic2ned(
            self.gps['lat'], self.gps['lon'], self.gps['alt_msl'],
            self.home_lat, self.home_lon, self.home_alt
        )
        drone_ned = np.array([n, e, d])
        vx = self.vel['vx']
        vy = self.vel['vy']

        drop_point = dropper.Dropper().compute_drop_point(
            target, drone_ned, vx, vy
        )

        if drop_point is None:
            return

        dist = np.linalg.norm([n - drop_point[0], e - drop_point[1]])

        if dist > DROP_THRESHOLD_M:
            return

        if not self.dropped_left:
            self.drop_left()
        else:
            self.drop_right()

    def _image_callback(self, msg):
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            red_frame = frame_bgr.copy()
            blue_frame = frame_bgr.copy()

            red_cv_frame, red_centroids = self.detector.detect(
                self.detector.maskRed(frame_bgr), red_frame
            )
            blue_cv_frame, blue_centroids = self.detector.detect(
                self.detector.maskBlue(frame_bgr), blue_frame
            )
            if self.gps is not None and self.att is not None:
                if abs(self.att["roll"]) < np.radians(15) and abs(self.att["pitch"]) < np.radians(15):
                    try:
                        n, e, d = p3d.geodetic2ned(
                            self.gps['lat'], self.gps['lon'], self.gps['alt_msl'], self.home_lat, self.home_lon, self.home_alt
                        )
                        r_drone = np.array([n, e, d])
                        self._accumulate('red', red_centroids, r_drone)
                        self._accumulate('blue', blue_centroids, r_drone)
                    except Exception as e:
                        self.get_logger().error(f"georeferencing error {e}")
            cv2.imshow("red frame", red_cv_frame)
            cv2.imshow("blue frame", blue_cv_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): 
                cv2.destroyAllWindows() 
                rclpy.shutdown() 
                exit(0)

        except Exception as e:
            self.get_logger().error(f"error in getting frame: {e}")
        

    def _accumulate(self, color: str, centroids, r_drone):
        if color in self.confirmed_targets:
            return
        store = self.red_detections if color == 'red' else self.blue_detections

        for (u, v) in centroids:
            if self.gps is not None and self.att is not None:
                result = georeference.georeference(
                    u, v, self.A,
                    self.att["roll"], self.att["pitch"], self.att["yaw"],
                    r_drone,
                )
                self.get_logger().info(f"detected at {result}")

                if result is not None:
                    store.append(result)

        if len(store) >= self.required_hits:
            avg_n = sum(r[0] for r in store) / self.required_hits
            avg_e = sum(r[1] for r in store) / self.required_hits
            self.confirmed_targets[color] = np.array([avg_n, avg_e])
            store.clear()
            self._try_upload_mission()

    def _try_upload_mission(self):
        if 'red' not in self.confirmed_targets or 'blue' not in self.confirmed_targets:
            return
        if self.mission_uploaded:
            return
        if self.wp_planner is None or self.att is None or self.gps is None:
            return

        self.mission_uploaded = True

        red_wps = self.wp_planner.build_waypoints(
            target_ned=self.confirmed_targets['red'], cruise_alt=self.gps['alt_rel'], yaw=self.att['yaw']
        )
        blue_wps = self.wp_planner.build_waypoints(
            self.confirmed_targets['blue'], self.gps['alt_rel'], self.att['yaw']
        )
        waypoints = red_wps + blue_wps

        def _upload():
            self._uploading = True
            try:
                self.wp_planner.upload_and_start(waypoints)
            except Exception as ex:
                self.get_logger().error(f"mission upload error: {ex}")
                self.mission_uploaded = False  # allow retry
            finally:
                self._uploading = False

        threading.Thread(target=_upload, daemon=True).start()


    def drop_left(self):
        self.publisher_payload_left.publish(Empty())
        self.dropped_left = True
        self.get_logger().info("dropped left payload")

    def drop_right(self):
        self.publisher_payload_right.publish(Empty())
        self.dropped_right = True
        self.get_logger().info("dropped right payload")


def main(args=None):
    rclpy.init(args=args)
    node = RosTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__=="__main__":
    main()
