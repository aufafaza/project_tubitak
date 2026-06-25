import cv2
import rclpy
import numpy as np
import sys
import os
import time
import threading
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import pymap3d as p3d

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from detection.detection import Detect
from drone.drone import Drone
from perception.geof import georeference, build_intrinsic, r_body_ned

class RosCameraSubscriber(Node):
    def __init__(self):
        super().__init__('vtalon_camera_processor')
        
        try:
            self.drone = Drone("udpin:localhost:14551")
        except Exception as e:
            self.get_logger().error(f"Could not connect to drone telemetry: {e}")
            self.drone = None

        self.latest_gps = None
        self.latest_att = None
        
        if self.drone is not None:
            self.telemetry_thread = threading.Thread(target=self._telemetry_updater, daemon=True)
            self.telemetry_thread.start()

        self.A = build_intrinsic(fx=381.36, fy=381.36, cx=320.0, cy=240.0)

        # change for flights outside gazebo 
        self.home_lat = -35.363261
        self.home_lon = 149.165230
        self.home_alt = 584.0

        self.subscription_down_camera = self.create_subscription(
            Image,
            '/down_camera/image',
            self._image_callback,
            qos_profile_sensor_data
        )
        
        self.bridge = CvBridge()
        self.detector = Detect(None, False)

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
            except Exception as e:
                self.get_logger().error(f"failed to update telemetry: {e}")

            time.sleep(0.05)

    def _image_callback(self, msg):
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            red_frame = frame_bgr.copy()
            blue_frame = frame_bgr.copy()

            redMask = self.detector.maskRed(frame_bgr)
            red_processed, red_centroids = self.detector.detect(redMask, red_frame)

            blueMask = self.detector.maskBlue(frame_bgr)
            blue_processed, blue_centroids = self.detector.detect(blueMask, blue_frame)
            
            if (len(red_centroids) > 0 or len(blue_centroids) > 0) and self.latest_gps is not None and self.latest_att is not None:
                gps = self.latest_gps
                att = self.latest_att

                try:
                    n, e, d = p3d.geodetic2ned(
                    gps['lat'], gps['lon'], gps['alt_msl'],
                    self.home_lat, self.home_lon, self.home_alt
                    )
                    r_drone_ned = np.array([n, e, d])
                      
                    for (u, v) in red_centroids:
                        result = georeference(u, v, self.A, att["roll"], att["pitch"], att["yaw"], r_drone_ned)
                        if result is not None: 
                            target_N, target_E = result
                            self.get_logger().info(
                                f"Target Detected at Pixel: ({u}, {v}) -> Local NED (North: {target_N:.2f}m, East: {target_E:.2f}m)"
                            )

                    for (u, v) in blue_centroids:
                        result = georeference(u, v, self.A, att["roll"], att["pitch"], att["yaw"], r_drone_ned)
                        if result is not None: 
                            target_N, target_E = result
                            self.get_logger().info(
                                f"Target Detected at Pixel: ({u}, {v}) -> Local NED (North: {target_N:.2f}m, East: {target_E:.2f}m)"
                            )
                            
                except Exception as math_err:
                    self.get_logger().error(f"Georeferencing math error: {math_err}")
            
            cv2.imshow("Red Target Detection", red_processed)
            cv2.imshow("Blue Target Detection", blue_processed)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                rclpy.shutdown()
                exit(0)
                
        except Exception as e:
            self.get_logger().error(f"Failed to process image frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RosCameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n close window.")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

