#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3Stamped, Pose
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from scipy.spatial.transform import Rotation as R

class WaypointGeneratedNode(Node):
    def __init__(self):
        super().__init__('waypoint_generated_node')
        
        # Constants
        self.R_EARTH = 6378137.0
        self.ORIGIN_LAT = None
        self.ORIGIN_LON = None
        self.COS_LAT = None
        self.home_locked = False
        
        # UAV state
        self.uav_north = 0.0
        self.uav_east = 0.0
        self.uav_alt = 0.0
        self.uav_yaw = 0.0
        self.uav_vx = 0.0
        self.uav_vy = 0.0
        self.uav_vz = 0.0
        
        # Flags to only generate waypoints once per color on first detection
        self.red_detected = False
        self.blue_detected = False
        
        # Target coordinates tracking
        self.red_target_lat = None
        self.red_target_lon = None
        self.blue_target_lat = None
        self.blue_target_lon = None
        self.left_dropped = False
        self.right_dropped = False

        # Latest payload poses from Gazebo Model Pose bridge
        self.latest_left_pose = None
        self.latest_right_pose = None

        # QOS profiles
        home_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Subscriptions
        self.create_subscription(NavSatFix, '/drone/home_geo', self.home_callback, qos_profile=home_qos)
        self.create_subscription(PoseStamped, '/drone/local_pose', self.pose_callback, 10)
        self.create_subscription(Vector3Stamped, '/drone/ground_velocity', self.vel_callback, 10)
        self.create_subscription(NavSatFix, '/target/conf_red', self.red_callback, 10)
        self.create_subscription(NavSatFix, '/target/conf_blue', self.blue_callback, 10)
        
        # Subscriptions for payload release topics
        self.create_subscription(Float64, '/mini_talon_vtail/payload/release', self.release_callback, 10)
        self.create_subscription(Float64, '/iris_with_gimbal/payload/release', self.release_callback, 10)

        # Subscriptions for Gazebo payload models poses (via bridge)
        self.create_subscription(Pose, '/model/payload_left/pose', self.left_pose_callback, 10)
        self.create_subscription(Pose, '/model/payload_right/pose', self.right_pose_callback, 10)
 
        # Publishers
        self.red1_pub = self.create_publisher(NavSatFix, '/waypoints/red1', 10)
        self.red2_pub = self.create_publisher(NavSatFix, '/waypoints/red2', 10)
        self.blue1_pub = self.create_publisher(NavSatFix, '/waypoints/blue1', 10)
        self.blue2_pub = self.create_publisher(NavSatFix, '/waypoints/blue2', 10)
        
        self.get_logger().info("Waypoint Generator Node initialized.")

    def home_callback(self, msg):
        if self.home_locked:
            return
        if msg.latitude != 0.0 and msg.longitude != 0.0:
            self.ORIGIN_LAT = msg.latitude
            self.ORIGIN_LON = msg.longitude
            self.COS_LAT = math.cos(math.pi * self.ORIGIN_LAT / 180.0)
            self.home_locked = True
            self.get_logger().info(f"[WP GEN] Home Origin Locked: Lat={self.ORIGIN_LAT:.6f}, Lon={self.ORIGIN_LON:.6f}")

    def pose_callback(self, msg):
        self.uav_east = msg.pose.position.x
        self.uav_north = msg.pose.position.y
        self.uav_alt = msg.pose.position.z
        
        # Get Yaw from Quaternion
        q = msg.pose.orientation
        rot = R.from_quat([q.x, q.y, q.z, q.w])
        yaw, _, _ = rot.as_euler('zyx', degrees=False)
        self.uav_yaw = yaw

    def vel_callback(self, msg):
        self.uav_vx = msg.vector.x  # East (X in ENU)
        self.uav_vy = msg.vector.y  # North (Y in ENU)
        self.uav_vz = msg.vector.z  # Up (Z in ENU)

    def left_pose_callback(self, msg):
        if not (msg.position.x == 0.0 and msg.position.y == 0.0 and msg.position.z == 0.0):
            self.latest_left_pose = msg

    def right_pose_callback(self, msg):
        if not (msg.position.x == 0.0 and msg.position.y == 0.0 and msg.position.z == 0.0):
            self.latest_right_pose = msg

    def release_callback(self, msg):
        val = msg.data
        # Release LEFT payload (> 0.75 / 1800+ PWM)
        if val > 0.75 and not self.left_dropped:
            self.left_dropped = True
            self.handle_drop("left (blue)")
        # Release RIGHT payload (triggers when val < 0.25, but ignores 0.000 / 1000 PWM startup default)
        elif val < 0.25 and abs(val) > 0.02 and not self.right_dropped:
            self.right_dropped = True
            self.handle_drop("right (red)")

    def handle_drop(self, label):
        self.get_logger().info(f"[WP GEN] Detect drop event for payload: {label}")
        
        # Capture current state at drop
        state = {
            'label': label,
            'east': self.uav_east,
            'north': self.uav_north,
            'alt': self.uav_alt,
            'vx': self.uav_vx,
            'vy': self.uav_vy,
            'vz': self.uav_vz,
        }
        
        # Get target location directly from exact SDF coordinates
        if "blue" in label:
            state['target_north'] = 150.0
            state['target_east'] = 60.0
        else:
            state['target_north'] = 120.0
            state['target_east'] = 0.0
                
        # Create a one-shot timer for 2.0s
        timer_ref = []
        def timer_callback():
            if timer_ref:
                timer_ref[0].cancel()
                self.destroy_timer(timer_ref[0])
            self.calculate_drop_position_and_error(state)
            
        timer = self.create_timer(2.0, timer_callback)
        timer_ref.append(timer)

    def calculate_drop_position_and_error(self, state):
        # Physics constants matching telemetry node
        m = 0.5       # kg
        Cd = 0.0      # Drag coefficient (Gazebo payload has no drag plugin)
        A = 0.00785   # m^2
        rho = 1.225   # kg/m^3
        g = 9.80665   # m/s^2
        k = 0.5 * rho * Cd * A / m
        
        sim_x = 0.0
        sim_y = 0.0
        sim_z = state['alt']
        sim_vx = state['vx']
        sim_vy = state['vy']
        sim_vz = state['vz']
        
        dt = 0.01
        t_fall = 0.0
        
        # Integrate forward up to 2 seconds or until hitting the ground (z <= 0)
        while sim_z > 0.0 and t_fall < 2.0:
            v_rel = math.sqrt(sim_vx**2 + sim_vy**2 + sim_vz**2)
            sim_vx += (-k * v_rel * sim_vx) * dt
            sim_vy += (-k * v_rel * sim_vy) * dt
            sim_vz += (-g - k * v_rel * sim_vz) * dt
            
            sim_x += sim_vx * dt
            sim_y += sim_vy * dt
            sim_z += sim_vz * dt
            t_fall += dt
            
        final_east = state['east'] + sim_x
        final_north = state['north'] + sim_y
        
        # Convert final position back to GPS
        final_lat, final_lon = self.enu_to_gps(final_north, final_east)
        
        # Calculate error
        error = math.sqrt((final_east - state['target_east'])**2 + (final_north - state['target_north'])**2)
        
        # Get actual Gazebo pose of model from ROS subscription
        gz_pose_msg = self.latest_left_pose if "blue" in state['label'] else self.latest_right_pose
        
        gz_info = ""
        if gz_pose_msg is not None:
            gz_east = gz_pose_msg.position.x
            gz_north = gz_pose_msg.position.y
            gz_alt = gz_pose_msg.position.z
            gz_lat, gz_lon = self.enu_to_gps(gz_north, gz_east)
            actual_error = math.sqrt((gz_east - state['target_east'])**2 + (gz_north - state['target_north'])**2)
            gz_info = (
                f"Actual Gazebo 2s position: E={gz_east:.2f}m, N={gz_north:.2f}m, Alt={gz_alt:.2f}m\n"
                f"Actual Gazebo 2s GPS: Lat={gz_lat:.6f}, Lon={gz_lon:.6f}\n"
                f"Actual Drop Error (Distance to target): {actual_error:.3f} meters\n"
            )
        else:
            gz_info = "Actual Gazebo position: Unable to query (No pose message received from bridge)\n"

        self.get_logger().info(
            f"\n=======================================================\n"
            f"[DROP RESULT - {state['label'].upper()}]\n"
            f"Drone state at drop: E={state['east']:.2f}m, N={state['north']:.2f}m, Alt={state['alt']:.2f}m, "
            f"Speed={math.sqrt(state['vx']**2 + state['vy']**2):.2f} m/s\n"
            f"Calculated 2s position: E={final_east:.2f}m, N={final_north:.2f}m, Alt={sim_z:.2f}m\n"
            f"Calculated 2s GPS: Lat={final_lat:.6f}, Lon={final_lon:.6f}\n"
            f"Target position: E={state['target_east']:.2f}m, N={state['target_north']:.2f}m\n"
            f"Calculated Drop Error (Distance to target): {error:.3f} meters\n"
            f"-------------------------------------------------------\n"
            f"{gz_info}"
            f"=======================================================\n"
        )

    def enu_to_gps(self, north, east):
        d_lat = north / self.R_EARTH
        d_lon = east / (self.R_EARTH * self.COS_LAT)
        lat = self.ORIGIN_LAT + math.degrees(d_lat)
        lon = self.ORIGIN_LON + math.degrees(d_lon)
        return lat, lon

    def gps_to_enu(self, lat, lon):
        d_lat = math.radians(lat - self.ORIGIN_LAT)
        d_lon = math.radians(lon - self.ORIGIN_LON)
        north = d_lat * self.R_EARTH
        east = d_lon * self.R_EARTH * self.COS_LAT
        return north, east

    def generate_waypoints(self, msg, color):
        if not self.home_locked:
            self.get_logger().warn("[WP GEN] Cannot generate waypoints: Home geolocation is not locked.")
            return False

        # Target coordinate
        t_lat = msg.latitude
        t_lon = msg.longitude
        t_north, t_east = self.gps_to_enu(t_lat, t_lon)

        # Vehicle heading vector: east is X (cos), north is Y (sin)
        # Perpendicular vector to the left of heading (90 deg counter-clockwise)
        perp_east = -math.sin(self.uav_yaw)
        perp_north = math.cos(self.uav_yaw)

        # Waypoint 1 (Direct Target Position)
        wp1_lat = t_lat
        wp1_lon = t_lon

        # Waypoint 2 (200m perpendicular augmented entry waypoint)
        wp2_north = t_north + 200.0 * perp_north
        wp2_east = t_east + 200.0 * perp_east
        wp2_lat, wp2_lon = self.enu_to_gps(wp2_north, wp2_east)

        # Calculate target detection/resolution error relative to ground truth
        actual_east = 60.0 if color == "blue" else 0.0
        actual_north = 150.0 if color == "blue" else 120.0
        detect_error = math.sqrt((t_east - actual_east)**2 + (t_north - actual_north)**2)

        # Package and publish Waypoint 1
        wp1_msg = NavSatFix()
        wp1_msg.header = msg.header
        wp1_msg.latitude = wp1_lat
        wp1_msg.longitude = wp1_lon
        
        # Package and publish Waypoint 2
        wp2_msg = NavSatFix()
        wp2_msg.header = msg.header
        wp2_msg.latitude = wp2_lat
        wp2_msg.longitude = wp2_lon

        if color == "red":
            self.red1_pub.publish(wp1_msg)
            self.red2_pub.publish(wp2_msg)
            self.get_logger().info(f"[WP GEN] Generated RED waypoints: red1={wp1_lat:.6f},{wp1_lon:.6f} | red2={wp2_lat:.6f},{wp2_lon:.6f} | Detection Error: {detect_error:.3f} meters")
        elif color == "blue":
            self.blue1_pub.publish(wp1_msg)
            self.blue2_pub.publish(wp2_msg)
            self.get_logger().info(f"[WP GEN] Generated BLUE waypoints: blue1={wp1_lat:.6f},{wp1_lon:.6f} | blue2={wp2_lat:.6f},{wp2_lon:.6f} | Detection Error: {detect_error:.3f} meters")
        
        return True

    def red_callback(self, msg):
        self.red_target_lat = msg.latitude
        self.red_target_lon = msg.longitude
        if not self.red_detected:
            if self.generate_waypoints(msg, "red"):
                self.red_detected = True

    def blue_callback(self, msg):
        self.blue_target_lat = msg.latitude
        self.blue_target_lon = msg.longitude
        if not self.blue_detected:
            if self.generate_waypoints(msg, "blue"):
                self.blue_detected = True

def main(args=None):
    rclpy.init(args=args)
    node = WaypointGeneratedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
