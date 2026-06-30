import numpy as np
import pymap3d as p3d
from pymavlink import mavutil

APPROACH_DIST = 150.0 

class WaypointPlanner:
    def __init__(self, mavcon, home_lat, home_lon, home_alt):
        self.mavcon = mavcon
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.home_alt = home_alt

    def ned_to_gps(self, north, east, alt_msl):
        lat, lon, _ = p3d.ned2geodetic(
            north, east, 0.0,
            self.home_lat, self.home_lon, self.home_alt
        )
        return lat, lon, alt_msl

    def build_waypoints(self, target_ned, cruise_alt, yaw):
        ap_N = target_ned[0] - APPROACH_DIST * np.cos(yaw)
        ap_E = target_ned[1] - APPROACH_DIST * np.sin(yaw)

        ap_lat, ap_lon, _ = self.ned_to_gps(ap_N, ap_E, cruise_alt)
        tgt_lat, tgt_lon, _ = self.ned_to_gps(target_ned[0], target_ned[1], cruise_alt)

        return [
            (ap_lat, ap_lon, cruise_alt),
            (tgt_lat, tgt_lon, cruise_alt),
        ]

    def _send_item(self, seq, waypoints):
        if seq == 0:
            self.mavcon.mav.mission_item_int_send(
                self.mavcon.target_system,
                self.mavcon.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1, 0, 0, 0, 0,
                int(self.home_lat * 1e7),
                int(self.home_lon * 1e7),
                int(self.home_alt)
            )
        else:
            lat, lon, alt = waypoints[seq - 1]
            self.mavcon.mav.mission_item_int_send(
                self.mavcon.target_system,
                self.mavcon.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1,
                0, 10, 0, 0,
                int(lat * 1e7),
                int(lon * 1e7),
                alt
            )

    def upload_and_start(self, waypoints):
        total = 1 + len(waypoints)  # item 0 = home placeholder

        self.mavcon.mav.mission_clear_all_send(
            self.mavcon.target_system,
            self.mavcon.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )
        ack = self.mavcon.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
        if ack is None:
            print("No ACK after mission_clear_all")
            return False

        self.mavcon.mav.mission_count_send(
            self.mavcon.target_system,
            self.mavcon.target_component,
            total,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )

        sent = 0
        while sent < total:
            req = self.mavcon.recv_match(
                type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'],
                blocking=True, timeout=5
            )
            if req is None:
                print(f"Timeout waiting for MISSION_REQUEST (sent {sent}/{total})")
                return False
            self._send_item(req.seq, waypoints)
            sent += 1

        ack = self.mavcon.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if ack is None or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print(f"Mission write failed: {ack}")
            return False

        mode_id = self.mavcon.mode_mapping().get('AUTO')
        if mode_id is None:
            print("AUTO mode not available")
            return False
        self.mavcon.mav.set_mode_send(
            self.mavcon.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print("Mission written and switching to AUTO")
        self._download_and_print()
        return True

    def _download_and_print(self):
        self.mavcon.mav.mission_request_list_send(
            self.mavcon.target_system,
            self.mavcon.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )
        count_msg = self.mavcon.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
        if count_msg is None:
            print("No MISSION_COUNT received — cannot verify upload")
            return

        total = count_msg.count
        print(f"\n--- Mission on FC ({total} items) ---")
        for seq in range(total):
            self.mavcon.mav.mission_request_int_send(
                self.mavcon.target_system,
                self.mavcon.target_component,
                seq,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            )
            item = self.mavcon.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)
            if item is None:
                print(f"  [{seq}] timeout")
                continue
            lat = item.x / 1e7
            lon = item.y / 1e7
            alt = item.z
            print(f"  [{item.seq}] cmd={item.command}  lat={lat:.7f}  lon={lon:.7f}  alt={alt:.1f}m  frame={item.frame}")

        self.mavcon.mav.mission_ack_send(
            self.mavcon.target_system,
            self.mavcon.target_component,
            mavutil.mavlink.MAV_MISSION_ACCEPTED,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )
        print("-----------------------------------\n")
