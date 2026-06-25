from pymavlink  import mavutil as mu 
from typing import Any 
from enum import Enum
import numpy as np 
import time 
class Drone: 
    class MissionState(Enum): 
        NONE = -1
        IDLE = 0 
        TAKEOFF = 1 
        CRUISE = 2
        DROP = 3 

    connectionstring = "" 
    connected = False 
    mavcon = None 
    state = MissionState.NONE

    def __init__(self, connectionstring : str): 
        self.connectionstring = connectionstring 
        self.mavcon: Any = None
        self.connected = False 
        self.getHeartbeat()
        self.state = self.MissionState.IDLE
    
    def getHeartbeat(self):
        print("log: waiting for heartbeat...") 
        try: 
            self.mavcon = mu.mavlink_connection(self.connectionstring, baud=57600)
            heartbeat = self.mavcon.wait_heartbeat(timeout=5)
        except Exception as e : 
            print("failed connection") 
            raise ConnectionError(f"failed to connect to drone at {self.connectionstring} with error code {e}")

        if heartbeat is None: 
            print("failed to get heartbeat")
            return 
        print("log: got heartbeat...")
        self.connected = True
    
    def getGPS(self) -> dict: 
        msg = self.mavcon.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
        if msg is None: 
            print("timed out while getting gps location") 
            return {} 
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7 

        alt_msl = msg.alt / 1000.0
        alt_relative = msg.relative_alt / 1000.0

        return { 
                "lat": lat, 
                "lon": lon, 
                "alt_msl": alt_msl, 
                "alt_rel": alt_relative, 
                "hdg": msg.hdg / 100.0
                }
    
    def gpsSafeCheck(self) -> bool: 
        msg = self.mavcon.recv_match(type='GPS_RAW_INT', blocking=True, timeout=3)
        if msg is None: 
            return False 

        if msg.fix_type >= 3 and msg.satellites_visible >= 3: 
            return True 
        else: 
            return False 

    def getAtt(self) -> dict: 
        msg = self.mavcon.recv_match(type = 'ATTITUDE', blocking = True, timeout=3)
        return { 
                "roll": msg.roll, 
                "pitch": msg.pitch,
                "yaw": msg.yaw
                }


    # notes on command long: 
    # (target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7) 
    def setArm(self) -> str: 
        if (self.preArm() != mu.mavlink.MAV_RESULT_ACCEPTED):
            return "prarm check failed" 
             
        arm_disarm = mu.mavlink.MAV_CMD_COMPONENT_ARM_DISARM 
        
        self.mavcon.mav.command_long_send(1, 0,
                                      arm_disarm, 
                                      0, 1, 0, 0, 0, 0, 0, 0)

        msg = self.mavcon.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        return msg.result; 

    def preArm(self) -> str: 
        prearm_check = mu.mavlink.MAV_CMD_RUN_PREARM_CHECKS 
        self.mavcon.mav.command_long_send(1, 0, 
                                      prearm_check, 
                                      0, 0, 0, 0, 0, 0, 0, 0) 
        msg = self.mavcon.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if msg is None: 
            print("timed out while getting gps location") 
            return "fail"

        return msg.result
    
    # def armed(self) -> bool: 
    #     isarmed = self.mavcon.recv_match(type='MAV_MODE', blocking=True, timeout=3)

    #     return isarmed
    
    
    def setMode(self, modeName: str, timeout: float = 5.0) -> bool: 
        modeName = modeName.upper().strip()
        available_modes = self.mavcon.mode_mapping() 

        if modeName not in available_modes: 
            return False 
        modeID = available_modes[modeName] 

        self.mavcon.mav.set_mode_send(
                self.mavcon.target_system,
                mu.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                modeID
                )
        # we need to block, based on gcs to AP documentation 
        startTime = time.time() 
        while time.time() - startTime < timeout: 
            ack = self.mavcon.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if ack is not None: 
                if ack.custom_mode == modeID:
                    return True 
            time.sleep(0.2)
        return False 
    
    def setParam(self, paramName: str, paramValue: float,  timeout: float = 5.0) -> bool: 
        paramID = paramName.encode('utf-8')
        # returns NULL if > 16, based on docs
        if len(paramID) > 16: 
            return False 
        
        if isinstance(paramValue, int): 
            paramType = mu.mavlink.MAV_PARAM_TYPE_INT32
        else:
            paramType = mu.mavlink.MAV_PARAM_TYPE_REAL32
        self.mavcon.mav.param_set_send(
            self.mavcon.target_system,
            self.mavcon.target_component,
            paramID,
            float(paramValue), 
            paramType 
        )

        # verification loop 
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.mavcon.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        
            if msg is not None:
                if isinstance(msg.param_id, bytes):
                    returned_param_name = msg.param_id.decode('utf-8').rstrip('\x00')
                else:
                    returned_param_name = msg.param_id.rstrip('\x00')
            
                if returned_param_name == paramName:
                    if abs(msg.param_value - paramValue) < 0.0001:
                        print(f"Success: {paramName} verified and updated to {paramValue}.")
                        return True
                    else:
                        print(f"Warning: Drone rejected value. Current remote value is {msg.param_value}.")
                        return False

        return False 


        
                
    # the code below is fucking wrong lmfao 
    # def takeoff(self, altitude: float) -> bool: 
    #     self.gpsSafeCheck()
    #     gpsData = self.getGPS() 
    #     takeoff_long = gpsData['lon']
    #     takeoff_lat = gpsData['lat']
    #
    #     takeoff = mu.mavlink.MAV_CMD_NAV_TAKEOFF
    #     self.mavcon.mav.command_long_send(1, 0, 
    #                                       takeoff, 
    #                                       0, 0.0, 0.0, 0.0, 0.0, takeoff_lat, takeoff_long, float(altitude)) 
    #     return True 
    #

