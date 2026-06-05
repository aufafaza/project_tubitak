from pymavlink  import mavutil as mu 
from typing import Any 
from enum import Enum
class Drone: 
    class MissionState(Enum): 
        IDLE = 0 
        TAKEOFF = 1 
        CRUISE = 2
        DROP = 3 

    connectionstring = "" 
    connected = False 
    mavcon = None 
    def __init__(self, connectionstring : str): 
        self.connectionstring = connectionstring 
        self.mavcon: Any = None
        self.connected = False 
        self.getHeartbeat()
    
    def getHeartbeat(self):
        print("log: waiting for heartbeat...") 
        try: 
            self.mavcon = mu.mavlink_connection(self.connectionstring, baud=57600)
            heartbeat = self.mavcon.wait_heartbeat(timeout=5) # type: ignore
        except Exception as e : 
            print("failed connection") 
            raise ConnectionError(f"failed to connect to drone at {self.connectionstring} with error code {e}")

        if heartbeat is None: 
            print("failed to get heartbeat")
            return 
        print("log: got heartbeat...")
        self.connected = True
    
    def getGPS(self): 
        msgGlobalPositionInt = self.mavcon.recv_match(type="GLOBAL_POSITION_INT")
        return msgGlobalPositionInt
    
    def getAtt(self) -> dict: 
        msg = self.mavcon.recv_match(type = 'ATTITUDE', blocking = True)

        return { 
                "roll": msg.roll, 
                "pitch": msg.pitch,
                "yaw": msg.yaw
                }

    # notes on command long: 
    # (target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7) 
    def setArm(self): 
        arm_disarm = mu.mavlink.MAV_CMD_COMPONENT_ARM_DISARM 
        self.mavcon.mav.command_long_send(1, 0,
                                      arm_disarm, 
                                      0, 1, 0, 0, 0, 0, 0, 0)
    def preArm(self) -> str: 
        prearm_check = mu.mavlink.MAV_CMD_RUN_PREARM_CHECKS 
        self.mavcon.mav.command_long_send(1, 0, 
                                      prearm_check, 
                                      0, 0, 0, 0, 0, 0, 0, 0) 
        msg = self.mavcon.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        return msg.result



