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
            heartbeat = self.mavcon.wait_heartbeat(timeout=1) # type: ignore
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
        print(msgGlobalPositionInt) 



