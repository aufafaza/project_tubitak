from drone.drone import Drone
from pymavlink import mavutil as mu
import math
import numpy as np 
if __name__ == "__main__": 
    try: 
        myDrone = Drone("udpin:localhost:14550")
        
        armMessage = myDrone.setArm()
        print(f"arm successful with message {armMessage} (should be 0)")  

        getGPSMessage = myDrone.getGPS() 
        print(getGPSMessage['lon']) 

        myDrone.setParam("TKOFF_ALT", 40)
        myDrone.setMode("TAKEOFF")
            
    except ConnectionError as e: 
        print(f"Init problem with {e}") 



