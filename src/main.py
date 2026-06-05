from drone.drone import Drone
from pymavlink import mavutil as mu
import math
if __name__ == "__main__": 
    try: 
        myDrone = Drone("udpin:localhost:14550")

        while True: 
            preArmMsg = myDrone.preArm()
            if (preArmMsg != mu.mavlink.MAV_RESULT_ACCEPTED): 
                print(f"prearm failed, do not arm {preArmMsg}") 
            else: 
                print(f"prearm successful with message {preArmMsg} (should be 0)")  


            
    except ConnectionError as e: 
        print(f"Init problem with {e}") 



