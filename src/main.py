from drone.drone import Drone
import math
if __name__ == "__main__": 
    try: 
        myDrone = Drone("udpin:127.0.0.1:14550")

        while True: 
            attDict = myDrone.getAtt()
            print(f"roll: {math.degrees(attDict['roll'])}, pitch: {math.degrees(attDict['pitch'])}, yaw: {math.degrees(attDict['yaw'])}")
            

    except ConnectionError as e: 
        print(f"Init problem with {e}") 



