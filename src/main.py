from drone.drone import Drone
if __name__ == "__main__": 

    try: 
        myDrone = Drone("udpin:127.0.0.1:14550")
        Drone.getGPS

    except ConnectionError as e: 
        print(f"Init problem with {e}") 



