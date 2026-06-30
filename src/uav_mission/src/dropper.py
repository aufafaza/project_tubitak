from math import sqrt 
import numpy as np 
class Dropper: 
    def __init__(self): 
            self.h = 0 
            self.g = 9.8 
            self.t_fall = 0 
            self.d_lead = 0 
            self.drop_point = 0

    # assume no wind drift yet, assume flight path is perpendicular to the target 
    def compute_drop_point(self, target_ned, drone_ned, vx, vy):
        h = -drone_ned[2]  # D is negative when airborne, so negate for altitude
        speed = np.linalg.norm([vx, vy])

        if speed < 0.1:
            return None

        v_hat = np.array([vx, vy]) / speed

        t_fall = sqrt(2 * h / self.g)
        d_lead = speed * t_fall

        drop_N = target_ned[0] - v_hat[0] * d_lead
        drop_E = target_ned[1] - v_hat[1] * d_lead

        return np.array([drop_N, drop_E])
    

