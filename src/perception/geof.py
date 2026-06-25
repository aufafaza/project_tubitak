import pymap3d as p3d
import numpy as np
from scipy.spatial.transform import Rotation

# mostly just translated from 
# 
#  https://doi.org/10.1016/j.isprsjprs.2019.05.009 section 2
# we assume that there is no gimbal attached to the frame. So the calculation for the gimbal rotation is not used? 
def r_body_ned(roll, pitch, yaw): 
    return Rotation.from_euler('ZYX', [yaw, pitch, roll]).as_matrix()

def build_intrinsic(fx, fy, cx, cy): 
    return np.array([[fx, 0, cx], 
                     [0, fy, cy],
                     [0, 0, 1]])

# generally the steps should be 
# detect objects, find pixel (u, v) in camera pixels
# get UAV pose from the navigation system 
# Calculate the rotation matrix from the NED coordinate to the camera coordinate 
# Use the georeferencing formula to calculate the real NE coordinate 


# georeferencing formula 
# [p_t(N), pt(E), 1]^T = Z.G_NE^(-1).A^-1[u v 1]^T 
# where A is the camera intrinsic matrix 

# we want the object in detection to immediately use the georeferencing formula, so we would just need to create one function. 

# adjust this param 
R_CAM_TO_BODY = np.array([
    [0, -1, 0],
    [1,  0, 0],
    [0,  0, 1]
])

def georeference(u, v, A, roll, pitch, yaw, r_drone_ned) -> tuple | None: 

    R_body_ned = r_body_ned(roll, pitch, yaw).T
    R_cam_ned = R_body_ned @ R_CAM_TO_BODY

    # deproject pixel 
    ray_ned = R_cam_ned @ (np.linalg.inv(A) @ np.array([u, v, 1.0]))

    # [2] is the Z coordinate 
    if ray_ned[2] <= 1e-6: 
        return None 
    lmbda = -r_drone_ned[2] / ray_ned[2]
    if lmbda < 0: 
        return None 
    
    return ( 
        r_drone_ned[0] + lmbda * ray_ned[0], 
        r_drone_ned[1] + lmbda * ray_ned[1], 
    )