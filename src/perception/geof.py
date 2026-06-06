import navpy 
import numpy as np

# mostly just translated from 
# 
#  https://doi.org/10.1016/j.isprsjprs.2019.05.009 section 2

def RbodyFromNED(roll, pitch, yaw): 
    return navpy.angle2dcm(yaw, pitch, roll) 

def buildMatrix(fx, fy, cx, cy): 
    return np.array([[fx, 0, cx],
                    [0, fy, cy], 
                    [0, 0, 1]])


def RGimbalFromBody(psi, theta): 
    Ry = np.array([[ np.cos(theta), 0, -np.sin(theta)],
                  [0, 1, 0], 
                  [np.sin(theta), 0, np.cos(theta)]])
    Rz = np.array([[ np.cos(psi), np.sin(psi), 0],
                   [-np.sin(psi), np.cos(psi), 0],
                   [0, 0, 1]])
    return Ry.T @ Rz.T

def RCamFromGimbal(): 
    return np.array([[0, 1, 0], 
                    [-1, 0, 0],
                    [0, 0, 1]])

def buildGNE(R_n_b, R_b_m, R_m_c, r_nc_in_NED): 
    R_c_n = np.linalg.inv(R_n_b @ np.linalg.inv(R_m_c @ R_b_m))
    r1, r2, r3 = R_c_n[:, 0], R_c_n[:, 1], R_c_n[:, 2] 
    t = -R_c_n @ r_nc_in_NED
    G = np.column_stack([r1, r2, r3, t])
    return G 

def georeference_ray(u, v, A, R_c_n, r_nc_in_NED):
    pixel = np.array([u, v, 1.0])
    ray_c = np.linalg.inv(A) @ pixel
    ray_n = R_c_n @ ray_c
    
    if ray_n[2] <= 1e-6:
        return None
        
    lmbda = -r_nc_in_NED[2] / ray_n[2]
    
    if lmbda < 0:
        return None
        
    target_N = r_nc_in_NED[0] + lmbda * ray_n[0]
    target_E = r_nc_in_NED[1] + lmbda * ray_n[1]
    return target_N, target_E

def georeference(u, v, A, G_NE): 
    pixel = np.array([u, v, 1.0]) 
    result = np.linalg.pinv(G_NE[:, :3]) @ (pixel - G_NE[:, 3])
    N = result[0] / result[2]
    E = result[1] / result[2] 
    return N, E
