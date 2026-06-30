import pymap3d as p3d 

home_lat = -35.363261
home_lon = 149.165230
home_alt = 584.0

target_lat = -35.3621761
target_lon = 149.1650757

e, n, u = p3d.geodetic2enu(target_lat, target_lon, home_alt,
                             home_lat, home_lon, home_alt)
print(f"x={e:.2f}  y={n:.2f}")