
from numpy import cos, sin, array
def calc_quaternion_from_euler(rpy):
    
    # Create quaternion from Radian angles
    half_r = 0.5 * rpy[0] 
    half_p = 0.5 * rpy[1]
    half_y = 0.5 * rpy[2]

    ## return q: w, x, y, z
    return array([cos(half_r)*cos(half_p)*cos(half_y) + sin(half_r)*sin(half_p)*sin(half_y),
                sin(half_r)*cos(half_p)*cos(half_y) - cos(half_r)*sin(half_p)*sin(half_y),
                cos(half_r)*sin(half_p)*cos(half_y) + sin(half_r)*cos(half_p)*sin(half_y),
                cos(half_r)*cos(half_p)*sin(half_y) - sin(half_r)*sin(half_p)*cos(half_y)])
