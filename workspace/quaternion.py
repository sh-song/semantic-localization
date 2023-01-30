def pose_to_4x4_Rt(x, y, z, roll, pitch, yaw):
    # Create quaternion from Euler angles
    roll *=0.5
    pitch *=0.5
    yaw *=0.5

    q = array([cos(roll)*cos(pitch)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw),
                sin(roll)*cos(pitch)*cos(yaw) - cos(roll)*sin(pitch)*sin(yaw),
                cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*cos(pitch)*sin(yaw),
                cos(roll)*cos(pitch)*sin(yaw) - sin(roll)*sin(pitch)*cos(yaw)])
    # q: [qw, qx, qy, qz] 

    # Create rotation matrix from quaternion
    # r_mat = array([[1 - 2*q[2]**2 - 2*q[3]**2, 2*q[1]*q[2] - 2*q[0]*q[3], 2*q[1]*q[3] + 2*q[0]*q[2]],
    #                   [2*q[1]*q[2] + 2*q[0]*q[3], 1 - 2*q[1]**2 - 2*q[3]**2, 2*q[2]*q[3] - 2*q[0]*q[1]],
    #                   [2*q[1]*q[3] - 2*q[0]*q[2], 2*q[2]*q[3] + 2*q[0]*q[1], 1 - 2*q[1]**2 - 2*q[2]**2]])
    
    return array([[1 - 2*q[2]**2 - 2*q[3]**2, 2*q[1]*q[2] - 2*q[0]*q[3], 2*q[1]*q[3] + 2*q[0]*q[2], x],
                    [2*q[1]*q[2] + 2*q[0]*q[3], 1 - 2*q[1]**2 - 2*q[3]**2, 2*q[2]*q[3] - 2*q[0]*q[1], y],
                    [2*q[1]*q[3] - 2*q[0]*q[2], 2*q[2]*q[3] + 2*q[0]*q[1], 1 - 2*q[1]**2 - 2*q[2]**2, z], 
                    [0, 0, 0, 1]], dtype=np.float64)



