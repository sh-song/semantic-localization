import numpy as np
import numpy.linalg as npl
from numpy import cos, sin, array

class CoordinateTransformer:


    def __init__(self):
        
        # Transforms
        self.VC_TO_WC = np.zeros([4,4], dtype=np.float64)
        self.WC_TO_VC = np.zeros([4,4], dtype=np.float64)

        self.CC_TO_VC = np.zeros([4,4], dtype=np.float64)
        self.VC_TO_CC = np.zeros([4,4], dtype=np.float64)

        self.CC_TO_WC = np.zeros([4,4], dtype=np.float64)
        self.WC_TO_CC = np.zeros([4,4], dtype=np.float64)

        self.CC_TO_IMG2D = np.zeros([3,4], dtype=np.float64)
        self.IMG_TO_CC = np.zeros([4,3], dtype=np.float64)

        self.IMG3D_TO_PLT =np.zeros([4,4], dtype=np.float64)
        self.PLT_TO_IMG =np.zeros([4,4], dtype=np.float64)
 
        self.IMG3D_TO_CV =np.zeros([4,4], dtype=np.float64)
        self.CV_TO_IMG =np.zeros([4,4], dtype=np.float64)
 
        self.IMG3D_TO_WC =np.zeros([4,4], dtype=np.float64)
        self.WC_TO_IMG =np.zeros([4,4], dtype=np.float64)

        self.WC_TO_CV = np.zeros([4,4], dtype=np.float64)
    def set_transforms_CC_VC(self, camera_pose_VC):
        T_camera_pose_VC = self._pose_to_4x4_Rt(camera_pose_VC)
        self.CC_TO_VC = T_camera_pose_VC
        self.VC_TO_CC = npl.inv(T_camera_pose_VC)

        # print(f"CC_TO_VC:\n{self.CC_TO_VC}\nVC_TO_CC:\n{self.VC_TO_CC}\n")

    def set_transforms_IMG_CC(self, K):
        self.CC_TO_IMG2D = np.hstack((K, np.zeros([3, 1])))
        self.IMG2D_TO_CC = npl.inv(K)

        f_x = K[0, 0]
        f_y = K[1, 1]
        p_x = K[0, 2]
        p_y = K[1, 2]

        tmp = np.array([-p_x / f_x, -p_y / f_y, 1, 0, 0, 0], dtype=np.float64) # z=1 
        self.IMG3D_TO_CC = self._pose_to_4x4_Rt(tmp)

        # print(f"CC_TO_IMG2D:\n{self.CC_TO_IMG2D}\nIMG3D_TO_CC:\n{self.IMG3D_TO_CC}\n")

    def set_transforms_PLT_IMG(self, plt_pose_IMG):
        # plt_pose_IMG = np.array([h//2, w//2, 0, 0, np.deg2rad(180), np.deg2rad(90)]) 
        self.IMG3D_TO_PLT = self._pose_to_4x4_Rt(plt_pose_IMG)
        self.PLT_TO_IMG3D = npl.inv(self.IMG3D_TO_PLT)

        # print(f"IMG3D_TO_PLT:\n{self.IMG3D_TO_PLT}\nPLT_TO_IMG3D:\n{self.PLT_TO_IMG3D}\n")
    def set_transforms_CV_IMG(self, cv_pose_IMG):
        self.IMG3D_TO_CV = self._pose_to_4x4_Rt(cv_pose_IMG)
        self.CV_TO_IMG3D = npl.inv(self.IMG3D_TO_CV)



    def update_transforms_VC_WC(self, vehicle_pose_WC):
        T_vehicle_pose_WC = self._pose_to_4x4_Rt(vehicle_pose_WC)

        self.VC_TO_WC = T_vehicle_pose_WC
        self.WC_TO_VC = npl.inv(T_vehicle_pose_WC)

        # print(f"VC_TO_WC:\n{self.VC_TO_WC}\nWC_TO_VC:\n{self.WC_TO_VC}\n")
   
    def update_transforms_CC_WC(self):

        self.CC_TO_WC = self.VC_TO_WC @ self.CC_TO_VC
        self.WC_TO_CC = self.VC_TO_CC @ self.WC_TO_VC

        # print(f"CC_TO_WC:\n{self.CC_TO_WC}\nWC_TO_CC:\n{self.WC_TO_CC}\n")

    def update_transforms_IMG_WC(self):

        self.IMG3D_TO_WC = self.CC_TO_WC @ self.IMG3D_TO_CC
        self.WC_TO_IMG = self.CC_TO_IMG2D @ self.WC_TO_CC

        # print(f"IMG3D_TO_WC:\n{self.IMG3D_TO_WC}\nWC_TO_IMG:\n{self.WC_TO_IMG}\n")
    def get_all_T_poses_WC(self):

        T_poses_WC_dict = {}
        T_poses_WC_dict['Camera'] = self.CC_TO_WC
        T_poses_WC_dict['Vehicle'] = self.VC_TO_WC
        T_poses_WC_dict['FOV'] = self.VC_TO_WC @ self.CC_TO_VC @ self.IMG3D_TO_CC  

        return T_poses_WC_dict


    def _pose_to_4x4_Rt(self, pose):
        
        # pose = np.array([x, y, z, roll, pitch, yaw])

        # Create quaternion from Radian angles
        half_r = 0.5 * pose[3] 
        half_p = 0.5 * pose[4]
        half_y = 0.5 * pose[5]

        q = array([cos(half_r)*cos(half_p)*cos(half_y) + sin(half_r)*sin(half_p)*sin(half_y),
                    sin(half_r)*cos(half_p)*cos(half_y) - cos(half_r)*sin(half_p)*sin(half_y),
                    cos(half_r)*sin(half_p)*cos(half_y) + sin(half_r)*cos(half_p)*sin(half_y),
                    cos(half_r)*cos(half_p)*sin(half_y) - sin(half_r)*sin(half_p)*cos(half_y)])
        
        return array([[1 - 2*q[2]**2 - 2*q[3]**2, 2*q[1]*q[2] - 2*q[0]*q[3], 2*q[1]*q[3] + 2*q[0]*q[2], pose[0]],
                        [2*q[1]*q[2] + 2*q[0]*q[3], 1 - 2*q[1]**2 - 2*q[3]**2, 2*q[2]*q[3] - 2*q[0]*q[1], pose[1]],
                        [2*q[1]*q[3] - 2*q[0]*q[2], 2*q[2]*q[3] + 2*q[0]*q[1], 1 - 2*q[1]**2 - 2*q[2]**2, pose[2]], 
                        [0, 0, 0, 1]], dtype=np.float64)

    def add_distortion_plumb_bob(self, u_arr, v_arr, k1, k2, k3, p1, p2):

        r_2 = u_arr**2 + v_arr**2
        r_4 = r_2**2
        r_6 = r_4*r_2
        u_arr_d = u_arr + u_arr * (k1*r_2 + k2*r_4 + k3*r_6) + 2*p1*u_arr*v_arr + p2*(r_2 + 2*u_arr**2)
        v_arr_d = v_arr + v_arr * (k1*r_2 + k2*r_4 + k3*r_6) + p1*(r_2 + 2*v_arr**2) + 2*p2*u_arr*v_arr


        # return np.array([u_arr_d, v_arr_d], dtype=np.uint8)
        return np.array([u_arr, v_arr], dtype=np.uint8)


    def tune_Rt(self, cmd, pose):
        if cmd == "WC_TO_VC":
            self.WC_TO_VC = self._pose_to_4x4_Rt(pose) @ self.WC_TO_VC
            self.VC_TO_WC = npl.inv(self.WC_TO_VC)
        else:
            print('\n\n\n[ERROR] Wrong Transform Command!!!')

    def run(self, cmd, pts):

        tr_pts = np.zeros(pts.shape)

        if cmd == "HELP":
            print("Put in command in string")

        elif cmd == "WC_TO_CC":
            tr_pts = self.WC_TO_CC @ pts 

        elif cmd == "CC_TO_IMG2D":
            tr_pts = self.CC_TO_IMG2D @ pts
            tr_pts = tr_pts / tr_pts[2, :]
            tr_pts = np.vstack([tr_pts, np.ones(tr_pts.shape[1])]) # HC
 
        elif cmd == "CC_TO_IMG3D":
            tr_pts = self.CC_TO_IMG2D @ pts
            tr_pts = tr_pts / tr_pts[2, :]
            tr_pts = np.vstack([tr_pts, np.zeros(tr_pts.shape[1])]) 
            tr_pts[[2, 3]] = tr_pts[[3, 2]] #[x, y, 0, 1].T

        elif cmd == "IMG3D_TO_PLT":
            tr_pts = self.IMG3D_TO_PLT @ pts


        elif cmd == "IMG3D_TO_CV":
            tr_pts = self.IMG3D_TO_CV @ pts


        elif cmd == "PLT_TO_IMG3D":
            tr_pts = self.PLT_TO_IMG3D @ pts
            #tr_pts = tr_pts / tr_pts[2, :]
        
        elif cmd == "IMG2D_TO_CC":
            tr_pts = self.IMG2D_TO_CC @ pts

            # tr_pts = tr_pts * tr_pts[2, :]
        
            tr_pts = np.vstack([tr_pts, 
                                np.ones(tr_pts.shape[1])])


        elif cmd == "CC_TO_VC":
            tr_pts = self.CC_TO_VC @ pts
        elif cmd == "CC_TO_WC":
            tr_pts = self.CC_TO_WC @ pts

        elif cmd == "IMG3D_TO_IMG2D":
            tr_pts = np.delete(pts, 2, 0)

        elif cmd == "WC_TO_VC":
            tr_pts = self.WC_TO_VC @ pts

        elif cmd == "IMG3D_TO_WC":
            tr_pts = self.IMG3D_TO_WC @ pts

        elif cmd == "WC_TO_CV":
            tr_pts = self.CC_TO_IMG2D @ self.WC_TO_CC @ pts
            tr_pts = tr_pts / tr_pts[2, :]
            tr_pts = np.vstack([tr_pts, np.zeros(tr_pts.shape[1])]) 
            tr_pts[[2, 3]] = tr_pts[[3, 2]] #[x, y, 0, 1].T
            tr_pts = self.IMG3D_TO_CV @ tr_pts

        else:
            print('\n\n\n[ERROR] Wrong Transform Command!!!')

        # print(f"Transform: {cmd}\n {pts[:, :3]}\nto\n{tr_pts[:, :3]}\n")

        return tr_pts