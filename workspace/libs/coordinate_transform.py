import numpy as np
import numpy.linalg as npl
from numpy import cos, sin, array

class CoordinateTransformer:


    def __init__(self, cfg):
        self.cfg = cfg
        self.K1 = self.cfg.K_left_color
        self.dist1 = self.cfg.dist_left_color



        #TODO: from cfg, camera extrinsic calibration
        self.pose_camera = np.array([0.2,0,1, np.deg2rad(-90), 0, 0]) 

    def pose_to_4x4_Rt(self, x, y, z, roll, pitch, yaw):
        # Create quaternion from Euler angles
        roll *=0.5
        pitch *=0.5
        yaw *=0.5

        q = array([cos(roll)*cos(pitch)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw),
                    sin(roll)*cos(pitch)*cos(yaw) - cos(roll)*sin(pitch)*sin(yaw),
                    cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*cos(pitch)*sin(yaw),
                    cos(roll)*cos(pitch)*sin(yaw) - sin(roll)*sin(pitch)*cos(yaw)])
        
        # Create rotation matrix from quaternion
        # r_mat = array([[1 - 2*q[2]**2 - 2*q[3]**2, 2*q[1]*q[2] - 2*q[0]*q[3], 2*q[1]*q[3] + 2*q[0]*q[2]],
        #                   [2*q[1]*q[2] + 2*q[0]*q[3], 1 - 2*q[1]**2 - 2*q[3]**2, 2*q[2]*q[3] - 2*q[0]*q[1]],
        #                   [2*q[1]*q[3] - 2*q[0]*q[2], 2*q[2]*q[3] + 2*q[0]*q[1], 1 - 2*q[1]**2 - 2*q[2]**2]])
        
        return array([[1 - 2*q[2]**2 - 2*q[3]**2, 2*q[1]*q[2] - 2*q[0]*q[3], 2*q[1]*q[3] + 2*q[0]*q[2], x],
                        [2*q[1]*q[2] + 2*q[0]*q[3], 1 - 2*q[1]**2 - 2*q[3]**2, 2*q[2]*q[3] - 2*q[0]*q[1], y],
                        [2*q[1]*q[3] - 2*q[0]*q[2], 2*q[2]*q[3] + 2*q[0]*q[1], 1 - 2*q[1]**2 - 2*q[2]**2, z], 
                        [0, 0, 0, 1]], dtype=np.float64)

    def add_distortion_plumb_bob(self, u_arr, v_arr, k1, k2, k3, p1, p2):

        r_2 = u_arr**2 + v_arr**2
        r_4 = r_2**2
        r_6 = r_4*r_2
        u_arr_d = u_arr + u_arr * (k1*r_2 + k2*r_4 + k3*r_6) + 2*p1*u_arr*v_arr + p2*(r_2 + 2*u_arr**2)
        v_arr_d = v_arr + v_arr * (k1*r_2 + k2*r_4 + k3*r_6) + p1*(r_2 + 2*v_arr**2) + 2*p2*u_arr*v_arr


        # return np.array([u_arr_d, v_arr_d], dtype=np.uint8)
        return np.array([u_arr, v_arr], dtype=np.uint8)

    def run(self, elems_world, pose_vehicle):


        cam_to_vehicle = self.pose_to_4x4_Rt(x=self.pose_camera[0],
                                             y=self.pose_camera[1],
                                             z=self.pose_camera[2], 
                                             roll=self.pose_camera[3],
                                             pitch=self.pose_camera[4],
                                             yaw=self.pose_camera[5])

        vehicle_to_world = self.pose_to_4x4_Rt(x=pose_vehicle[0],
                                             y=pose_vehicle[1],
                                             z=pose_vehicle[2], 
                                             roll=pose_vehicle[3],
                                             pitch=pose_vehicle[4],
                                             yaw=pose_vehicle[5])

        cam_to_world = cam_to_vehicle @ vehicle_to_world
        world_to_cam = npl.inv(cam_to_world)
        cam_to_img = np.hstack((self.K1, np.zeros([3, 1])))

        print('world')
        print(elems_world)
        elms_img = cam_to_img @ world_to_cam @ elems_world

        print('img')
        print(elms_img)
        elms_img = elms_img / elms_img[2, :]

        return self.add_distortion_plumb_bob(u_arr=elms_img[0, :],
                                      v_arr=elms_img[1, :],
                                      k1=self.dist1[0],
                                      k2=self.dist1[1],
                                      k3=self.dist1[2],
                                      p1=self.dist1[3],
                                      p2=self.dist1[4])