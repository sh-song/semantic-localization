import numpy as np
import numpy.linalg as npl
from numpy import cos, sin, array

class CoordinateTransformer:


    def __init__(self, cfg):
        self.cfg = cfg
        self.K1 = self.cfg.K_left_color
        self.dist1 = self.cfg.dist_left_color

        #TODO: from cfg, camera extrinsic calibration
        self.pose_camera = np.array([1,1,1, np.deg2rad(90),0 , 0]) 

    def pose_to_4x4_Rt(self, pose):
        
        # Create quaternion from Radian angles
        roll = 0.5*pose[3] 
        pitch = 0.5 * pose[4]
        yaw = 0.5 * pose[5]

        q = array([cos(roll)*cos(pitch)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw),
                    sin(roll)*cos(pitch)*cos(yaw) - cos(roll)*sin(pitch)*sin(yaw),
                    cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*cos(pitch)*sin(yaw),
                    cos(roll)*cos(pitch)*sin(yaw) - sin(roll)*sin(pitch)*cos(yaw)])
        
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

    def test(self):


        tmp_world = np.array([1,1,0,0])
        print('in world', tmp_world)

        world_to_vehicle = self.pose_to_4x4_Rt(0,0,0, 0, 0 ,np.deg2rad(-90))
        tmp_vehicle = world_to_vehicle @ tmp_world
        print('in vehicle', tmp_vehicle)

        vehicle_to_camera = self.pose_to_4x4_Rt(0,0,0, 0, np.deg2rad(-90) ,0)
        tmp_camera = vehicle_to_camera @ tmp_vehicle
        print('in camera', tmp_camera)

        world_to_camera = vehicle_to_camera @ world_to_vehicle

        tmp2_camera = np.array([0, -1, 1, 0])
        print('in camera2', tmp2_camera)


        #camera_to_world
        tmp2_world = npl.inv(vehicle_to_camera @ world_to_vehicle) @ tmp2_camera
        print('in world2', tmp2_world)



    def run(self, elems_world, pose_vehicle):


        # cam_to_vehicle = self.pose_to_4x4_Rt(x=self.pose_camera[0],
        #                                      y=self.pose_camera[1],
        #                                      z=self.pose_camera[2], 
        #                                      roll=self.pose_camera[3],
        #                                      pitch=self.pose_camera[4],
        #                                      yaw=self.pose_camera[5])

        # vehicle_to_world = self.pose_to_4x4_Rt(x=pose_vehicle[0],
        #                                      y=pose_vehicle[1],
        #                                      z=pose_vehicle[2], 
        #                                      roll=pose_vehicle[3],
        #                                      pitch=pose_vehicle[4],
        #                                      yaw=pose_vehicle[5])

        # world_to_cam = npl.inv(vehicle_to_world @ cam_to_vehicle)
        ##############

        world_to_vehicle = self.pose_to_4x4_Rt(-1,0,0, 0, 0 ,np.deg2rad(-90))
        vehicle_to_camera = self.pose_to_4x4_Rt(0,0,0, 0, np.deg2rad(-90) ,0)
        world_to_camera = vehicle_to_camera @ world_to_vehicle


        ########
        cam_to_img = np.hstack((self.K1, np.zeros([3, 1])))

#######
        cam_to_img = np.array([[1000, 0, 500], [0, 1000, 300], [0, 0, 1]]) 
        cam_to_img = np.hstack((cam_to_img, np.zeros([3, 1])))

#####
        print('world')
        print(elems_world[:,0])

        print('vehicle')
        elems_vehicle = world_to_vehicle @ elems_world
        print(elems_vehicle[:, 0])
        elms_img = cam_to_img @ world_to_camera @ elems_world

        print('img')
        print(elms_img[0, :])
        elms_img = elms_img / elms_img[2, :]

        return self.add_distortion_plumb_bob(u_arr=elms_img[0, :],
                                      v_arr=elms_img[1, :],
                                      k1=self.dist1[0],
                                      k2=self.dist1[1],
                                      k3=self.dist1[2],
                                      p1=self.dist1[3],
                                      p2=self.dist1[4])