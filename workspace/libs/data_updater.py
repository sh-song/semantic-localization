import rospy
import numpy as np
from novatel_oem7_msgs.msg import INSPVA
import pymap3d as pm


class DataUpdater:
    def __init__(self, cfg):

        self.cfg = cfg
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self._pose_cb)

        self.base_lla = cfg.base_lla

        ## Initial values
        self.latitude = cfg.init_lla[0]
        self.longitude = cfg.init_lla[1]
        self.altitude = cfg.init_lla[2]
        self.heading = 140 #degs

    def _seg_cb(self, msg):
        self.seg_img = msg.image

    def _pose_cb(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude 
        self.altitude=msg.height
        self.heading = msg.azimuth

    def get_vehicle_pose(self, target=None):

        if target is not None:
            lla = self.cfg.test_scene_llas[target]
            
            #### UNDER C
            tmp = (lla[0], lla[1], lla[2], self.base_lla[0], self.base_lla[1], self.base_lla[2])
            ########

            x, y, z = pm.geodetic2enu(*tmp)


            vehicle_pose_WC = np.array([x, y, z, 0, 0, np.deg2rad(151)])

            print(f"\n======Updated Vehicle Pose, z: {z}")

        else: ## target == None
            x, y, z =  pm.geodetic2enu(self.latitude, self.longitude, self.altitude, self.base_lla[0], self.base_lla[1], self.base_lla[2])
            vehicle_pose_WC = np.array([x, y, z, 0, 0, np.deg2rad(self.heading)])


        return vehicle_pose_WC
