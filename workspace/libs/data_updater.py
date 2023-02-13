import rospy
import numpy as np
from novatel_oem7_msgs.msg import INSPVA
import pymap3d as pm


class DataUpdater:
    def __init__(self, cfg):
        rospy.init_node('semantic-localization')

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

    def get_vehicle_pose(self):
        print(f"\n=========+EAT")
        print(self.latitude, self.longitude, self.altitude, self.base_lla[0], self.base_lla[1], self.base_lla[2])
        tmp = [self.latitude, self.longitude, self.altitude, self.base_lla[0], self.base_lla[1], self.base_lla[2]]
        for item in tmp:
            print(type(item))
        print(f"\n=========DONE\n")
        x, y, z =  pm.geodetic2enu(self.latitude, self.longitude, self.altitude, self.base_lla[0], self.base_lla[1], self.base_lla[2])
        vehicle_pose_WC = np.array([x, y, z, 0, 0, np.deg2rad(self.heading)])
        return vehicle_pose_WC
