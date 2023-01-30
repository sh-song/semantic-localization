#!/usr/bin/python3
from config import Config
from libs.data_loader import DataLoader
from libs.output_saver import OutputSaver
from libs.map_element_extraction import MapElementExtractor
from libs.coordinate_transform import CoordinateTransformer
from libs.visualizer import Visualizer
import numpy as np
import numpy.linalg as npl
import cv2
class SemanticLocalization:

    # pose = np.array[x, y, z, roll, pitch, yaw]

    def __init__(self, cfg):
        self.cfg = cfg

        self.loader = DataLoader(cfg)
        self.saver = OutputSaver(cfg)
        self.extractor = MapElementExtractor(cfg)
        self.transformer = CoordinateTransformer(cfg)
        self.visualizer = Visualizer()
    def run(self):
        # Set world
        world = self.visualizer.create_3d_world(mapsize=10, res=40, vis=False)
        world_display = self.visualizer.get_world_display(world)

        world_origin_WC = np.array([0, 0, 0, 0, 0, 0])
        T_world_origin_WC = self.transformer.pose_to_4x4_Rt(world_origin_WC)

        world_display.add_coordinate_marker_from_T(T_world_origin_WC)
        world_display.add_text_on_point('[World]', *T_world_origin_WC[0:3, 3])

        # Load data
        left_imgs = self.loader.load_images(self.cfg.left_images)
        img = left_imgs[0]

        # Set vehicle pose
        vehicle_pose_WC = np.array([5, 0, 0, 0, 0, np.deg2rad(90)])
        world_display.add_ego(vehicle_pose_WC[:4].reshape(4,1))

        T_vehicle_pose_WC = self.transformer.pose_to_4x4_Rt(vehicle_pose_WC)
        world_display.add_coordinate_marker_from_T(T_vehicle_pose_WC)
        world_display.add_text_on_point('[Vehicle]', *T_vehicle_pose_WC[0:3, 3])

        VC_to_WC = T_vehicle_pose_WC
        WC_to_VC = npl.inv(VC_to_WC)

        # Set camera pose
        camera_pose_VC = np.array([1, 0, 1.5, 0, np.deg2rad(100), 0]) 
        T_camera_pose_VC = self.transformer.pose_to_4x4_Rt(camera_pose_VC)
        T_camera_pose_WC = VC_to_WC @ T_camera_pose_VC
        world_display.add_coordinate_marker_from_T(T_camera_pose_WC)
        world_display.add_text_on_point('[Camera]', *T_camera_pose_WC[0:3, 3])

        CC_to_VC = T_camera_pose_VC
        VC_to_CC = npl.inv(CC_to_VC)

        CC_to_WC = VC_to_WC @ CC_to_VC
        WC_to_CC = npl.inv(CC_to_WC)

        print('type', img.shape)
        # elems_world = self.extractor.run()

        # Extract elements
        left_lane_WC = self.extractor.generate_straight_lane(4)
        right_lane_WC = self.extractor.generate_straight_lane(6)

        world_display.add_points(left_lane_WC)
        world_display.add_points(right_lane_WC)

        # Transform elements
        elems_world_WC = np.hstack([left_lane_WC, right_lane_WC])
        elems_world_CC = VC_to_CC @ WC_to_VC @ elems_world_WC #TODO:
        #replace with WC_to_CC and comment inverses

        world_display.show()

        ## Image Plane
        CC_to_Img = cfg.K_left_color
        CC_to_Img = np.hstack((CC_to_Img, np.zeros([3, 1])))

        elems_world_Img = CC_to_Img @ elems_world_CC
        elems_world_Plt = self.transformer.Img_to_Plt(elems_world_Img)

        grid_width, grid_height = self.visualizer.get_meshgrid_from_image(img)

        image_plane_display = self.visualizer.get_image_plane_display()
        image_plane_display.add_meshgrid(grid_width, grid_height)
        # image_plane_display.add_points(elems_world_Img)
        image_plane_display.add_points(elems_world_Plt)
        image_plane_display.show()

        # img = self.visualizer.draw_multiple_dots(img, elems_world_Plt)
        # cv2.imshow('yes', img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
    
if __name__ == "__main__":

    #Initialize
    cfg = Config()
    stereo = SemanticLocalization(cfg)

    #Run
    stereo.run()