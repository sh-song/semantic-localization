#!/usr/bin/python3
from config import Config
from libs.data_loader import DataLoader
from libs.output_saver import OutputSaver
from libs.map_element_extraction import MapElementExtractor
from libs.coordinate_transform import CoordinateTransformer
from libs.visualizer import Visualizer
from libs.data_updater import DataUpdater
import numpy as np
import numpy.linalg as npl
import cv2


    
class SemanticLocalization:

    # pose = np.array[x, y, z, roll, pitch, yaw]

    def __init__(self, cfg):

        ## Load data from cfg
        self.camera_pose_VC = cfg.camera_pose_VC

        self.loader = DataLoader(cfg)
        self.saver = OutputSaver(cfg)
        self.extractor = MapElementExtractor(cfg)
        self.transformer = CoordinateTransformer()
        self.visualizer = Visualizer()
        self.updater = DataUpdater(cfg)

        self.IMG_H = cfg.IMG_H
        self.IMG_W = cfg.IMG_W

        ## Init shared data
        self.seg_img = None
        # self.vehicle_pose_WC = np.array([5, 3, 0, 0, 0, np.deg2rad(90)])
        self.vehicle_pose_WC = cfg.initial_vehicle_pose_WC

        ## Set initial transforms
        self.transformer.set_transforms_CC_VC(cfg.camera_pose_VC)
        self.transformer.set_transforms_IMG_CC(K=cfg.K_left_color)
        self.transformer.set_transforms_PLT_IMG(cfg.plt_pose_IMG)

        ## Set map
        # global_map = self.loader.run("GLOBAL MAP")
        # self.extractor.set_global_map(global_map)
        

        ## Set visualization 3D space
        #self.visualizer.set_2D_plane()




    def run(self):
        
        ## Update vehicle pose
        vehicle_pose_WC = self.updater.get_vehicle_pose()

        self.transformer.update_transforms_VC_WC(vehicle_pose_WC)
        self.transformer.update_transforms_CC_WC()
        self.transformer.update_transforms_IMG_WC()

        # elems_WC_dict = self.extractor.run(["VIRTUAL ROAD LANE"], vehicle_pose_WC)
        elems_WC_dict = self.extractor.run(["ROAD LANE"], vehicle_pose_WC)
        elems_WC = np.hstack(list(elems_WC_dict.values()))

        elems_CC = self.transformer.run("WC_TO_CC", elems_WC)
        elems_IMG3D = self.transformer.run("CC_TO_IMG3D", elems_CC)
        elems_PLT = self.transformer.run("IMG3D_TO_PLT", elems_IMG3D)

        ## Visualize 3D
        self.visualizer.reset_display_3D()
        T_poses_WC_dict = self.transformer.get_all_T_poses_WC()
        self.visualizer.update_3D_poses(T_poses_WC_dict)

        for name, points in elems_WC_dict.items():
            self.visualizer.update_3D_points(name, points, color='orange')

        fov_grid_PLT = self.visualizer.get_meshgrid_2D(self.IMG_H,
                                               self.IMG_W)
        grid_pts_number = fov_grid_PLT[0].shape[0] * fov_grid_PLT[0].shape[1] 

        print('----------FOV START')
        fov_pts_PLT = np.vstack([fov_grid_PLT[0].flatten(),
                                fov_grid_PLT[1].flatten(),
                                np.zeros(grid_pts_number),
                                np.ones(grid_pts_number)])

        fov_pts_IMG2D = np.vstack([fov_grid_PLT[1].flatten(),
                                fov_grid_PLT[0].flatten(),
                                np.ones(grid_pts_number)])


        fov_pts_CC = self.transformer.run("IMG2D_TO_CC", fov_pts_IMG2D)
        fov_pts_WC = self.transformer.run("CC_TO_WC", fov_pts_CC)

        self.visualizer.update_3D_points('FOV', fov_pts_WC, color='red')


        ## Visualize 2D
        self.visualizer.reset_display_2D()

        self.visualizer.update_2D_points(fov_pts_PLT, color='red')

        self.visualizer.update_2D_points(elems_PLT)

        ## Show Visualization
        self.visualizer.show_display()
        self.visualizer.reset_figure()

if __name__ == "__main__":

    #Initialize
    cfg = Config()
    stereo = SemanticLocalization(cfg)

    #Run
    # while True
    stereo.run()