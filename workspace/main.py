#!/usr/bin/python3
import rospy
from config import Config
from libs.data_loader import DataLoader
from libs.output_saver import OutputSaver
from libs.map_element_extraction import MapElementExtractor
from libs.coordinate_transform import CoordinateTransformer
from libs.visualizer_plt import VisualizerPLT
from libs.visualizer_ros import VisualizerROS
from libs.visualizer_img import VisualizerIMG
from libs.visualizer_pr import VisualizerPR
from libs.data_updater import DataUpdater
from libs.optimizer import Optimizer
import numpy as np
import numpy.linalg as npl
import cv2
import argparse

    
class SemanticLocalization:

    # pose = np.array[x, y, z, roll, pitch, yaw]

    def __init__(self, cfg, is_vis_pyrender=False):

        ## Load data from cfg

        self.updater = DataUpdater(cfg)
        self.camera_pose_VC = cfg.camera_pose_VC
        self.loader = DataLoader(cfg)
        self.saver = OutputSaver(cfg)
        self.extractor = MapElementExtractor(cfg)
        self.transformer = CoordinateTransformer()
        self.optimizer = Optimizer(self.transformer, self.extractor)
        self.is_vis_pyrender = is_vis_pyrender

        if self.is_vis_pyrender:
            self.visualizer_pr = VisualizerPR()
            self.visualizer_pr.init_nodes()

        self.visualizer_img = VisualizerIMG()
        # self.visualizer = VisualizerROS()
        # self.visualizer = VisualizerPLT()


        self.IMG_H = cfg.IMG_H
        self.IMG_W = cfg.IMG_W

        self.raw_imgs = self.loader.load_images('data/images/songdo_onehour_raw')
        self.seg_imgs = self.loader.load_images('data/images/songdo_onehour_seg')


        self.vehicle_pose_WC = cfg.initial_vehicle_pose_WC

        ## Set initial transforms
        self.transformer.set_transforms_CC_VC(cfg.camera_pose_VC)
        # self.transformer.set_transforms_IMG_CC(K=cfg.K_left_color)
        self.transformer.set_transforms_IMG_CC(K=cfg.K_ioniq60)
        self.transformer.set_transforms_PLT_IMG(cfg.plt_pose_IMG)
        self.transformer.set_transforms_CV_IMG(cfg.cv_pose_IMG)



    def run(self):
        ## Update vehicle pose
        vehicle_pose_WC = self.updater.get_vehicle_pose('scene1')
        raw_img = self.raw_imgs[0]
        seg_img = self.seg_imgs[0]

        h = seg_img.shape[0] #1080
        w = seg_img.shape[1] #1920
        c = seg_img.shape[2] #3


        vehicle_pose_WC[2] += -6 ## z offset

        self.transformer.update_transforms_VC_WC(vehicle_pose_WC)
        # self.transformer.tune_Rt("WC_TO_VC", pose=delta_pose) ## z offset
        self.transformer.update_transforms_CC_WC()
        self.transformer.update_transforms_IMG_WC()

        # elems_WC_dict = self.extractor.run(["VIRTUAL LINEMARK"], vehicle_pose_WC)
        elems_WC_dict, elems_length_dict = self.extractor.run(["LINEMARK", "TRAFFICLIGHT", "SAFETYSIGN", "SURFACEMARK"], vehicle_pose_WC)

        elems_WC_list = []
        elems_indices_list = []
        for i, (key, value) in enumerate(elems_WC_dict.items()):
            elems_WC_list.append(value)
            elems_indices_list += [i] * elems_length_dict[key]
        elems_indices = np.array(elems_indices_list).reshape(1, len(elems_indices_list))
        

        ##TODO: Add rectangular shape around Trafficlight

        
        ## To Camera Coordinate
        elems_WC = np.hstack(elems_WC_list)
        elems_CC = self.transformer.run("WC_TO_CC", elems_WC)

        ## Mask
        z_mask = elems_CC[2, :] > 0
        elems_CC = elems_CC[:, z_mask]
        elems_indices = elems_indices[:, z_mask]

        ## 2D Projection
        elems_IMG3D = self.transformer.run("CC_TO_IMG3D", elems_CC)
        elems_CV = self.transformer.run("IMG3D_TO_CV", elems_IMG3D)
        # print(f"\n Elems Altitude in VC:\n{elems_VC[2, :]}")



        ###########UCUCU
        seg_img_mask = seg_img[:, :, 0] == 2
        seg_mask_for_view = seg_img_mask.astype(np.uint8) * 255
        seg_mask_for_view = cv2.cvtColor(seg_mask_for_view, cv2.COLOR_GRAY2BGR)
        self.visualizer_img.set_frame(seg_mask_for_view)
        self.visualizer_img.draw_circles(elems_CV, elems_indices)
        self.visualizer_img.save('before')
        
        print(f'\n=====init pose: {vehicle_pose_WC}')
        self.optimizer.run(seg_img_mask.astype(np.uint8), pose0=vehicle_pose_WC)
        

        ## 2D Visualization

        elems_WC = elems_WC_dict['LINEMARK']
        elems_CV = self.transformer.run("WC_TO_CV", elems_WC)
        seg_mask_for_view = seg_img_mask.astype(np.uint8) * 255
        seg_mask_for_view = cv2.cvtColor(seg_mask_for_view, cv2.COLOR_GRAY2BGR)
        self.visualizer_img.set_frame(seg_mask_for_view)

        after_elems_indices = np.zeros([1, elems_CV.shape[1]])
        self.visualizer_img.draw_circles(elems_CV, after_elems_indices)
        self.visualizer_img.save('after')
        # self.visualizer_img.show()

        ## 3D Visualization
        if self.is_vis_pyrender:
            T_poses_WC_dict = self.transformer.get_all_T_poses_WC()
            for key, pose in T_poses_WC_dict.items():
                new_pose = self.transformer.run('WC_TO_VC', pose)
                self.visualizer_pr.update_3D_pose(key, new_pose)

            elems_VC = self.transformer.run("WC_TO_VC", elems_WC)
            self.visualizer_pr.set_map_elements(elems_VC)



if __name__ == "__main__":

    argparser = argparse.ArgumentParser()
    argparser.add_argument('--vis', default='plt')
    argparser.add_argument('--hz', default='30')

    args = argparser.parse_args()

    if args.vis == "pyrender":
        is_vis_pyrender = True
    else:
        is_vis_pyrender = False

    rospy.init_node('semantic_localization')

    #Initialize

    cfg = Config()
    stereo = SemanticLocalization(cfg, is_vis_pyrender)

    #Run
    # while True
    rate = rospy.Rate(30)
    # while not rospy.is_shutdown():
    import cProfile, pstats
    with cProfile.Profile() as pr:
        stereo.run()


    # pr.print_stats(sort='tottime')
    pr.print_stats(sort='ncalls')

    # stereo.run()
        # rate.sleep()