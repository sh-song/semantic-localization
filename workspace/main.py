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
        # Load data
        left_imgs = self.loader.load_images(self.cfg.left_images)

        img = left_imgs[0]

        pose_vehicle = np.array([5, 0, 0, 0, 0, np.deg2rad(90)])
        print('type', img.shape)
        # elems_world = self.extractor.run()
        left_lane = self.extractor.generate_straight_lane(4.5)
        right_lane = self.extractor.generate_straight_lane(5.5)

        elems_world = np.hstack([left_lane, right_lane])
        output = self.transformer.run(elems_world, pose_vehicle)
        print(output)

        world = self.visualizer.create_3d_world(mapsize=10, res=40, vis=False)

        world_display = self.visualizer.get_world_display(world)

        world_display.add_points(left_lane)
        world_display.add_points(right_lane)
        world_display.add_ego(pose_vehicle[:4].reshape(4,1))
        world_display.show()

        img = self.visualizer.draw_multiple_dots(img, output.T)
        cv2.imshow('yes', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
if __name__ == "__main__":

    #Initialize
    cfg = Config()
    stereo = SemanticLocalization(cfg)

    #Run
    stereo.run()