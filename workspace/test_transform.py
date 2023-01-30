import numpy as np
import numpy.linalg as npl

from libs.coordinate_transform import CoordinateTransformer
from libs.visualizer import Visualizer
from libs.map_element_extraction import MapElementExtractor

from config import Config

cfg = Config()

transformer = CoordinateTransformer(cfg)
visualizer = Visualizer()

world = visualizer.create_3d_world(mapsize=10, res=40, vis=False)
world_display = visualizer.get_world_display(world)

world_origin_WC = np.array([0, 0, 0, 0, 0, 0])
T_world_origin_WC = transformer.pose_to_4x4_Rt(world_origin_WC)
world_display.add_coordinate_marker_from_T(T_world_origin_WC)
world_display.add_text_on_point('[World]', *T_world_origin_WC[0:3, 3])

vehicle_pose_WC = np.array([5, 2, 0, 0, 0, np.deg2rad(90)])
T_vehicle_pose_WC = transformer.pose_to_4x4_Rt(vehicle_pose_WC)
world_display.add_coordinate_marker_from_T(T_vehicle_pose_WC)
world_display.add_text_on_point('[Vehicle]', *T_vehicle_pose_WC[0:3, 3])

VC_to_WC = T_vehicle_pose_WC
WC_to_VC = npl.inv(VC_to_WC)

camera_pose_VC = np.array([1, 0, 1.5, 0, np.deg2rad(100), 0]) 
T_camera_pose_VC = transformer.pose_to_4x4_Rt(camera_pose_VC)
T_camera_pose_WC = VC_to_WC @ T_camera_pose_VC
world_display.add_coordinate_marker_from_T(T_camera_pose_WC)
world_display.add_text_on_point('[Camera]', *T_camera_pose_WC[0:3, 3])

CC_to_VC = T_camera_pose_VC
VC_to_CC = npl.inv(CC_to_VC)

image_plane_CC = visualizer.create_3d_world(mapsize=0.5, res=50)
image_plane_CC = np.vstack((image_plane_CC[0:2, :], np.ones((2, image_plane_CC.shape[1]))))
image_plane_WC = VC_to_WC @ CC_to_VC @ image_plane_CC
world_display.add_points(image_plane_WC)

world_display.show()

