import numpy as np
import pyrender as pr
import trimesh

import time

# Create the scene
scene = pr.Scene()
# scene = pr.Scene(bg_color=[0.0, 0.0, 0.0], ambient_light=[1.0, 1.0, 1.0])

# Create the mesh
# mesh = pr.Mesh.from_points(pose[:, 0:3], colors=pose[:, 3:6]/255.0)
# node = pr.Node(mesh=mesh)

# tm = trimesh.creation.icosahedron()
pose = np.eye(4)

axis_tm = trimesh.creation.axis(origin_size=0.04, transform=pose)

origin_axis_tm = axis_tm.copy()
vehicle_axis_tm = axis_tm.copy()

origin_axis_m = pr.Mesh.from_trimesh(origin_axis_tm, poses=pose, smooth=False)
pose[:3, 3] = [1,1,1]
vehicle_axis_m = pr.Mesh.from_trimesh(vehicle_axis_tm, poses=pose, smooth=False)

origin_node = pr.Node(mesh=origin_axis_m)
vehicle_node = pr.Node(mesh=vehicle_axis_m, matrix=pose)
scene.add_node(vehicle_node)
scene.add_node(origin_node)
# scene.add(origin_axis)
# scene.add(vehicle_axis)
# pts = tm.vertices.copy()
# colors = np.random.uniform(size=pts.shape)
# m = pr.Mesh.from_points(pts, colors=colors)


# sm = trimesh.creation.uv_sphere(radius=0.1)
# sm.visual.vertex_colors = [1.0, 0.0, 0.0]
# tfs = np.tile(np.eye(4), (len(pts), 1, 1))
# tfs[:,:3,3] = pts

# m = pr.Mesh.from_trimesh(sm, poses=tfs)

# Create the viewer
light = pr.PointLight(color=[1.0, 1.0, 1.0], intensity=2.0)
cam = pr.PerspectiveCamera(yfov=np.pi / 3.0, aspectRatio=1.414)
nl = pr.Node(light=light, matrix=np.eye(4))
nc = pr.Node(camera=cam, matrix=np.eye(4))

scene.add_node(nl, parent_node=vehicle_node)
# scene.add(cam, parent_node=nm)

viewer = pr.Viewer(scene, use_raymond_lighting=True, 
                        run_in_thread=True, 
                        show_world_axis=True,
                        show_mesh_axes=True,
                        window_title='Vehicle Coordinate')

# r = pr.OffscreenRenderer(viewport_width=640,
#                                 viewport_height=480,
#                                 point_size=1.0)
# Update the mesh in real-time
i = 0
while True:

    # r.render(scene)
    pose = np.eye(4)
    pose[:3,3] = [i, 0, 0]
    viewer.render_lock.acquire()
    scene.set_pose(vehicle_node, pose)
    viewer.render_lock.release()
    i += 0.01
    time.sleep(0.1)