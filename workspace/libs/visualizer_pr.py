
import numpy as np

import pyrender as pr
import trimesh

class VisualizerPR:

    def __init__(self):
        self.scene = pr.Scene()
    


        self.nodes = {}

        self.viewer = pr.Viewer(self.scene,
                            use_raymond_lighting=True,
                            run_in_thread=True, 
                            show_world_axis=True,
                            show_mesh_axes=True,
                            window_title='Vehicle Coordinate')


        self.current_map_elements_node = pr.Node()

    def init_nodes(self):

        pose0 = np.eye(4)

        self.nodes['origin'] = self._get_node('origin', pose0)
        self.nodes['Vehicle'] = self._get_node('Vehicle', pose0)
        self.nodes['Camera'] = self._get_node('Camera', pose0)
        self.nodes['FOV'] = self._get_node('FOV', pose0)

        for key, node in self.nodes.items():
            self.scene.add_node(node)

    def update_3D_pose(self, name, pose):

        self.viewer.render_lock.acquire()
        self.scene.set_pose(self.nodes[name], pose)
        self.viewer.render_lock.release()

    def set_map_elements(self, pts):

        # tm = trimesh.creation.icosahedron()
        # pts = tm.vertices.copy()
        # length = pts.shape[0]

        sm = trimesh.creation.uv_sphere(radius=0.01)
        sm.visual.vertex_colors = [1.0, 0.0, 0.0]

        length = pts.shape[1]

        # print(pts.shape)
        tfs = np.tile(np.eye(4), (length, 1, 1))
        
        # print(pts.T[:, 1:4])
        tfs[:,:3,3] = pts.T[:, 1:4]
        # print(tfs)
        m = pr.Mesh.from_trimesh(sm, poses=tfs)


        new_node = pr.Node(mesh=m)

        self.viewer.render_lock.acquire()
        try:
            self.scene.remove_node(self.current_map_elements_node)
        except Exception as e:
            print(e)
        self.scene.add_node(new_node)
        self.viewer.render_lock.release()

        self.current_map_elements_node = new_node

    def _get_node(self, name, pose0):
        axis_mesh = self._get_axis_mesh()
        return pr.Node(name=name, mesh=axis_mesh, matrix=pose0)

    def _get_axis_mesh(self):
        pose = np.eye(4)
        axis_tm = trimesh.creation.axis(origin_size=0.04, transform=pose)
        return pr.Mesh.from_trimesh(axis_tm, poses=pose, smooth=False)

