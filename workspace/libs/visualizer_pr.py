
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
                            window_title='Vehicle Coordinate')

        self.current_map_elements_node = pr.Node()

    def init_nodes(self):

        pose0 = np.eye(4)

        self.nodes['origin'] = self._get_axis_node('origin', pose0)
        self.nodes['Vehicle'] = self._get_axis_node('Vehicle', pose0)
        self.nodes['Camera'] = self._get_axis_node('Camera', pose0)
        self.nodes['FOV'] = self._get_axis_node('FOV', pose0)

        self.nodes['Grid'] = self._get_grid_node(map_size=100)

        for key, node in self.nodes.items():
            self.scene.add_node(node)


    def update_3D_pose(self, name, pose):

        self.viewer.render_lock.acquire()
        self.scene.set_pose(self.nodes[name], pose)
        self.viewer.render_lock.release()

        
    def set_map_elements(self, pts):
        sm = trimesh.creation.uv_sphere(radius=0.1)
        sm.visual.vertex_colors = [1.0, 0.0, 0.0]

        length = pts.shape[1]
        tfs = np.tile(np.eye(4), (length, 1, 1))
        tfs[:,:3,3] = pts.T[:, 0:3]

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

    def _get_axis_node(self, name, pose0):
        axis_mesh = self._get_axis_mesh()
        return pr.Node(name=name, mesh=axis_mesh, matrix=pose0)

    def _get_axis_mesh(self):
        pose = np.eye(4)
        axis_tm = trimesh.creation.axis(origin_size=0.04, transform=pose)
        return pr.Mesh.from_trimesh(axis_tm, poses=pose, smooth=False)

    def _get_grid_node(self, map_size=100, spacing=0.5):
        '''
        map_size : [m]
        spacing : [m]
        '''
        size_x = map_size
        size_y = map_size
        spacing_factor = 1 / spacing
        x = np.linspace(-size_x // 2, size_x // 2 - 1, int(size_x * spacing_factor))
        y = np.linspace(-size_y // 2, size_y // 2 - 1, int(size_y * spacing_factor)) 
        z = np.zeros(1)

        xx, yy, zz = np.meshgrid(x, y, z, indexing='ij')
        print(f"size: {xx.shape}")
        grid_points = np.stack((xx, yy, zz), axis=-1).reshape((-1, 3))

        grid_mesh = pr.Mesh.from_points(grid_points)
        grid_node = pr.Node(mesh=grid_mesh)
        return grid_node