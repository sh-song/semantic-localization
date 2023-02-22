import numpy as np
import pyrender as pr
import trimesh

# # Define the size of the 3D-grid
# x_size = 100
# y_size = 100
# z_size = 5

# # Create a set of points that define the 3D-grid
# points = np.empty((x_size*y_size*z_size, 3), dtype=np.float32)
# for z in range(z_size):
#     for y in range(y_size):
#         for x in range(x_size):
#             i = x + y * x_size + z * x_size * y_size
#             points[i] = [x - x_size/2, y - y_size/2, z]

# # Create a Trimesh object from the set of points
# mesh = trimesh.Trimesh(vertices=points, process=False)

# print(mesh)

# # Create a PyRender scene and add the mesh to it
# # scene = pyrender.Scene(bg_color=[0.0, 0.0, 0.0], ambient_light=[1.0, 1.0, 1.0])
# scene = pr.Scene()

# mesh_node = pr.Node(mesh=pr.Mesh.from_trimesh(mesh, smooth=False))
# scene.add_node(mesh_node)




# Create a 3D grid of points spaced 1m apart, centered at the origin
size_x = 1000
size_y = 1000
x = np.linspace(-size_x // 2, size_x // 2 - 1, size_x)
y = np.linspace(-size_y // 2, size_y // 2 - 1, size_y) 
z = np.zeros(1)

xx, yy, zz = np.meshgrid(x, y, z, indexing='ij')
print(f"size: {xx.shape}")
grid_points = np.stack((xx, yy, zz), axis=-1).reshape((-1, 3))

grid_mesh = pr.Mesh.from_points(grid_points)
grid_node = pr.Node(mesh=grid_mesh)
# Create a mesh object from the grid points


# Create a scene and add the mesh object to it
scene = pr.Scene()
scene.add_node(grid_node)


tm = trimesh.creation.icosahedron()
m = pr.Mesh.from_trimesh(tm, smooth=False)
scene.add(m)
# Create a pr viewer and render the scene
viewer = pr.Viewer(scene, use_raymond_lighting=True, 
                        # run_in_thread=True, 
                        # show_world_axis=True,
                        # show_mesh_axes=True,
                        window_title='Vehicle Coordinate')

