import cv2
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

class Visualizer:


    def __init__(self):
        pass


    def draw_multiple_dots(self, img, elems):

        # dots = np.array([[100, 100], [200, 200], [300, 300]])

        dots = elems[0:2, :].T
        print(dots)
        h = img.shape[0]
        w = img.shape[1]
        print(f"height: {h}, width: {w}")

        # Draw circles on the img using NumPy array operations
        for x, y in dots:

            if 0 < x < h and 0 < y < w:
                print(f"Draw point in ({x}, {y})")
                cv2.circle(img , (x, y), 5, (0, 0, 255), -1)
            
        # Overlay the dots on the img
        return img

    def plot_2d(self, elems):
        plt.scatter(elems[0], elems[1])
        plt.show()



    def get_meshgrid_from_image(self, img):
        h_half = img.shape[0] // 2 #512 /2
        w_half = img.shape[1] // 2 #1392 /2
 

        h_space = np.linspace(-h_half, h_half, h_half//10)
        w_space = np.linspace(-w_half, w_half, w_half//10)

        # Generate the meshgrid
        height, width = np.meshgrid(h_space, w_space)

        return width, height
    def create_3d_world(self, mapsize=400, res=16, vis=False):

        baseline = np.arange(0, mapsize, mapsize / res)
        world = []
        for x in baseline:
            xs = np.ones(res) * x
            ys = baseline
            world.append(np.hstack([xs, ys]).reshape(2,res))

        world = np.concatenate(world, axis=1)

        world = np.vstack((world, np.zeros((1, world.shape[1]))))

        if vis:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.scatter(world[0,:], world[1,:], world[2,:], marker='.')
            plt.show()

        return world


    def get_image_plane_display(self):

        class ImagePlaneDisplay():

            def __init__(self):
                fig = plt.figure(figsize=(16,16))

                # Set plot
                self.ax = fig.add_subplot(111)
                self.ax.set_title('<Image Plane>')
                self.ax.set_aspect('equal')

            def add_points(self, points):
                self.ax.scatter(points[0, :], points[1, :], color='orange')

            def add_meshgrid(self, gridX, gridY):
                self.ax.scatter(gridX, gridY, color='red', marker='s')

            def show(self):
                plt.show() 

        return ImagePlaneDisplay()


    def get_world_display(self, world):

        class WorldDisplay:
            def __init__(self, world):
                fig = plt.figure(figsize=(16,16))

                # Set plot
                self.ax = fig.add_subplot(111, projection='3d')

                self.ax.set_title('<World>')


                self.ax.auto_scale_xyz([0, 10], [0, 10], [0, 3])
                self.ax.set_aspect('equal')

                # Set axis names
                self.ax.set_xlabel('$X$', fontsize=20)
                self.ax.set_ylabel('$Y$', fontsize=20)
                self.ax.set_zlabel('$Z$', fontsize=20, rotation = 0)

                # Set world
                self.ax.scatter(world[0,:], world[1,:], world[2,:], marker='.', alpha=0.3)
                            
            def add_points(self, points, color='r', marker='o', size=100):
                self.ax.scatter(points[0,:], points[1,:], points[2,:], marker=marker, s=size, c=color, alpha=1.0)

            def add_vector_from_T(self, T):
                base = np.array([1, 0, 0])
                start_point = T[0:3, 3]
                end_point = T[0:3, 0:3] @ base

                self.ax.quiver(*start_point, *end_point, color='red')

            def add_coordinate_marker_from_T(self, T, length=1):
                start_point = T[0:3, 3]
                rot =T[0:3, 0:3] 
                colors = ['red', 'blue', 'green']
                for i, color in enumerate(colors):
                    basis = np.zeros(3)
                    basis[i] = length
                    end_point = rot @ basis
                    self.ax.quiver(*start_point, *end_point, color=color)

            def add_text_on_point(self, text, x, y, z):
                self.ax.text(x, y, z, text, size=20, color='k')


            def add_ego(self, points):
                self.ax.scatter(points[0,:], points[1,:], points[2,:], marker='^', s=700, c='gray', alpha=1.0)
            def show(self):
                plt.show()

        return WorldDisplay(world)