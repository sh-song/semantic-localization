import cv2
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

class Visualizer:


    def __init__(self):
        pass


    def draw_multiple_dots(self, img, dots):

        # dots = np.array([[100, 100], [200, 200], [300, 300]])

        h = img.shape[0]
        w = img.shape[1]

        # Draw circles on the img using NumPy array operations
        for x, y in dots:

            if 0 < x < h and 0 < y < w:
                cv2.circle(img , (x, y), 5, (0, 0, 255), -1)
            
        # Overlay the dots on the img
        return img

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

    def get_world_display(self, world):

        class WorldDisplay:
            def __init__(self, world):
                fig = plt.figure(figsize=(16,16))

                self.ax = fig.add_subplot(111, projection='3d')
                self.ax.scatter(world[0,:], world[1,:], world[2,:], marker='.', alpha=0.5)
                            
            def add_points(self, points):

                self.ax.scatter(points[0,:], points[1,:], points[2,:], marker='o', s=100, c='r', alpha=1.0)

            def add_ego(self, points):
                self.ax.scatter(points[0,:], points[1,:], points[2,:], marker='o', s=1000, c='g', alpha=1.0)
            def show(self):
                plt.show()

        return WorldDisplay(world)