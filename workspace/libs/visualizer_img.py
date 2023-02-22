import numpy as np

import cv2

class VisualizerIMG:
    def __init__(self):
        pass


        self.frame = None

    def set_frame(self, img):
        self.frame = img

    def draw_circles(self, elems):
        # Set the color of the markers to orange
        marker_color = (0, 140, 255)  # BGR format

        # Draw the markers on the frame

        elems = elems.astype(int)
        print(f"suspicious: {tuple(elems[:2, 0])}")
        length = elems.shape[1]

        ## For OpenCV coordinate
        elems[[0, 1]] = elems[[1, 0]]

        for i in range(length):
            center = tuple(elems[:2, i])
            cv2.circle(self.frame, center, radius=5, color=marker_color, thickness=-1)

    def show(self):
        cv2.imshow('frame', self.frame)


