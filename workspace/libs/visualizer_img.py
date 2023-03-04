import numpy as np
import cv2

class VisualizerIMG:
    def __init__(self):
        self.frame = None

    def set_frame(self, img):
        self.frame = img

    def draw_circles(self, elems, indices):
        colors = [(0, 0, 255), (0, 140, 255), (255, 0, 0), (0, 255, 0)]
        elems = elems.astype(int)

        ## For OpenCV coordinate
        elems[[0, 1]] = elems[[1, 0]]

        length = elems.shape[1]
        for i in range(length):
            center = tuple(elems[:2, i])
            idx = int(indices[0, i])
            cv2.circle(self.frame, center, radius=5, color=colors[idx], thickness=-1)

    def show(self):
        cv2.imshow('frame', self.frame)
        while True:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()



    def save(self, name):
        cv2.imwrite(f'results/{name}.png', self.frame)

