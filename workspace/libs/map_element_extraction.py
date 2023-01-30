import numpy as np



class MapElementExtractor:

    def __init__(self, cfg):
        self.cfg = cfg
        self.current_elements = np.ones([4, 100], dtype=np.float32)

        element = np.array([3.0, 3.0, 3.0, 1.0], dtype=np.float32)


    def generate_elements(self, n):
        n = (10 - 0) // 1
        xs = np.ones(n) * 1
        ys = np.arange(0, 10, 1)
        zs = np.ones(n) * 0
        ones = np.ones(n)

        elem = np.hstack([xs, ys, zs, ones]).reshape(4, n)
        return elem
    def generate_straight_lane(self, x):
        length = 20
        n = 50
        xs = np.ones(n) * x
        ys = np.linspace(0, 20, n)
        zs = np.ones(n) * 0
        ones = np.ones(n)

        elem = np.hstack([xs, ys, zs, ones]).reshape(4, n)
        return elem
    def run(self, local_map=None):

        self.current_elements = self.generate_elements(3)


        return self.current_elements