import numpy as np
import cv2
import numpy.linalg as npl
from scipy.optimize import least_squares
import lmfit
import matplotlib.pyplot as plt

def generate_square_border_image(size, thickness, x_shift, y_shift, seed = None):
    image = np.zeros((size, size), np.uint8)

    noise_range = [-300, 300]

    if seed is not None:
        np.random.seed(seed)

    for i in range(4):
        x_shift_noised = x_shift + np.random.randint(noise_range[0], noise_range[1])
        y_shift_noised = y_shift + np.random.randint(noise_range[0], noise_range[1])

        # Calculate the coordinates of the square border
        x1, y1 = (size // 4) + x_shift_noised, (size // 4) + y_shift_noised
        x2, y2 = (size * 3 // 4) + x_shift_noised, (size // 4) + y_shift_noised
        x3, y3 = (size * 3 // 4) + x_shift_noised, (size * 3 // 4) + y_shift_noised
        x4, y4 = (size // 4) + x_shift_noised, (size * 3 // 4) + y_shift_noised

        # Draw the square border on the image
        pts = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]], np.int32)
        cv2.polylines(image, [pts], True, 255, thickness)

    # Return the resulting image
    return image



class Optimizer:

    def __init__(self):
        self.seg_dist = None
        self.pixel_count = None

    def run(self, seg_img, pose0):

        self._set_probability_map(seg_img)

        params = lmfit.Parameters()
        params.add('x_trans', value=pose0[0], min=-500, max=500)
        params.add('y_trans', value=pose0[1], min=-500, max=500)
        fit_kws = {'diff_step' : 1000}

        result = lmfit.minimize(self._objective_function, params, method='least_squares', **fit_kws)

        print(f'\n{result.params.pretty_print()}')
        print(f"init: {pose0}")
        print(f"Result: [{result.params['x_trans'].value} {result.params['y_trans'].value}]")
        print(f"GT: {pose_gt}")


    def _set_probability_map(self, seg_img):
        self.seg_dist = self._distance_transfrom(seg_img)
        self.pixel_count = self.seg_dist.shape[0] * self.seg_dist.shape[1]

    def _distance_transfrom(self, gray):
        # Perform the distance transform
        dist = cv2.distanceTransform(gray, 
                                    cv2.DIST_L2, 0)
        
        # Standardize the distance transform
        normalized = cv2.normalize(dist, None, 0, 1.0, cv2.NORM_MINMAX)
        return normalized

    def _objective_function(self, params):

        x_trans = params['x_trans'].value
        y_trans = params['y_trans'].value

        #####TODO: change to real projection image
        proj = generate_square_border_image(1000, 20, x_trans, y_trans, seed=my_seed)
        ##########

        condition = proj != 0
        row_idxs, col_idxs = np.where(condition)

        output = np.ones(self.pixel_count)

        target_count = row_idxs.shape[0] 
        # print(f"pixel count: {pixel_count}")
        # print(f"target count: {target_count}")
        
        probs = self.seg_dist[(row_idxs, col_idxs)]

        inside = probs - 1
        print(probs)

        output[:target_count] = inside
        cost = np.sum(np.square(inside))
        cost2 = np.sum(np.square(output))
        # print(f"cost: {cost}")
        return output





if __name__ == "__main__":


    pose_gt = np.array([0, 0])
    my_seed = 1126
    seg = generate_square_border_image(1000, 20, pose_gt[0], pose_gt[1], seed=my_seed)


    opt = Optimizer()
    pose0 = np.array([100, -100])
    opt.run(seg, pose0)

    # # print(lmfit.fit_report(result))

    # print(f'\n{result.params.pretty_print()}')
    # # print(f"result: {result.params['x_trans']} {result.params['y_trans']}")
    # print(f"init: {pose0}")
    # print(f"Result: [{result.params['x_trans'].value} {result.params['y_trans'].value}]")
    # print(f"GT: {pose_gt}")

    # # print(cost)
        
        


    # cv2.imshow('seg', seg_dist)
    # cv2.imshow('proj', proj)
    # # cv2.imshow('proj', proj)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # plt.plot(cost_list)
    # plt.xlabel('iteration')
    # plt.ylabel('cost')
    # plt.show()
