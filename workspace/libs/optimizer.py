import lmfit
import numpy as np
import cv2
class Optimizer:

    def __init__(self, transformer, extractor):
        self.seg_dist = None
        self.pixel_count = None
        self.transformer = transformer
        self.extractor = extractor
        self.height = 1080
        self.width = 1920
        self.iter_count = 0

    def run(self, seg_img, pose0):

        self._set_probability_map(seg_img)

        params = lmfit.Parameters()
        params.add('x_trans', value=pose0[0])
        params.add('y_trans', value=pose0[1])
        params.add('z_trans', value=pose0[2], min=-100, max=100)
        params.add('roll', value=pose0[3], min=np.rad2deg(-5), max=np.rad2deg(5))
        params.add('pitch', value=pose0[4], min=np.deg2rad(-10), max=np.deg2rad(10))
        # params.add('yaw', value=pose0[5], min=np.deg2rad(-180), max=np.deg2rad(180))
        params.add('yaw', value=pose0[5], min=pose0[5] - 1, max=pose0[5] + 1)

        fit_kws = {'diff_step' : 100}
        result = lmfit.minimize(self._objective_function, params, method='least_squares', **fit_kws)
        # result = lmfit.minimize(self._objective_function, params, method='least_squares')



        print(f'\n{result.params.pretty_print()}')
        # print(f"init: {pose0}")
        # print(f"Result: [{result.params['x_trans'].value} {result.params['y_trans'].value}]")
        print(lmfit.fit_report(result))


    def _set_probability_map(self, seg_img):
        self.seg_dist = self._distance_transfrom(seg_img)
        self.height = self.seg_dist.shape[0]
        self.width = self.seg_dist.shape[1]
        self.pixel_count = self.height * self.width

    def _distance_transfrom(self, gray):
        # Perform the distance transform
        dist = cv2.distanceTransform(gray, 
                                    cv2.DIST_L2, 0)
        
        # Standardize the distance transform
        normalized = cv2.normalize(dist, None, 0, 1.0, cv2.NORM_MINMAX)
        return normalized

    def _objective_function(self, params):
        print(f"\nIter:{self.iter_count}")
        self.iter_count += 1


        pose = np.array([params['x_trans'].value,
                        params['y_trans'].value,
                        params['z_trans'].value,
                        params['roll'].value,
                        params['pitch'].value,
                        params['yaw'].value
                         ])
        #####TODO: change to real projection image
        self.transformer.update_transforms_VC_WC(pose)
        self.transformer.update_transforms_CC_WC()
        self.transformer.update_transforms_IMG_WC()

        elems_WC_dict, elems_length_dict = self.extractor.run(["LINEMARK"], pose)

        elems_WC = elems_WC_dict['LINEMARK']
        elems_CC = self.transformer.run("WC_TO_CC", elems_WC)
        elems_IMG3D = self.transformer.run("CC_TO_IMG3D", elems_CC)
        elems_CV = self.transformer.run("IMG3D_TO_CV", elems_IMG3D)


        row_idxs = elems_CV[0, :].astype(int)
        col_idxs = elems_CV[1, :].astype(int)


        tmp_mask = row_idxs < self.height
        row_idxs = row_idxs[tmp_mask]
        col_idxs = col_idxs[tmp_mask]

        tmp_mask = row_idxs > 0
        row_idxs = row_idxs[tmp_mask]
        col_idxs = col_idxs[tmp_mask]

        tmp_mask = col_idxs < self.width
        row_idxs = row_idxs[tmp_mask]
        col_idxs = col_idxs[tmp_mask]

        tmp_mask = col_idxs > 0
        row_idxs = row_idxs[tmp_mask]
        col_idxs = col_idxs[tmp_mask]



        output = np.ones(self.pixel_count)
        # output = np.zeros(self.pixel_count)

        target_count = row_idxs.shape[0] 

        if (row_idxs.shape[0] != 0 and col_idxs.shape[0] != 0):
            probs = self.seg_dist[(row_idxs, col_idxs)]
            inside = probs - 1
            output[:target_count] = inside

        # cost = np.sum(np.square(inside))
        # cost2 = np.sum(np.square(output))
        return output