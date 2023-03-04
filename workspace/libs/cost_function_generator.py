import numpy as np
import cv2
import numpy.linalg as npl

def generate_square_border_image(size, thickness, x_shift, y_shift):
    image = np.zeros((size, size), np.uint8)

    # Calculate the coordinates of the square border
    x1, y1 = (size // 4) + x_shift, (size // 4) + y_shift
    x2, y2 = (size * 3 // 4) + x_shift, (size // 4) + y_shift
    x3, y3 = (size * 3 // 4) + x_shift, (size * 3 // 4) + y_shift
    x4, y4 = (size // 4) + x_shift, (size * 3 // 4) + y_shift

    # Draw the square border on the image
    pts = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]], np.int32)
    cv2.polylines(image, [pts], True, 255, thickness)

    # Return the resulting image
    return image

def distance_transform(gray):
    # Perform the distance transform
    dist = cv2.distanceTransform(gray, 
                                cv2.DIST_L2, 0)
    
    # Standardize the distance transform
    normalized = cv2.normalize(dist, None, 0, 1.0, cv2.NORM_MINMAX)
    return normalized

def levenberg_marquardt(f, jacobian, y, x0, tol=1e-6, maxiter=100):
    x = x0
    lambda_ = 0.01
    for i in range(maxiter):
        r = y - f(x)
        J = jacobian(x)
        H = J.T @ J
        while True:
            try:
                p = np.linalg.solve(H + lambda_ * np.eye(len(x)), J.T @ r)
            except np.linalg.LinAlgError:
                lambda_ *= 10
            else:
                break
        x_new = x + p
        r_new = y - f(x_new)
        if np.sum(r_new ** 2) < np.sum(r ** 2):
            lambda_ /= 10
            x = x_new
            r = r_new
        else:
            lambda_ *= 10
        if np.linalg.norm(p) < tol:
            break
    return x


import numpy as np
from scipy.optimize import NonlinearConstraint, minimize

class Optimizer:

    def __init__(self):

        self.prob_map = None
        self.proj_img = None


    def set_probability_map(self, prob_map):
        self.prob_map = prob_map
    def set_projection_image(self, proj_img):
        self.proj_img


    def cost_function(self, pose):
        x, y, z, r, p, y = pose
        # Apply the 6DOF transformation to the image
        M = np.array([
            [np.cos(y)*np.cos(p), -np.sin(y)*np.cos(r)+np.cos(y)*np.sin(p)*np.sin(r), np.sin(y)*np.sin(r)+np.cos(y)*np.sin(p)*np.cos(r), x],
            [np.sin(y)*np.cos(p), np.cos(y)*np.cos(r)+np.sin(y)*np.sin(p)*np.sin(r), -np.cos(y)*np.sin(r)+np.sin(y)*np.sin(p)*np.cos(r), y],
            [-np.sin(p), np.cos(p)*np.sin(r), np.cos(p)*np.cos(r), z],
            [0, 0, 0, 1]
        ])
        transformed_image = cv2.warpPerspective(self.image, M, self.image.shape[:2][::-1])
        # Calculate the L2 norm of each pixel value in the transformed image
        l2_norm = np.sqrt(np.sum((transformed_image.astype('float') - 1)**2, axis=2))
        return np.sum(l2_norm)
    
    def run(self, x0):
        def f(x):
            return self.cost_function(x)
        
        def jacobian(x):
            epsilon = 1e-6
            J = np.zeros((self.image.shape[0]*self.image.shape[1], 6))
            for i in range(6):
                dx = np.zeros(6)
                dx[i] = epsilon
                J[:, i] = (f(x + dx) - f(x - dx)) / (2 * epsilon)
            return J
        
        y = np.zeros(self.image.shape[0]*self.image.shape[1]) #size== number of pixels
        x_opt = levenberg_marquardt(f, jacobian, y, x0)
        return x_opt



if __name__ == "__main__":

    xshift = 3
    yshift = 4
    pose = np.array([xshift, yshift])
    pose0 = np.array([0, 0])

    seg = generate_square_border_image(800, 20, xshift, yshift)

    proj = generate_square_border_image(800, 20, 0, 0)
    seg_dist = distance_transform(seg)

    optimizer = Optimizer()

    
    ### Levenberg-Marquardt
    x = pose0
    lambda_ = 0.01
    maxiter = 100

    pixel_count = seg.shape[0] * seg.shape[1]
    
    y = np.zeros(pixel_count)
    def f(x):
        return optimizer.cost_fuction(x)

    def jacobian(x, f):
        
        x_size = x.shape[0] #6
        epsilon = 1e-6
        J = np.zeros((pixel_count, x_size))
        for i in range(x_size):
            dx = np.zeros(x_size)
            dx[i] = epsilon
            J[:, i] = (f(x + dx) - f(x - dx)) / (2 * epsilon)
        return J
    

    for i in range(maxiter):
        r = y - f(x)
        J = jacobian(x, f)
        H = J.T @ J

        ## TODO: implement while-loop with exception
        x_delta = npl.solve(H + lambda_ * np.eye(len(x)), J.T @ r)
        ##

        x_new = x + x_delta
        r_new = y - f(x_new)
        if 










    # optimized_pose = optimizer.run(pose0)
    cv2.imshow('seg', seg_dist)
    cv2.imshow('proj', proj)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
