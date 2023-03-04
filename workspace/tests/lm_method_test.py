import numpy as np
import cv2
import numpy.linalg as npl
from scipy.optimize import least_squares

def generate_square_border_image(size, thickness, x_shift, y_shift):
    image = np.zeros((size, size), np.uint8)

    for noise in [0, 50, 100, -39]:
        x_shift_noised = x_shift + noise
        y_shift_noised = y_shift + noise

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

def distance_transform(gray):
    # Perform the distance transform
    dist = cv2.distanceTransform(gray, 
                                cv2.DIST_L2, 0)
    
    # Standardize the distance transform
    normalized = cv2.normalize(dist, None, 0, 1.0, cv2.NORM_MINMAX)
    return normalized



if __name__ == "__main__":

    xshift = 50
    yshift = 0
    pose_gt = np.array([xshift, yshift])
    pose0 = np.array([30, 100])

    seg = generate_square_border_image(1000, 20, pose_gt[0], pose_gt[1])

    seg_dist = distance_transform(seg)


    pixel_count = seg_dist.shape[0] * seg_dist.shape[1]

    proj = None

    def f(pose0):
        global proj
        proj = generate_square_border_image(1000, 20, pose0[0], pose0[1])
        row_idxs, col_idxs = np.where(proj != 0)

        print(f"pose: {pose0}")
        pixel_count = seg.shape[0] * seg.shape[1]
        output = np.zeros(pixel_count)
        print(f"pixel count: {pixel_count}")

        target_count = row_idxs.shape[0] 
        print(f"target count: {target_count}")
        

        probs = seg_dist[(row_idxs, col_idxs)]


        inside = probs - 1
        output[:target_count] = inside
        cost = np.sum(np.square(inside))
        cost2 = np.sum(np.square(output))
        print(f"cost: {cost}")
        print(f"cost: {cost2}")
        return output

    # cost= f(pose0)

    # bounds = [(-1000, 1000), (-1000, 1000)]
    bounds = ([-500, -500], [500, 500])
    result = least_squares(f, pose0, method='trf', bounds=bounds, verbose=1, diff_step=0.01)

    print(f"result: {result.x}")
    print(f"GT: {pose_gt}")

    # print(cost)
        
        


    cv2.imshow('seg', seg_dist)
    cv2.imshow('proj', proj)
    # cv2.imshow('proj', proj)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

