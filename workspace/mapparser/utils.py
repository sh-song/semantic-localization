import numpy as np
from quadratic_spline_interpolate import QuadraticSplineInterpolate


def euc_distance(pt1, pt2):
    return np.sqrt((pt2[0]-pt1[0])**2+(pt2[1]-pt1[1])**2)

def find_nearest_idx(pts, pt):
    min_dist = sys.maxsize
    min_idx = 0

    for idx, pt1 in enumerate(pts):
        dist = euc_distance(pt1, pt)
        if dist < min_dist:
            min_dist = dist
            min_idx = idx

    return min_idx

def filter_same_points(points):
    filtered_points = []
    pre_pt = None

    for pt in points:
        if pre_pt is None or pt != pre_pt:
            filtered_points.append(pt)    

        pre_pt = pt

    return filtered_points

def interpolate(points, precision):
    points = filter_same_points(points)

    if len(points) < 2:
        return points, None, None, None

    wx, wy, wz = zip(*points)
    itp = QuadraticSplineInterpolate(list(wx), list(wy))
    itp2 = QuadraticSplineInterpolate(list(wx), list(wz))

    itp_points = []
    s = []
    yaw = []
    k = []

    for n, ds in enumerate(list(np.arange(0.0, itp.s[-1], precision)) + [itp.s[-1]]):
        s.append(ds)
        x, y = itp.calc_position(ds)
        zs = itp2.calc_s_from_x(x)
        _, z = itp2.calc_position(zs)
        dyaw = itp.calc_yaw(ds)

        dk = itp.calc_curvature(ds)

        itp_points.append((float(x), float(y), float(z)))
        yaw.append(dyaw)
        k.append(dk)

    return itp_points, s, yaw, k