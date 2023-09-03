# Imports

import numpy as np
from scipy.spatial.transform import Rotation


# %%

def estimate_pose(uvd1, uvd2, pose_iterations, ransac_iterations, ransac_threshold):
    """
    Estimate Pose by repeatedly calling ransac

    :param uvd1:
    :param uvd2:
    :param pose_iterations:
    :param ransac_iterations:
    :param ransac_threshold:
    :return: Rotation, R; Translation, T; inliers, array of n booleans
    """

    R = Rotation.identity()
    print("RRRRRRRRR", R.as_matrix())

    for i in range(0, pose_iterations):
        w, t, inliers = ransac_pose(uvd1, uvd2, R, ransac_iterations, ransac_threshold)
        print("w", w)
        print("flattened w", w.ravel())
        R2 = Rotation.from_rotvec(w.reshape(3,))
        print("R2", R2)
        R = Rotation.from_rotvec(w.ravel()) * R

        print("Rotation, trans, inliers",R,t, inliers)

    return R, t, inliers

def solve_w_t(uvd1, uvd2, R0):
    """
    solve_w_t core routine used to compute best fit w and t given a set of stereo correspondences

    :param uvd1: 3xn ndarray : normailzed stereo results from frame 1
    :param uvd2: 3xn ndarray : normailzed stereo results from frame 1
    :param R0: Rotation type - base rotation estimate
    :return: w, t : 3x1 ndarray estimate for rotation vector, 3x1 ndarray estimate for translation
    """

    # TODO Your code here replace the dummy return value with a value you compute
    w = t = np.zeros((3,1))
    # print("Rotation", R0)

    rotation_matrix = R0.as_matrix()
    num_points = uvd1.shape[1]

    A_matrix = np.zeros((2 * num_points, 6))
    b = np.zeros((2 * num_points, 1))

    for i in range(num_points):
        y = rotation_matrix @ np.array([uvd2[0, i], uvd2[1, i], 1])
        d2 = uvd2[2, i]

        y_skew_d2 = np.array([[0, y[2], -y[1], d2, 0, 0],
                              [-y[2], 0, y[0], 0, d2, 0],
                              [y[1], -y[0], 0, 0, 0, d2]])

        b_2 = np.array([[1, 0, -uvd1[0, i]],
                        [0, 1, -uvd1[1, i]]])

        A_ = b_2 @ y_skew_d2
        b_ = -b_2 @ y

        A_matrix[2 * i:2 * i + 2, :] = A_
        b[2 * i:2 * i + 2, 0] = b_.ravel()

    op = np.linalg.lstsq(A_matrix, b, rcond=None)[0]
    w = op[0:3]
    t = op[3:]

    return w, t

def find_inliers(w, t, uvd1, uvd2, R0, threshold):
    """

    find_inliers core routine used to detect which correspondences are inliers

    :param w: ndarray with 3 entries angular velocity vector in radians/sec
    :param t: ndarray with 3 entries, translation vector
    :param uvd1: 3xn ndarray : normailzed stereo results from frame 1
    :param uvd2:  3xn ndarray : normailzed stereo results from frame 2
    :param R0: Rotation type - base rotation estimate
    :param threshold: Threshold to use
    :return: ndarray with n boolean entries : Only True for correspondences that pass the test
    """


    n = uvd1.shape[1]

    # TODO Your code here replace the dummy return value with a value you compute
    bool_array = np.zeros((n, 1))
    rotation_matrix = Rotation.as_matrix(R0)
    print("ROTTTTT", rotation_matrix)
    for i in range(n):
        uv_arr = np.concatenate((uvd2[0:2, i], np.array([1])))
        y = rotation_matrix @ np.array([uvd2[0, i], uvd2[1, i], 1])
        d2 = uvd2[2, i]
        y_skew_d2 = np.array([[0, float(y[2]), float(-y[1]), d2, 0, 0],
                              [float(-y[2]), 0, float(y[0]), 0, d2, 0],
                              [float(y[1]), float(-y[0]), 0, 0, 0, d2]])

        b_2 = np.array([[1, 0, -uvd1[0, i]],
                        [0, 1, -uvd1[1, i]]])

        b_ = ((-1 * b_2) @ y).reshape(-1, 1)
        A_ = b_2 @ y_skew_d2

        w_del = w.reshape(3, 1)
        t_del = t.reshape(3, 1)
        x = np.vstack((w_del, t_del))
        delta = (A_ @ x) - b_
        if np.linalg.norm(delta) < threshold:
            bool_array[i] = 1

    return bool_array.flatten()
    # return np.zeros(n, dtype='bool')


def ransac_pose(uvd1, uvd2, R0, ransac_iterations, ransac_threshold):
    """

    ransac_pose routine used to estimate pose from stereo correspondences

    :param uvd1: 3xn ndarray : normailzed stereo results from frame 1
    :param uvd2: 3xn ndarray : normailzed stereo results from frame 1
    :param R0: Rotation type - base rotation estimate
    :param ransac_iterations: Number of RANSAC iterations to perform
    :ransac_threshold: Threshold to apply to determine correspondence inliers
    :return: w, t : 3x1 ndarray estimate for rotation vector, 3x1 ndarray estimate for translation
    :return: ndarray with n boolean entries : Only True for correspondences that are inliers

    """
    n = uvd1.shape[1]


    # TODO Your code here replace the dummy return value with a value you compute
    w = t = np.zeros((3,1))
    best_inliers = np.zeros(n, dtype='bool')
    best_w = np.zeros((3, 1))
    best_t = np.zeros((3, 1))

    n = uvd1.shape[1]

    if ransac_iterations == 0:
        w, t = solve_w_t(uvd1, uvd2, R0)
        inliers = find_inliers(w, t, uvd1, uvd2, R0, ransac_threshold)
    else:
        max_len = 0
        w = t = np.zeros((3, 1))
        inliers = np.zeros(n, dtype=bool)

        for k in range(ransac_iterations):
            rn = np.random.choice(n, size=10, replace=False)
            w_, t_ = solve_w_t(uvd1[:, rn], uvd2[:, rn], R0)
            inliers_ = find_inliers(w_, t_, uvd1, uvd2, R0, ransac_threshold)

            if np.sum(inliers_) > max_len:
                max_len = np.sum(inliers_)
                w, t = w_, t_
                inliers = inliers_

    return w, t, inliers

    # return w, t, np.zeros(n, dtype='bool')
