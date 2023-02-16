#%% Imports

import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
from scipy.spatial.transform import Rotation


#%% Functions

def nominal_state_update(nominal_state, w_m, a_m, dt):
    """
    function to perform the nominal state update

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                    all elements are 3x1 vectors except for q which is a Rotation object
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :return: new tuple containing the updated state
    """
    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    # YOUR CODE HERE
    # new_p = np.zeros((3, 1))
    # new_v = np.zeros((3, 1))
    # new_q = Rotation.identity()
    new_p = p + v * dt + (1 / 2) * (q.as_matrix() @ (a_m - a_b) + g) * dt ** 2
    new_v = v + (q.as_matrix() @ (a_m - a_b) + g) * dt
    q_update = Rotation.from_rotvec(((w_m - w_b) * dt).flatten())
    new_q = q * q_update

    return new_p, new_v, new_q, a_b, w_b, g


def error_covariance_update(nominal_state, error_state_covariance, w_m, a_m, dt,
                            accelerometer_noise_density, gyroscope_noise_density,
                            accelerometer_random_walk, gyroscope_random_walk):
    """
    Function to update the error state covariance matrix

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :param accelerometer_noise_density: standard deviation of accelerometer noise
    :param gyroscope_noise_density: standard deviation of gyro noise
    :param accelerometer_random_walk: accelerometer random walk rate
    :param gyroscope_random_walk: gyro random walk rate
    :return:
    """

    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    # YOUR CODE HERE
    F_x = np.identity(18, dtype=np.float64)
    F_x[:3, 3:6] = np.identity(3) * dt
    F_x[3:6, 6:9] = - q.as_matrix() @ find_skew_symmetric_matrice((a_m - a_b).flatten()) * dt
    F_x[3:6, 9:12] = - q.as_matrix() * dt
    F_x[3:6, 15:18] = np.identity(3) * dt
    F_x[6:9, 6:9] = Rotation.from_rotvec(((w_m - w_b) * dt).flatten()).as_matrix().T
    F_x[6:9, 12:15] = - np.identity(3) * dt

    Q_i = np.identity(12, dtype=np.float64)
    Q_i[0:3, 0:3] = np.identity(3) * (accelerometer_noise_density ** 2) * (dt ** 2)
    Q_i[3:6, 3:6] = np.identity(3) * (gyroscope_noise_density ** 2) * (dt ** 2)
    Q_i[6:9, 6:9] = np.identity(3) * (accelerometer_random_walk ** 2) * dt
    Q_i[9:12, 9:12] = np.identity(3) * (gyroscope_random_walk ** 2) * dt

    F_i = np.zeros((18, 12), dtype=np.float64)
    F_i[3:15, :] = np.identity(12)



    # return an 18x18 covariance matrix
    return F_x @ error_state_covariance @ F_x.T + F_i @ Q_i @ F_i.T

def find_skew_symmetric_matrice(vec):
    result = np.zeros((3,3), dtype=np.float)
    result[0, 1] = - vec[2]
    result[1, 0] = vec[2]
    result[0, 2] = vec[1]
    result[2, 0] = - vec[1]
    result[1, 2] = - vec[0]
    result[2, 1] = vec[0]
    return result

def measurement_update_step(nominal_state, error_state_covariance, uv, Pw, error_threshold, Q):
    """
    Function to update the nominal state and the error state covariance matrix based on a single
    observed image measurement uv, which is a projection of Pw.

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param uv: 2x1 vector of image measurements
    :param Pw: 3x1 vector world coordinate
    :param error_threshold: inlier threshold
    :param Q: 2x2 image covariance matrix
    :return: new_state_tuple, new error state covariance matrix
    """
    
    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    # YOUR CODE HERE - compute the innovation next state, next error_state covariance
    P_c = q.as_matrix().T @ (Pw - p)
    innovation = uv - P_c[:2] / P_c[2]
    if np.linalg.norm(innovation) <= error_threshold:
        X_c = P_c[0, 0]
        Y_c = P_c[1, 0]
        Z_c = P_c[2, 0]
        dzdP = np.array([[1/Z_c, 0, - X_c / (Z_c ** 2)],
                         [0, 1/Z_c, - Y_c / (Z_c ** 2)]])
        dPdtheta = find_skew_symmetric_matrice((q.as_matrix().T @ (Pw - p)).flatten())
        dPdp = - q.as_matrix().T
        dzdtheta = dzdP @ dPdtheta
        dzdp = dzdP @ dPdp
        Ht = np.zeros((2, 18))
        Ht[:, :3] = dzdp
        Ht[:, 6:9] = dzdtheta
        Kt = error_state_covariance @ Ht.T @ np.linalg.inv(Ht @ error_state_covariance @ Ht.T + Q)
        dx = Kt @ innovation # 18x1 vector
        p += dx[:3]
        v += dx[3:6]
        q = q * Rotation.from_rotvec(dx[6:9].flatten())
        a_b += dx[9:12]
        w_b += dx[12:15]
        g += dx[15:18]
        error_state_covariance = (np.identity(18) - Kt @ Ht) @ error_state_covariance @ (np.identity(18) - Kt @ Ht).T
        error_state_covariance += Kt @ Q @ Kt.T
    return (p, v, q, a_b, w_b, g), error_state_covariance, innovation
