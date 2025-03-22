import numpy as np

def calculate_center(box):
    """Calculates the center coordinates of a bounding box."""
    x, y, w, h = box
    center_x = x + w / 2
    center_y = y + h / 2
    return center_x, center_y

def calculate_bottom_center(box):
    """Calculates the bottom center coordinates of a bounding box."""
    x, y, w, h = box
    bottom_center_x = x + w / 2
    bottom_center_y = y + h
    return bottom_center_x, bottom_center_y

def initialize_kalman_filter(dt, u_x, u_y, std_acc, x_std_meas, y_std_meas):
    """Initializes a Kalman filter for tracking."""
    A = np.matrix([[1, 0, dt, 0],
                   [0, 1, 0, dt],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    B = np.matrix([[(dt**2)/2, 0],
                   [0, (dt**2)/2],
                   [dt, 0],
                   [0, dt]])
    H = np.matrix([[1, 0, 0, 0],
                   [0, 1, 0, 0]])
    Q = np.matrix([[(dt**4)/4, 0, (dt**3)/2, 0],
                   [0, (dt**4)/4, 0, (dt**3)/2],
                   [(dt**3)/2, 0, dt**2, 0],
                   [0, (dt**3)/2, 0, dt**2]]) * std_acc**2
    R = np.matrix([[x_std_meas**2, 0],
                   [0, y_std_meas**2]])
    P = np.eye(A.shape[1])
    x = np.matrix([[0], [0], [0], [0]])
    return A, B, H, Q, R, P, x

def update_kalman_filter(A, B, H, Q, R, P, x, u, z):
    """Updates the Kalman filter with new measurements."""
    # Predict
    x = np.dot(A, x) + np.dot(B, u)
    P = np.dot(np.dot(A, P), A.T) + Q

    # Update
    S = np.dot(H, np.dot(P, H.T)) + R
    K = np.dot(np.dot(P, H.T), np.linalg.inv(S))
    y = z - np.dot(H, x)
    x = x + np.dot(K, y)
    I = np.eye(H.shape[1])
    P = (I - np.dot(K, H)) * P

    return x, P
