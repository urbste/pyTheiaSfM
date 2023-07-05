# Steffen Urban, 2023
import numpy as np

def skew(vector):
    return np.array([[0, -vector[2], vector[1]], 
                    [vector[2], 0, -vector[0]], 
                    [-vector[1], vector[0], 0]])

def rot_between_vectors(a,b):
    # rotates a -> b

    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a,b)
    c = np.dot(a,b)
    s = np.linalg.norm(v)

    return np.eye(3) + skew(v) + np.linalg.matrix_power(skew(v),2)*((1-c)/s**2)