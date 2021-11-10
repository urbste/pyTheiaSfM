import numpy as np
from numpy.core.numeric import ones_like


def add_noise_to_point(points_2d, noise_std):
    points_2d_noisy = points_2d + np.random.randn(points_2d.shape[0], points_2d.shape[1]) * noise_std
    return points_2d_noisy

