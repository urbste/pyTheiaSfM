# Steffen Urban, 2023
import numpy as np
import cv2

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


def reprojection_error(recon):

    reproj_errors = []
    for vid in recon.ViewIds():
        view = recon.MutableView(vid)
        if view.IsEstimated():
            for tid in view.TrackIds():
                if recon.Track(tid).IsEstimated():
                    rep_pt2 = view.Camera().ProjectPoint(recon.Track(tid).Point())[1]
                    pt2 = view.GetFeature(tid).point
                    reproj_errors.append(np.linalg.norm(rep_pt2-pt2))
        
    return np.mean(np.array(reproj_errors))

def plot_loftr_matches(kpts1, image1, kpts2, image2):
    pair = np.hstack((image1, image2))
    for i in range(len(kpts1)):
        kpi, kpj = kpts1[i], kpts2[i]
        cv2.circle(pair, (int(kpi[0]), int(kpi[1])), 2, (0,0,255), -1)
        cv2.circle(pair, (int(kpj[0])+image2.shape[1], int(kpj[1])), 2, (0,0,255), -1)
        cv2.line(pair, (int(kpi[0]), int(kpi[1])),
                        (int(kpj[0])+image2.shape[1], int(kpj[1])), (255,0,0), 1)
    return cv2.cvtColor(pair, cv2.COLOR_RGB2BGR)
