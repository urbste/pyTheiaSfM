from pytheia.pytheia.sfm import PoseFromThreePoints
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import time

Deg2Rad = 180.0/np.pi
r1 = 15 * Deg2Rad
r2 = -10 * Deg2Rad

R1 = R.from_rotvec(np.array([r1, 0, 0]))
R2 = R.from_rotvec(np.array([0, r2, 0]))
R_gt = (R1.as_matrix() @ R2.as_matrix()).astype(np.float32)
t = np.array([0.3, -1.7, 1.15], dtype=np.float32)

pts3d = np.array([[-0.3001, -1.4487, -0.7815],
                  [-0.5840, 0.6965, 0.764],
                  [1.2271, 1.3889, 1.1257]], dtype=np.float32)      
pts2d = np.zeros((2,3), dtype=np.float32)
for i in range(3):            
    tmp = R_gt @ pts3d[:,i] + t
    tmp /= tmp[2]
    pts2d[:,i] = tmp[0:2]

start = time.time()
for i in range(1000):
    d = PoseFromThreePoints(pts2d.T, pts3d.T)
end = time.time()


for i in range(len(d[1])):
    r_est = R.from_matrix(d[1][i])
    r_gt = R.from_matrix(R_gt)

    print("Rot error: {:.2f}".format(np.linalg.norm(r_gt.as_rotvec()-r_est.as_rotvec())))
    print("Trans error: {:.2f}".format(np.linalg.norm(d[2][i]-d[2][i])))

print("Time pytheia per pose: {:.6f}ms".format((end-start)/1000.*1000))

# start = time.time()
# for i in range(1000):
#     d = cv2.solveP3P(pts2d, pts3d, np.eye(3, dtype=np.float32), np.zeros((1,5), dtype=np.float32), cv2.SOLVEPNP_AP3P)
# end = time.time()
# print("Time OpenCV AP3P per pose: {:.6f}ms".format((end-start)/1000.*1000))

# start = time.time()
# for i in range(1000):
#     d = cv2.solveP3P(pts2d, pts3d, np.eye(3, dtype=np.float32), np.zeros((1,5), dtype=np.float32), cv2.SOLVEPNP_P3P)
# end = time.time()
# print("Time OpenCV P3P per pose: {:.6f}ms".format((end-start)/1000.*1000))