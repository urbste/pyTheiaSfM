import pytheia as pt

import numpy as np
from numpy.random import rand

# two camera scene for testing functions in sfm pose folder

R1 = np.eye(3)
t1 = np.array([0.0,0.0,5.0])
R2 = np.array([[0.0,0.0,-1.0],[0.0,1.0,0.0],[1.0,0.0,0.0]]).T
t2 = np.array([0.0,0.0,5.0])
pts3d = rand(3,10) * 2

tmp1 = (R1@pts3d).T + t1
tmp1 = tmp1.T
tmp1 /= tmp1[2,:]
pts2d_cam1 = tmp1[0:2,:]

tmp2 = (R2@pts3d).T + t2
tmp2 = tmp2.T
tmp2 /= tmp2[2,:]
pts2d_cam2 = tmp2[0:2,:]


rt = pt.sfm.PoseFromThreePoints(pts2d_cam1.T, pts3d.T)
print('PoseFromThreePoints')
print(rt)
rt = pt.sfm.NormalizedEightPointFundamentalMatrix(pts2d_cam1.T, pts2d_cam2.T)
print('NormalizedEightPointFundamentalMatrix')
print(rt)
rt = pt.sfm.FivePointRelativePose(pts2d_cam1.T, pts2d_cam2.T)
print('FivePointRelativePose')
print(rt)
rt = pt.sfm.FourPointPoseAndFocalLength(pts2d_cam1.T, pts3d.T)
print('FourPointPoseAndFocalLength')
print(rt)
rt = pt.sfm.FourPointHomography(pts2d_cam1.T, pts2d_cam2.T)
print('FourPointHomography')
print(rt)
rt = pt.sfm.FourPointsPoseFocalLengthRadialDistortion(pts2d_cam1.T, pts3d.T)
print('FourPointsPoseFocalLengthRadialDistortion')
print(rt)


rt = pt.sfm.DlsPnp(pts2d_cam1.T, pts3d.T)
print('DlsPnp')
print(rt)
rt = pt.sfm.SevenPointFundamentalMatrix(pts2d_cam1[:,0:7].T, pts2d_cam2[:,0:7].T)
print('SevenPointFundamentalMatrix')
print(rt)

'''
f1 = np.array([2,1], dtype='float64')
f2 = np.array([4,3], dtype='float64')
fc1 = FeatureCorrespondence(f1, f2)
essential_matrix = np.array([[1,1,0],[1,0,1],[0,-1,1]], dtype= 'float64')
rt = GetBestPoseFromEssentialMatrix(essential_matrix, fc1)
print('GetBestPoseFromEssentialMatrix')
print(rt)
'''
'''
NormalizedEightPointFundamentalMatrix
FivePointRelativePose
FourPointPoseAndFocalLength
FourPointsPoseFocalLengthRadialDistortion
FourPointHomography
DlsPnp
SevenPointFundamentalMatrix

RelativePoseFromTwoPointsWithKnownRotation
PositionFromTwoRays
SimTransformPartialRotation
FourPointRelativePosePartialRotation
ThreePointRelativePosePartialRotation
TwoPointPosePartialRotation
'''