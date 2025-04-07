from random_recon_gen import RandomReconGenerator
import pytheia as pt
import numpy as np
from scipy.spatial.transform import Rotation as R

def test_robust_rotation_estimator():
    gen = RandomReconGenerator()
    gen.generate_random_recon()

    # init the options to the rotation estimator
    rotation_estimator_options = pt.sfm.RobustRotationEstimatorOptions()
    #rotation_estimator_options.max_num_l1_iterations = 100
    rotation_estimator = pt.sfm.RobustRotationEstimator(rotation_estimator_options)
    
    vids = gen.recon.ViewIds()
    
    view_pairs, init_rotations = {}, {}
    gt_rotations = {}
    for i in range(0, len(vids)):
        id1 = vids[i]
        init_rotations[id1] = np.zeros((3,1)) # just initialize with zeros here
        ri = gen.recon.View(id1).Camera().GetOrientationAsAngleAxis()
        gt_rotations[id1] = ri
        for j in range(i, len(vids)):
            id2 = vids[j]

            rj = gen.recon.View(id2).Camera().GetOrientationAsAngleAxis()

            two_view_info = pt.sfm.TwoViewInfo()
            two_view_info.focal_length_1 = 1.0
            two_view_info.focal_length_2 = 1.0
            two_view_info.position_2 = np.zeros((3,1), dtype=np.float32)
            two_view_info.rotation_2 = pt.math.RelativeRotationFromTwoRotations(ri, rj)

            view_pairs[(id1, id2)] = two_view_info
    
    # rotation_estimator.SetFixedGlobalRotations({vids[0]})
    result = rotation_estimator.EstimateRotations(view_pairs, init_rotations)

    # compare results
    aligned = pt.math.AlignOrientations(gt_rotations, result)

    for id in aligned:
        rij = pt.math.RelativeRotationFromTwoRotations(aligned[id], gt_rotations[id])
        assert np.linalg.norm(rij) < 1e-8

if __name__ == "__main__":
    test_robust_rotation_estimator()

