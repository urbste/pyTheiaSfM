from random_recon_gen import RandomReconGenerator
import pytheia as pt
import numpy as np
from scipy.spatial.transform import Rotation as R

def get_view_graph(recon, outlier_chance=0.1):
    view_pairs, init_rotations = {}, {}
    vids = recon.ViewIds()
    view_graph = pt.sfm.ViewGraph()

    gt_rotations = {}
    outlier_edges = []
    for i in range(0, len(vids)):
        id1 = vids[i]
        init_rotations[id1] = np.zeros((3,1)) # just initialize with zeros here
        ri = recon.View(id1).Camera().GetOrientationAsAngleAxis()
        gt_rotations[id1] = ri
        for j in range(i, len(vids)):
            id2 = vids[j]

            rj = recon.View(id2).Camera().GetOrientationAsAngleAxis()

            two_view_info = pt.sfm.TwoViewInfo()
            two_view_info.focal_length_1 = 1.0
            two_view_info.focal_length_2 = 1.0
            two_view_info.position_2 = np.zeros((3,1), dtype=np.float32)
            two_view_info.rotation_2 = pt.math.RelativeRotationFromTwoRotations(ri, rj) 
            if np.random.rand() < outlier_chance:
                two_view_info.rotation_2 = two_view_info.rotation_2 + np.random.rand(3)
                outlier_edges.append((id1, id2))
            view_graph.AddEdge(id1, id2, two_view_info)
    
    return view_graph, outlier_edges

def test_view_graph_cycles():
    gen = RandomReconGenerator()
    gen.generate_random_recon()
    
    add_noise_rand_chance = 0.1 # add a wrong connection with 10% chance
    view_graph, outlier_edges = get_view_graph(gen.recon, add_noise_rand_chance)

    max_rot_error = 20
    pt.sfm.FilterViewGraphCyclesByRotation(max_rot_error, view_graph)

    # check if all outlier edges are removed
    for e in outlier_edges:
        assert view_graph.HasEdge(e[0],e[1]) == False

if __name__ == "__main__":
    test_view_graph_cycles()

    test_view_graph_cycles()