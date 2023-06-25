# Steffen Urban, 2023

# This simple example demonstrates the use of pytheia to reconstruct the 
# Fountain dataset scene
# If you want to build your own SFM pipeline make sure to 
# adapt the feature matchers and camera parameters accordingly
import numpy as np
import cv2, os, glob, argparse
import pytheia as pt

min_num_inlier_matches = 30

# create correspondences of keypoints locations from indexed feature matches
def correspondence_from_indexed_matches(filtered_matches, pts1, pts2):
    correspondences = []
    for match in filtered_matches:
        point1 = np.array(pts1[match.queryIdx])
        point2 = np.array(pts2[match.trainIdx])
        feature_correspondece = pt.matching.FeatureCorrespondence(
            pt.sfm.Feature(point1), pt.sfm.Feature(point2))
        correspondences.append(feature_correspondece)

    return correspondences

def match_image_pair(img_i_data, img_j_data):

    # create BFMatcher object
    matcher = cv2.BFMatcher()
    matches = matcher.knnMatch(img_i_data["desc"], img_j_data["desc"], k=2)

    # Apply ratio test
    filtered_matches = []
    for m,n in matches:
        if m.distance < 0.8*n.distance:
            filtered_matches.append(m)

    print('Number of putative matches: {}'.format(len(filtered_matches)))

    if len(filtered_matches) < min_num_inlier_matches:
        print('Number of putative matches too low!')
        return False, None

    correspondences = correspondence_from_indexed_matches(
        filtered_matches, img_i_data["kpts"], img_j_data["kpts"])

    options = pt.sfm.EstimateTwoViewInfoOptions()
    options.ransac_type = pt.sfm.RansacType(0) # pt.sfm.RansacType(1): prosac, pt.sfm.RansacType(2): lmed
    options.use_lo = True # Local Optimization Ransac
    options.use_mle = True 

    success, twoview_info, inlier_indices = pt.sfm.EstimateTwoViewInfo(
        options, prior, prior, correspondences)

    print('Only {} matches survived after geometric verification'.format(len(inlier_indices)))
    if len(inlier_indices) < min_num_inlier_matches:
        print('Number of putative matches after geometric verification are too low!')
        return False, None

    else:
        verified_matches = []
        for i in range(len(inlier_indices)):
            verified_matches.append(filtered_matches[inlier_indices[i]])

        correspondences = correspondence_from_indexed_matches(
            verified_matches, img_i_data["kpts"], img_j_data["kpts"])
        twoview_info.num_verified_matches = len(verified_matches)

        for i in range(len(verified_matches)):
            track_builder.AddFeatureCorrespondence(
                img_i_data["view_id"], correspondences[i].feature1, 
                img_j_data["view_id"], correspondences[i].feature2)
        
        return True, twoview_info

def extract_features(img, featuretype):
    if featuretype == 'akaze':
        feature = cv2.AKAZE_create(cv2.AKAZE_DESCRIPTOR_KAZE_UPRIGHT, 0, 3, 1e-3)
    elif featuretype == 'sift':
        feature = cv2.SIFT_create(3000)
    
    kpts, desc = feature.detectAndCompute(img, None)

    return [p.pt for p in kpts], desc

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Argument parser for sfm pipeline')
    parser.add_argument('--path_fountain_dataset', type=str, required=True)
    parser.add_argument('--feature', type=str, default='akaze',
                    help='feature descriptor type: sift or akaze')
    parser.add_argument('--reconstruction', type=str, default='global',
                    help='reconstruction type: global, incremental or hybrid')
    parser.add_argument('--img_ext', default='png')
    args = parser.parse_args()
    featuretype = args.feature
    reconstructiontype = args.reconstruction

    view_graph = pt.sfm.ViewGraph()
    recon = pt.sfm.Reconstruction()
    track_builder = pt.sfm.TrackBuilder(3, 30)

    prior = pt.sfm.CameraIntrinsicsPrior()
    prior.focal_length.value = [2759.48]
    prior.aspect_ratio.value = [2764.16/2759.48]
    prior.principal_point.value = [1520.69, 1006.81]
    prior.image_width = 3072
    prior.image_height = 2048
    prior.skew.value = [0]
    prior.camera_intrinsics_model_type = 'PINHOLE'

    # pinhole camera
    camera = pt.sfm.Camera()
    camera.SetFromCameraIntrinsicsPriors(prior)

    # opencv extraction of features from images
    img_path = args.path_fountain_dataset
    images_files = glob.glob(os.path.join(img_path,'*.'+args.img_ext))
    image_names = [os.path.splitext(os.path.split(f)[1])[0] for f in images_files]
    
    print('{} images have been found'.format(len(images_files)))
    print('Image files: {}'.format(images_files))

    # add views and extract feature
    img_data = {}
    for idx, image_name in enumerate(image_names):
        img_name_ext = image_name+"."+args.img_ext
        vid = recon.AddView(img_name_ext, 0, idx)
        v = recon.MutableView(vid)
        v.SetCameraIntrinsicsPrior(prior)
        image = cv2.imread(os.path.join(img_path,img_name_ext))
        kpts, desc = extract_features(image, featuretype)
        img_data[img_name_ext] = {"view_id": vid, "kpts": kpts, "desc": desc}
    
    # make sure the intrinsics are all initialized fromt he priors
    pt.sfm.SetCameraIntrinsicsFromPriors(recon) 

    view_ids = sorted(recon.ViewIds())
    for i in range(len(view_ids)):
        img_i_name = recon.View(view_ids[i]).Name()

        for j in range(i+1, len(view_ids)):
            img_j_name = recon.View(view_ids[j]).Name()
            
            success, twoview_info = match_image_pair( 
                img_data[img_i_name], img_data[img_j_name])
            
            if success == True:
                view_id1 = recon.ViewIdFromName(img_i_name)
                view_id2 = recon.ViewIdFromName(img_j_name)
                view_graph.AddEdge(view_id1, view_id2, twoview_info)
                print("Match between image {} and image {}. ".format(img_i_name, img_j_name))
            else:
                print("No match between image {} and image {}. ".format(img_i_name, img_j_name))
    
    print('{} edges were added to the view graph.'.format(view_graph.NumEdges()))
    track_builder.BuildTracks(recon)
    options = pt.sfm.ReconstructionEstimatorOptions()
    options.num_threads = 7
    options.rotation_filtering_max_difference_degrees = 10.0
    options.bundle_adjustment_robust_loss_width = 3.0
    options.bundle_adjustment_loss_function_type = pt.sfm.LossFunctionType(1)
    options.subsample_tracks_for_bundle_adjustment = False
    options.filter_relative_translations_with_1dsfm = True

    if reconstructiontype == 'global':
        options.global_position_estimator_type = pt.sfm.GlobalPositionEstimatorType.LIGT
        options.global_rotation_estimator_type = pt.sfm.GlobalRotationEstimatorType.HYBRID  
        reconstruction_estimator = pt.sfm.GlobalReconstructionEstimator(options)
    elif reconstructiontype == 'incremental':
        reconstruction_estimator = pt.sfm.IncrementalReconstructionEstimator(options)
    elif reconstructiontype == 'hybrid':
        reconstruction_estimator = pt.sfm.HybridReconstructionEstimator(options)
    recon_sum = reconstruction_estimator.Estimate(view_graph, recon)

    print('Reconstruction summary message: {}'.format(recon_sum.message))
    pt.io.WritePlyFile(os.path.join(img_path,"fountain.ply"), recon, (255,0,0), 2)
    pt.io.WriteReconstruction(recon, os.path.join(img_path,"fountain.recon"))
