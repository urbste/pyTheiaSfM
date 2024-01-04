# Steffen Urban, 2023

# This simple example demonstrates the use of pytheia on the Aqualoc dataset using:
# - LOFTR for feature matching
# - Cosplace for place recognition
# - GraphMatch for two view pairing
# - Aqualoc for geometric verification
# - Alignment to gravity
# - priors on the z axis using pressure sensor data

import numpy as np
import cv2, os, glob, argparse
import pytheia as pt
import torch
import natsort
from kornia.feature import LoFTR
import kornia as K
import kornia.feature as KF
import yaml
from yaml.loader import SafeLoader
from tqdm import tqdm


def find_approx_focal_length_through_inlier_cnt():
    # simple heuristic to find the approximate focal length
    # based on the number of inliers fundamental matrix estimation
    # we assume that the focal length is the same for all images
    # and that the images are taken with the same camera
    
    pt.sfm.EstimateTwoViewInfoOptions()

    


########## some utility functions ##########
def draw_float_matches(img1, img2, kpts1, kpts2, inliers):
    img1_ = np.ascontiguousarray(img1.squeeze(0).astype(np.float32))
    img2_ = np.ascontiguousarray(img2.squeeze(0).astype(np.float32))
    concat = (np.concatenate([img1_,img2_], 1) * 255).astype(np.uint8)
    if not inliers:
        inliers = list(range(len(kpts1)))
    for id, (pt1, pt2) in enumerate(zip(kpts1, kpts2)):
        p1 = (int(round(pt1[0])), int(round(pt1[1])))
        p2 = (int(round(pt2[0])), int(round(pt2[1])))
        if id in inliers:
            clr = (0,255,0)
            cv2.line(concat, p1, (p2[0]+img2_.shape[1],p2[1]), clr, 1, lineType=16)
    return concat

def load_img_tensor(img_path, inf_shape_wh, device, dtype):
    image = K.io.load_image(img_path, K.io.ImageLoadType.RGB32)[None, ...]

    original_img_size = image.shape[3:1:-1]
    image = K.geometry.resize(image, inf_shape_wh[::-1], antialias=True)

    return image.to(device).to(dtype), original_img_size

########## loftr feature matching ##########
def match_loftr(matcher, img_i, img_j, min_conf):
    with torch.no_grad():
        input_dict = {
            "image0": K.color.rgb_to_grayscale(img_i),  
            "image1": K.color.rgb_to_grayscale(img_j),
        }

        correspondences = matcher(input_dict)

        kp_i = (correspondences["keypoints0"]).cpu().numpy()
        kp_j = (correspondences["keypoints1"]).cpu().numpy()
        conf = correspondences["confidence"].cpu().numpy()

    sorted_idx = np.argsort(conf)[::-1]
    
    conf_sorted = conf[sorted_idx]
    kpi_sorted = kp_i[sorted_idx,:]
    kpj_sorted = kp_j[sorted_idx,:]

    conf_thresh = np.where(conf_sorted > min_conf)[0]
    conf_sorted = conf_sorted[conf_thresh]
    kpi_sorted = kpi_sorted[conf_thresh,:]
    kpj_sorted = kpj_sorted[conf_thresh,:]

    return kpi_sorted, kpj_sorted, conf_thresh

########## two view estimation ##########
def estimate_two_view_geometry(kp_i, kp_j, cam_prior_i, cam_prior_j):

    correspondences = []
    for idx in range(kp_i.shape[0]):
        correspondences.append(pt.matching.FeatureCorrespondence(
            pt.sfm.Feature(kp_i[idx]), pt.sfm.Feature(kp_j[idx])))

    options = pt.sfm.EstimateTwoViewInfoOptions()
    # pt.sfm.RansacType(1): prosac, pt.sfm.RansacType(2): lmed
    options.ransac_type = pt.sfm.RansacType(2) 
    options.use_lo = False # Local Optimization
    options.use_mle = True 
    options.max_sampson_error_pixels = 1.0
    options.expected_ransac_confidence = 0.999
    options.max_ransac_iterations = 5000
    options.min_ransac_iterations = 100

    success, two_view_info, inliers = pt.sfm.EstimateTwoViewInfo(
        options, cam_prior_i, cam_prior_j, correspondences)
    
    print("two_view_info: {}".format(two_view_info.focal_length_1))
    print("two_view_info: {}".format(two_view_info.focal_length_2))
    return success, two_view_info, inliers, correspondences

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Argument parser for sfm pipeline')
    parser.add_argument('--image_path', 
                        default="/home/steffen/Data/Wildschwein/images/", 
                        type=str)
    parser.add_argument('--reconstruction_type', type=str, default='incremental', 
                        help='reconstruction type: global, incremental or hybrid')
    parser.add_argument('--img_ext', default='jpg')
    parser.add_argument('--debug', type=int, default=0)
    parser.add_argument('--temporal_match_window', type=int, default=3) # for video sequences, if 0 no temporal window
    parser.add_argument('--place_recog_window_size', type=int, default=5)
    parser.add_argument('--camera_model', type=str, default='PINHOLE', 
                        choices=['PINHOLE', 'PINHOLE_RADIAL_TANGENTIAL', 'DIVISION_UNDISTORTION'])
    parser.add_argument('--shared_intrinsics', action='store_true', default=False, help='same intrinsics for all views')
    parser.add_argument('--approx_fov', type=float, default=-1.0, help='approximate field of view of the camera')
    parser.add_argument('--focal_length', type=float, default=-1.0, help='focal length of the camera')
    args = parser.parse_args()
    reconstructiontype = args.reconstruction_type

    view_graph = pt.sfm.ViewGraph()
    recon = pt.sfm.Reconstruction()
    track_builder = pt.sfm.TrackBuilder(3, 30)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    dtype = torch.float32
    if device == "cuda":
        if torch.cuda.is_bf16_supported():
            dtype = torch.float16

    # load models (LOFTR and COSPLACE)
    place_recog = torch.hub.load("gmberton/cosplace", 
        "get_trained_model", backbone="ResNet18", fc_output_dim=128)
    place_recog.to(device).to(dtype)
    matcher = LoFTR(pretrained="outdoor").to(device).to(dtype)
    matcher.eval()
    place_recog.eval()
    

    images_files = glob.glob(os.path.join(args.image_path,'*.'+args.img_ext))
    image_names = natsort.natsorted([os.path.splitext(os.path.split(f)[1])[0] for f in images_files])

    # check the size of one image
    loftr_inf_shape = (640//2, 512//2)
    img_data = {"names": [], "data": [], 
                "original_shapes": [], "feat_vec": []}

    # match all pairs of images
    for idx, image_name in enumerate(image_names):
        vid = recon.AddView(image_name, 0 if args.shared_intrinsics else idx, idx)
        v = recon.MutableView(vid)
        path = os.path.join(args.image_path,image_name+"."+args.img_ext)
        img_tensor_rgb, original_img_shape = load_img_tensor(
            path, loftr_inf_shape, device, dtype)

        prior = pt.sfm.CameraIntrinsicsPrior()
        prior.image_width = original_img_shape[0]
        prior.image_height = original_img_shape[1]

        if args.focal_length > 0:
            prior.focal_length.value = [args.focal_length]
        elif args.approx_fov > 0:
            fov_est = args.approx_fov * np.pi / 180.0
            focal_ratio = 0.5 / np.tan(fov_est * 0.5)
            prior.focal_length.value = [focal_ratio * max(original_img_shape)]
        else:
            print("WARNING: You did not set any focal length or FOV. Will try using fundamental matrix to estimate the focal length.")
        print("Initializing focal length to {}".format(prior.focal_length.value[0]))
              
        prior.principal_point.value = [original_img_shape[0]/2, original_img_shape[1]/2]
        prior.aspect_ratio.value = [1.0]
        prior.skew.value = [0]
        prior.camera_intrinsics_model_type = args.camera_model

        v.SetCameraIntrinsicsPrior(prior)

        img_data["names"].append(image_name)
        img_data["data"].append(img_tensor_rgb)
        img_data["original_shapes"].append(original_img_shape)

        if args.place_recog_window_size > 0:
            with torch.no_grad():
                feature = place_recog(img_tensor_rgb).cpu().numpy().astype(np.float32)
            img_data["feat_vec"].append(feature)
    
    num_images = len(img_data["names"])

    pairs_to_match = []
    # create match pairs based on visual similarity (cosplace descriptors)
    if args.place_recog_window_size > 0:
        image_name_to_match = pt.matching.GraphMatch(img_data["names"], 
                                np.asarray(img_data["feat_vec"]).squeeze(1), 
                                args.place_recog_window_size)
        pairs_to_match = [(img_data["names"].index(pair[0]),
                        img_data["names"].index(pair[1])) for pair in image_name_to_match]

    num_visual_similar = len(pairs_to_match)
    print("{} pairs with visually similarity found.".format(num_visual_similar))

    # create temporal match pairs (if we have a video sequence)
    for vid in range(0, num_images):
        for vjd in range(vid+1, vid+1+args.temporal_match_window):
            if vjd >= num_images:
                break
            pair = (vid, vjd)
            if pair in pairs_to_match:
                continue
            pairs_to_match.append(pair)    
    print("Added {} temporal matching pairs.".format(
        len(pairs_to_match)-num_visual_similar))

    print("Will match {} pairs intead of N*N {} pairs.".format(
        len(pairs_to_match), num_images**2/2))
    
    # iterate image buffer and match each view with each other   
    med_focal_length = []
    for match_pair in tqdm(pairs_to_match):
        vi, vj = match_pair
        scale_i = np.array(img_data["original_shapes"][vi]) / np.array(loftr_inf_shape)
        scale_j = np.array(img_data["original_shapes"][vj]) / np.array(loftr_inf_shape)
        kp_i, kp_j, conf = match_loftr(matcher, 
            img_data["data"][vi], img_data["data"][vj], 0.8)
        # estimate two view info
        if len(kp_i) < 200:
            continue
        kp_i, kp_j = kp_i * scale_i, kp_j * scale_j
        success, tvi, inliers, correspondences = estimate_two_view_geometry(
                kp_i, kp_j, prior, prior)
        med_focal_length.append(tvi.focal_length_1)
        med_focal_length.append(tvi.focal_length_2)
        if success == True and len(inliers) > 100:
            if args.debug:
                img = draw_float_matches(img_data["data"][vi].permute(0,2,3,1).cpu().numpy(),
                                img_data["data"][vj].permute(0,2,3,1).cpu().numpy(),
                                kp_i/scale_i, kp_j/scale_j, inliers)
                cv2.imshow("matches", img.astype(np.uint8))
                cv2.waitKey(1)
            # add edge in view graph
            view_id1 = recon.ViewIdFromName(img_data["names"][vi])
            view_id2 = recon.ViewIdFromName(img_data["names"][vj])
            view_graph.AddEdge(view_id1, view_id2, tvi)
            # add tracks
            for i in range(len(inliers)):
                track_builder.AddFeatureCorrespondence(
                    view_id1, correspondences[inliers[i]].feature1, 
                    view_id2, correspondences[inliers[i]].feature2)
            if args.debug:
                print("Matched {} and {} with {} inliers".format(view_id1, view_id2, len(inliers)))

    if args.shared_intrinsics:
        median_focal_length = np.median(med_focal_length)
        print("Median focal length: {}".format(median_focal_length))            
        prior.focal_length.value = [median_focal_length]    

        for vid in recon.ViewIds():
            v = recon.MutableView(vid)
            v.SetCameraIntrinsicsPrior(prior)
    
    # make sure the intrinsics are all initialized fromt he priors
    pt.sfm.SetCameraIntrinsicsFromPriors(recon) 

    print('{} edges were added to the view graph.'.format(view_graph.NumEdges()))
    track_builder.BuildTracks(recon)
    options = pt.sfm.ReconstructionEstimatorOptions()
    options.num_threads = 10
    options.rotation_filtering_max_difference_degrees = 15.0
    options.bundle_adjustment_robust_loss_width = 0.004 * max(original_img_shape)
    options.bundle_adjustment_loss_function_type = pt.sfm.LossFunctionType(1)
    options.subsample_tracks_for_bundle_adjustment = False
    options.filter_relative_translations_with_1dsfm = True
    options.min_triangulation_angle_degrees = 2.0
    options.num_retriangulation_iterations = 2
    options.triangulation_method = pt.sfm.TriangulationMethodType(0)
    options.track_parametrization_type = pt.sfm.TrackParametrizationType.XYZW_MANIFOLD
    options.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.FOCAL_LENGTH
    options.full_bundle_adjustment_growth_percent = 25.0
    options.track_selection_image_grid_cell_size_pixels = prior.image_height // 10

    if reconstructiontype == 'global':
        options.global_position_estimator_type = pt.sfm.GlobalPositionEstimatorType.LEAST_UNSQUARED_DEVIATION # LEAST_UNSQUARED_DEVIATION, LINEAR_TRIPLET, NONLINEAR, LIGT
        options.global_rotation_estimator_type = pt.sfm.GlobalRotationEstimatorType.HYBRID
        reconstruction_estimator = pt.sfm.GlobalReconstructionEstimator(options)
    elif reconstructiontype == 'incremental':
        reconstruction_estimator = pt.sfm.IncrementalReconstructionEstimator(options)
    elif reconstructiontype == 'hybrid':
        reconstruction_estimator = pt.sfm.HybridReconstructionEstimator(options)
    recon_sum = reconstruction_estimator.Estimate(view_graph, recon)

    from utils import reprojection_error
    print('Reconstruction summary message: {}'.format(recon_sum.message))
    print("Reprojection error before final BA: {}".format(reprojection_error(recon)))  
    pt.io.WritePlyFile(os.path.join(args.image_path,"recon_pre.ply"), recon, (255,0,0), 2)
    pt.io.WriteReconstruction(recon, os.path.join(args.image_path,"recon_pre.recon"))

    # perform bundle adjustment and use gravity priors as well
    pt.sfm.SetOutlierTracksToUnestimated(set(recon.TrackIds()), 0.003 * max(original_img_shape), 1.0, recon)
    opts = pt.sfm.BundleAdjustmentOptions()
    opts.robust_loss_width = 0.001 * max(original_img_shape)
    opts.verbose = True
    opts.loss_function_type = pt.sfm.LossFunctionType.HUBER
    opts.use_homogeneous_point_parametrization = True
    opts.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.FOCAL_LENGTH_RADIAL_DISTORTION
    ba_sum = pt.sfm.BundleAdjustReconstruction(opts, recon)

    print('Reconstruction summary message: {}'.format(recon_sum.message))
    pt.io.WritePlyFile(os.path.join(args.image_path,"recon.ply"), recon, (255,0,0), 2)
    pt.io.WriteReconstruction(recon, os.path.join(args.image_path,"recon.recon"))
    print("Reprojection error after final BA: {}".format(reprojection_error(recon)))  

    if args.shared_intrinsics:
        print("Calibrated Focal length: {}".format(recon.View(0).Camera().FocalLength()))

    cv2.destroyAllWindows()
