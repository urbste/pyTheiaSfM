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
import natsort, time
import kornia
import yaml
from yaml.loader import SafeLoader
from tqdm import tqdm

MIN_INLIER_MATCHES = 30

def colorize_reconstruction(recon, image_path):

    color_for_track_set = set()  # Changed from list to set for faster lookup
    for idx, v_id in enumerate(recon.ViewIds()):
        view = recon.View(v_id)
        if not view.IsEstimated():
            continue

        image = cv2.imread(os.path.join(image_path, view.Name()))
        image_height, image_width, _ = image.shape

        for t_id in view.TrackIds():
            if not recon.Track(t_id).IsEstimated() or t_id in color_for_track_set:
                continue
            pt_in_img = view.GetFeature(t_id).point
            xy_int = [int(pt_in_img[0]), int(pt_in_img[1])]
            if not (0 <= xy_int[0] < image_width and 0 <= xy_int[1] < image_height):
                continue

            img_clr = image[xy_int[1], xy_int[0], :]
            recon.MutableTrack(t_id).SetColor(img_clr)
            color_for_track_set.add(t_id)  # Use add for a set

# create correspondences of keypoints locations from indexed feature matches
def correspondence_from_indexed_matches(match_indices, pts1, pts2):
    num_matches = match_indices.shape[0]
    correspondences = [
         pt.matching.FeatureCorrespondence(
            pt.sfm.Feature(pts1[match_indices[m,0],:]), 
            pt.sfm.Feature(pts2[match_indices[m,1],:])) for m in range(num_matches)]

    return correspondences

def draw_keypoints(img, kpts):
    img_ = np.ascontiguousarray(img.astype(np.float32))
    for id, pt in enumerate(kpts):
        p = (int(round(pt[0])), int(round(pt[1])))
        cv2.circle(img_, p, 2, (255,0,0))

    return img_

########## some utility functions ##########
def draw_float_matches(img1, img2, inlier_corres, si, sj):
    img1_ = np.ascontiguousarray(img1.squeeze(0).astype(np.float32))
    img2_ = np.ascontiguousarray(img2.squeeze(0).astype(np.float32))
    concat = (np.concatenate([img1_,img2_], 1) * 255).astype(np.uint8)

    for corr in inlier_corres:
        feat1 = corr.feature1.point / si
        feat2 = corr.feature2.point / sj
        p1 = (int(round(feat1[0])), int(round(feat1[1])))
        p2 = (int(round(feat2[0])), int(round(feat2[1])))
        clr = (0,255,0)
        cv2.line(concat, p1, (p2[0]+img2_.shape[1],p2[1]), clr, 1, lineType=16)
    return concat

def load_img_tensor(img_path, inf_shape_max, device, dtype):
    image = kornia.io.load_image(img_path, kornia.io.ImageLoadType.RGB32)[None, ...]

    original_img_size = image.shape[3:1:-1]
    scaler = inf_shape_max / max(original_img_size)
    inf_shape_wh = (int(original_img_size[1] * scaler), 
                    int(original_img_size[0] * scaler))
    image = kornia.geometry.resize(image, inf_shape_wh, antialias=True)

    return image.to(device).to(dtype), original_img_size

def extract_disk_features(image, extractor, debug=False):
    with torch.no_grad():
        features = extractor(image, n = 5000, 
            score_threshold = 0.0, 
            window_size = 1, 
            pad_if_not_divisible=True)

    keypoints = [f.keypoints for f in features]
    scores = [f.detection_scores for f in features]
    descriptors = [f.descriptors for f in features]
    del features

    keypoints = torch.stack(keypoints, 0)
    scores = torch.stack(scores, 0)
    descriptors = torch.stack(descriptors, 0)

    if debug:
        img_kpts = draw_keypoints(image.permute(0,2,3,1).cpu().numpy().squeeze(0),
            keypoints.cpu().numpy().squeeze(0))
        cv2.imshow("Detected DISK features", img_kpts)
        cv2.waitKey(1)

    return {'lafs': kornia.feature.laf_from_center_scale_ori(keypoints).to(image),
            'keypoint_scores': scores.to(image),
            'descriptors': descriptors.to(image)}

def extract_sift_features(image, extractor, debug=False):
    with torch.no_grad():
        lafs, scores, descriptors = extractor(kornia.color.rgb_to_grayscale(image))
    if debug:
        keypoints = kornia.feature.get_laf_center(lafs)
        img_kpts = draw_keypoints(image.permute(0,2,3,1).cpu().numpy().squeeze(0),
            keypoints.cpu().numpy().squeeze(0))
        cv2.imshow("Detected DISK features", img_kpts)
        cv2.waitKey(1)

    return {'lafs': lafs.to(image),
            'descriptors': descriptors.to(image)}


def estimate_two_view_geometry(correspondences, cam_prior_i, cam_prior_j):

    options = pt.sfm.EstimateTwoViewInfoOptions()
    # pt.sfm.RansacType(1): prosac, pt.sfm.RansacType(2): lmed
    options.ransac_type = pt.sfm.RansacType(0) 
    options.use_lo = True # Local Optimization
    options.use_mle = True 
    options.max_sampson_error_pixels = 1.5
    options.expected_ransac_confidence = 0.999
    options.max_ransac_iterations = 5000
    options.min_ransac_iterations = 1000

    success, two_view_info, inliers = pt.sfm.EstimateTwoViewInfo(
        options, cam_prior_i, cam_prior_j, correspondences)

    return success, two_view_info, inliers

def match_image_pair(
    image_i, image_j,
    f_i, f_j, 
    scale_i, scale_j,
    matcher, 
    cam_prior_i, cam_prior_j, min_conf, device):

    with torch.no_grad():

        #start = time.perf_counter()
        scores, matches = matcher(
            lafs1 = torch.from_numpy(f_i["lafs"]).to(device),
            lafs2 = torch.from_numpy(f_j["lafs"]).to(device),
            desc1 = torch.from_numpy(f_i["descriptors"]).to(device).squeeze(0),
            desc2 = torch.from_numpy(f_j["descriptors"]).to(device).squeeze(0),
            hw1 = torch.tensor(image_i.shape[2:]).to(device),
            hw2 = torch.tensor(image_j.shape[2:]).to(device))
        #end = time.perf_counter()
        #print("Matching took {}s".format((end-start)))
        torch.cuda.empty_cache()

    # create match indices
    if matches.shape[0] < MIN_INLIER_MATCHES:
        print('Number of putative matches too low!')
        return False, None, None

    kpts0 = kornia.feature.get_laf_center(f_i["lafs"])[0] * scale_i
    kpts1 = kornia.feature.get_laf_center(f_j["lafs"])[0] * scale_j
    
    correspondences = correspondence_from_indexed_matches(matches, kpts0, kpts1)

    success, two_view_info, inliers = estimate_two_view_geometry(
        correspondences, cam_prior_i, cam_prior_j)

    inlier_correspondences = [correspondences[inliers[i]] for i in range(len(inliers))]
    
    if args.debug:
        img = draw_float_matches(image_i.permute(0,2,3,1).cpu().numpy(),
                        image_j.permute(0,2,3,1).cpu().numpy(),
                        inlier_correspondences, scale_i, scale_j)
        cv2.imshow("matches", img)
        cv2.waitKey(1)

    if not success:
        print('TwoViewInfo estimation failed!')
        return False, None, None

    if len(inliers) < MIN_INLIER_MATCHES or not success:
        print('Number of putative matches after geometric verification are too low!')
        return False, None, None
    else:
        return True, two_view_info, inlier_correspondences


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Argument parser for sfm pipeline')
    parser.add_argument('--image_path', 
                        default="", 
                        type=str)
    parser.add_argument('--reconstruction_type', type=str, default='incremental', 
                        help='reconstruction type: global, incremental or hybrid')
    parser.add_argument('--img_ext', default='jpg')
    parser.add_argument('--debug', type=int, default=0)
    parser.add_argument('--temporal_match_window', type=int, default=3) # for video sequences, if 0 no temporal window
    parser.add_argument('--place_recog_window_size', type=int, default=5)
    parser.add_argument('--camera_model', type=str, default='PINHOLE', 
                        choices=['PINHOLE', 'PINHOLE_RADIAL_TANGENTIAL', 'DIVISION_UNDISTORTION'])
    parser.add_argument('--shared_intrinsics', action='store_true', default=True, help='same intrinsics for all views')
    parser.add_argument('--approx_fov', type=float, default=-1.0, help='approximate field of view of the camera')
    parser.add_argument('--focal_length', type=float, default=-1.0, help='focal length of the camera')
    parser.add_argument('--feature_type', type=str, default="disk", choices=["sift", "disk"])
    parser.add_argument('--export_nerfstudio_json', type=str, default="", help="Export nerfstudio data.")
    parser.add_argument('--export_sfm_depth', action="store_true", default=True)
    parser.add_argument('--average_scene_depth_mm', default=1000, help="Specify an average scene depth if you know it.")
    args = parser.parse_args()
    reconstructiontype = args.reconstruction_type

    view_graph = pt.sfm.ViewGraph()
    recon = pt.sfm.Reconstruction()
    track_builder = pt.sfm.TrackBuilder(3, 30)

    device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
    dtype = torch.float32
    # if device == "cuda":
    #     if torch.cuda.is_bf16_supported():
    #         dtype = torch.float16

    # load models (LOFTR and COSPLACE)
    place_recog = torch.hub.load("gmberton/cosplace", 
        "get_trained_model", backbone="ResNet18", fc_output_dim=128)
    place_recog.eval().to(device).to(dtype)

    if args.feature_type == "disk":
        extractor = kornia.feature.DISK.from_pretrained("depth").eval().to(device).to(dtype)
    elif args.feature_type == "sift":
        extractor = kornia.feature.SIFTFeature(num_features=2000, upright=False, device=device)

    matcher = kornia.feature.LightGlueMatcher(feature_name=args.feature_type).to(device).to(dtype)

    images_files = glob.glob(os.path.join(args.image_path,'*.'+args.img_ext))
    image_names = natsort.natsorted([os.path.split(f)[1] for f in images_files])

    # check the size of one image
    inf_shape_max = 1920
    img_data = {"names": [], "data": [], 
                "original_shapes": [], "inference_shapes": [],
                "feat_vec": [], "feats" : []}

    # load image, extract features and place recognition feature vectors
    for idx, image_name in enumerate(image_names):
        vid = recon.AddView(image_name, 0 if args.shared_intrinsics else idx, idx)
        v = recon.MutableView(vid)
        
        ## Load image
        path = os.path.join(args.image_path,image_name)
        img_tensor_rgb, original_img_shape = load_img_tensor(
            path, inf_shape_max, device, dtype)

        ## initialize intrinsics
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

        # extract features
        if args.feature_type == "disk":
            features = extract_disk_features(img_tensor_rgb, extractor, args.debug)
        elif args.feature_type == "sift":
            features = extract_sift_features(img_tensor_rgb, extractor, args.debug)

        img_data["names"].append(image_name)
        img_data["data"].append(img_tensor_rgb)
        img_data["original_shapes"].append(original_img_shape)
        img_data["inference_shapes"].append(img_tensor_rgb.shape[4:1:-1])
        img_data["feats"].append({
            "lafs" : features["lafs"].cpu().numpy(),
            "descriptors" : features["descriptors"].cpu().numpy()})

        # extract place features
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
        scale_i = np.array(img_data["original_shapes"][vi]) / \
            np.array(img_data["inference_shapes"][vi])
        scale_j = np.array(img_data["original_shapes"][vj]) / \
            np.array(img_data["inference_shapes"][vj])

        success, tvi, inlier_correspondences = match_image_pair(
            img_data["data"][vi], img_data["data"][vj],
            img_data["feats"][vi], img_data["feats"][vj],
            scale_i, scale_j,
            matcher, prior, prior, 0.8, device)


        if success:
            view_id1 = recon.ViewIdFromName(img_data["names"][vi])
            view_id2 = recon.ViewIdFromName(img_data["names"][vj])
            view_graph.AddEdge(view_id1, view_id2, tvi)
            for c in inlier_correspondences:
                track_builder.AddFeatureCorrespondence(
                    vi, c.feature1, 
                    vj, c.feature2)
            if args.shared_intrinsics:
                med_focal_length.append(tvi.focal_length_1)
                med_focal_length.append(tvi.focal_length_2)

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
    pt.sfm.SetOutlierTracksToUnestimated(set(recon.TrackIds()), 0.002 * max(original_img_shape), 1.0, recon)
    opts = pt.sfm.BundleAdjustmentOptions()
    opts.robust_loss_width = 0.001 * max(original_img_shape)
    opts.verbose = True
    opts.loss_function_type = pt.sfm.LossFunctionType.HUBER
    opts.use_homogeneous_point_parametrization = True
    opts.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.FOCAL_LENGTH
    ba_sum = pt.sfm.BundleAdjustReconstruction(opts, recon)

    opts.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.PRINCIPAL_POINTS
    ba_sum = pt.sfm.BundleAdjustReconstruction(opts, recon)

    print('Reconstruction summary message: {}'.format(recon_sum.message))
    pt.io.WritePlyFile(os.path.join(args.image_path,"recon.ply"), recon, (255,0,0), 2)
    pt.io.WriteReconstruction(recon, os.path.join(args.image_path,"recon.recon"))
    print("Reprojection error after final BA: {}".format(reprojection_error(recon)))  

    if args.shared_intrinsics:
        print("Calibrated Focal length: {}".format(recon.View(0).Camera().FocalLength()))
    
    colorize_reconstruction(recon, args.image_path)

    if args.export_nerfstudio_json != "":
        # make colmap directory structure
        colmap_sparse = os.path.join(args.image_path, "../", "sparse", "0")
        os.makedirs(colmap_sparse, exist_ok=True)
        pt.io.WriteColmapFiles(recon, colmap_sparse)
        
        res = pt.io.WriteNerfStudio(args.image_path, recon, 16, args.export_nerfstudio_json)
        if not res:
            print("Unable to write nerfstudio files.")

        if args.export_sfm_depth:
            import json
            from utils import median_scene_depth_in_view, find_img_name_idx_in_transforms
            with open(args.export_nerfstudio_json, "r") as f:
                transforms = json.load(f)

            depth_img_path = os.path.join(args.image_path, "sfm_depth")
            os.makedirs(depth_img_path, exist_ok=True)

            med_scene_depth = median_scene_depth_in_view(recon, 0)
            scale_rec_to_mm = args.average_scene_depth_mm / med_scene_depth
            
            # now create depth images
            for vid in recon.ViewIds():
                W, H = original_img_shape
                sfm_depth = np.zeros((H, W), dtype=np.float32)

                view_names = []
                for vid in recon.ViewIds():
                    view = recon.View(vid)
                    if not view.IsEstimated():
                        continue

                    for tid in view.TrackIds():
                        if not recon.Track(tid).IsEstimated():
                            continue
                        depth, rep_pt2 = view.Camera().ProjectPoint(recon.Track(tid).Point())

                        u, v = int(rep_pt2[0]), int(rep_pt2[1])

                        depth *= scale_rec_to_mm
                        if 0 <= v < W and 0 <= u <= H and depth > 0 and depth > 0.001 and depth < 10000:
                            sfm_depth[v, u] = depth
                    depth_img_name = view.Name().split(".")[0] + ".png"
                    depth_file_path = os.path.join(depth_img_path, depth_img_name)
                    cv2.imwrite(depth_file_path, sfm_depth.astype(np.uint16))

                    frame_idx_in_json = find_img_name_idx_in_transforms(transforms, view.Name())
                    transforms["frames"][frame_idx_in_json]["depth_file_path"] = depth_file_path
                    transform_matrix = np.array(transforms["frames"][frame_idx_in_json]["transform_matrix"])
                    transform_matrix[:3,3] *= scale_rec_to_mm
                    transforms["frames"][frame_idx_in_json]["transform_matrix"] = transform_matrix.tolist()
                    
            with open(args.export_nerfstudio_json, "w") as f:
                json.dump(transforms, f, indent=4)

    cv2.destroyAllWindows()
