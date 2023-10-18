# Steffen Urban, 2023

# This simple example demonstrates the use of pytheia to reconstruct the 
# Fountain dataset scene
# In this case, we are using deep feature detection and matching 
# based on the DISK detector and the Lightglue matcher from Kornia


import numpy as np
import os, glob, argparse, time, natsort, cv2
import pytheia as pt
import torch, kornia
from image_utils import load_image
from tqdm import tqdm
from utils import reprojection_error, plot_loftr_matches

min_num_inlier_matches = 50

# create correspondences of keypoints locations from indexed feature matches
def correspondence_from_indexed_matches(pts1, pts2):
    correspondences = [
         pt.matching.FeatureCorrespondence(
            pt.sfm.Feature(pts1[m,:]), 
            pt.sfm.Feature(pts2[m,:])) for m in range(pts1.shape[0])]

    return correspondences

def match_image_pair(img_i_data, img_j_data, matcher, min_conf, cam_prior0, cam_prior1, match_size, dtype, device):

    with torch.no_grad():
        data = {'image0': img_i_data["image"], 
                'image1': img_j_data["image"]}
        
        img1 = kornia.geometry.resize(img_i_data["image"], match_size, antialias=True)
        img2 = kornia.geometry.resize(img_j_data["image"], match_size, antialias=True)

        scales1 = [img1.shape[3] / img_i_data["image"].shape[3], img1.shape[2] / img_i_data["image"].shape[2]]
        scales2 = [img2.shape[3] / img_j_data["image"].shape[3], img2.shape[2] / img_j_data["image"].shape[2]]

        input_dict = {
            "image0": kornia.color.rgb_to_grayscale(img1).to(dtype).to(device),  # LofTR works on grayscale images only
            "image1": kornia.color.rgb_to_grayscale(img2).to(dtype).to(device),
        }

        correspondences = matcher(input_dict)

        kpts0 = correspondences["keypoints0"].cpu().numpy() / scales1
        kpts1 = correspondences["keypoints1"].cpu().numpy() / scales2

        conf = correspondences["confidence"].cpu().numpy()

        sorted_idx = np.argsort(conf)[::-1]
        
        c_sorted = conf[sorted_idx]
        kp1_s = kpts0[sorted_idx,:]
        kp2_s = kpts1[sorted_idx,:]

        conf_thresh = np.where(c_sorted > min_conf)[0]
        c_sorted = c_sorted[conf_thresh]
        kp1_s = kp1_s[conf_thresh,:]
        kp2_s = kp2_s[conf_thresh,:]

    # # plot matches
    # img0 = img_i_data["image"].squeeze(0).permute(1,2,0).cpu().numpy()
    # img1 = img_j_data["image"].squeeze(0).permute(1,2,0).cpu().numpy()
    # img_match = plot_loftr_matches(kpts0, img0, kpts1, img1)
    # cv2.imshow("matches", img_match)
    # cv2.waitKey(1)

    correspondences = correspondence_from_indexed_matches(kpts0, kpts1)

    options = pt.sfm.EstimateTwoViewInfoOptions()
    options.ransac_type = pt.sfm.RansacType(1) # prosac sampler as sorted matches
    options.use_lo = True # Local Optimization Ransac
    options.use_mle = True 
    options.max_sampson_error_pixels = 1.0

    success, twoview_info, inlier_indices = pt.sfm.EstimateTwoViewInfo(
        options, cam_prior0, cam_prior1, correspondences)

    if not success:
        return False, None

    if len(inlier_indices) < min_num_inlier_matches or not success:
        return False, None
    else:

        twoview_info.num_verified_matches = len(inlier_indices)

        for i in range(len(inlier_indices)):
            corr = correspondences[inlier_indices[i]]
            track_builder.AddFeatureCorrespondence(
                img_i_data["view_id"], corr.feature1, 
                img_j_data["view_id"], corr.feature2)
        
        return True, twoview_info

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Argument parser for sfm pipeline')
    parser.add_argument('--path_poster_images', type=str, default="/home/steffen/Data/nerfstudio/poster/images")
    parser.add_argument('--output_path', type=str, default="/home/steffen/Data/nerfstudio/poster/pytheia/")
    parser.add_argument('--reconstruction', type=str, default='incremental',
                    help='reconstruction type: global, incremental or hybrid')
    parser.add_argument('--img_ext', default='png')
    parser.add_argument('--device', default="cuda")
    parser.add_argument('--use_fp16', default=True)
    parser.add_argument('--temporal_match_window', default=6)

    args = parser.parse_args()
    reconstructiontype = args.reconstruction

    os.makedirs(args.output_path, exist_ok=True)

    matcher = kornia.feature.LoFTR(pretrained="indoor")

    device = "cpu"
    dtype = torch.float32
    if torch.cuda.is_available() and args.device == "cuda":
        device = args.device
        matcher = matcher.cuda()
        if args.use_fp16 and torch.cuda.is_bf16_supported():
            matcher = matcher.half()
            dtype = torch.float16

    start_rec_time = time.perf_counter()
    view_graph = pt.sfm.ViewGraph()
    recon = pt.sfm.Reconstruction()
    track_builder = pt.sfm.TrackBuilder(4, 30)

    prior = pt.sfm.CameraIntrinsicsPrior()
    prior.focal_length.value = [1147.1526990016812]
    prior.aspect_ratio.value = [1158.8252259026597/1147.1526990016812]
    prior.principal_point.value = [540.0, 960.0]
    prior.image_width = 1080
    prior.image_height = 1920
    prior.skew.value = [0]
    prior.camera_intrinsics_model_type = 'PINHOLE'

    # pinhole camera
    camera = pt.sfm.Camera()
    camera.SetFromCameraIntrinsicsPriors(prior)

    # opencv extraction of features from images
    img_path = args.path_poster_images
    images_files = glob.glob(os.path.join(img_path,'*.'+args.img_ext))
    image_names = natsort.natsorted([os.path.splitext(os.path.split(f)[1])[0] for f in images_files])
    
    print('{} images have been found'.format(len(images_files)))

    # add views and extract features
    img_data = {}
    
    for idx, image_name in enumerate(tqdm(image_names)):
        img_name_ext = image_name+"."+args.img_ext
        vid = recon.AddView(img_name_ext, 0, idx)
        v = recon.MutableView(vid)
        v.SetCameraIntrinsicsPrior(prior)
        # load images to torch and resize to max_edge=1024
        image = kornia.io.load_image(os.path.join(img_path, img_name_ext), 
                                    kornia.io.ImageLoadType.RGB32, device="cpu")[None, ...]

        img_data[img_name_ext] = {
            "view_id": vid, "image" : image}
        
    # make sure the intrinsics are all initialized fromt he priors
    pt.sfm.SetCameraIntrinsicsFromPriors(recon) 
    view_names = list(img_data.keys())

    pairs_to_match = []
    # create temporal match pairs (if we have a video sequence)
    for vid in range(0, len(img_data)):
        for vjd in range(vid+1, vid+1+args.temporal_match_window):
            if vjd >= len(img_data):
                break
            pair = (vid, vjd)
            if pair in pairs_to_match:
                continue
            pairs_to_match.append(pair)    
    print("Added {} temporal matching pairs.".format(
        len(pairs_to_match)))

    # now match the images using lightglue
    view_ids = sorted(recon.ViewIds())
    for match_pair in tqdm(pairs_to_match):
        vi, vj = match_pair

        img_i_name = view_names[vi]
        img_j_name = view_names[vj]
        
        success, twoview_info = match_image_pair( 
            img_data[img_i_name], img_data[img_j_name], 
            matcher, 0.8, 
            prior, prior, 
            (prior.image_height//3, prior.image_width//3), dtype, device)
        
        if success == True:
            view_id1 = recon.ViewIdFromName(img_i_name)
            view_id2 = recon.ViewIdFromName(img_j_name)
            view_graph.AddEdge(view_id1, view_id2, twoview_info)

    print('{} edges were added to the view graph.'.format(view_graph.NumEdges()))
    track_builder.BuildTracks(recon)
    options = pt.sfm.ReconstructionEstimatorOptions()
    options.num_threads = 7
    options.rotation_filtering_max_difference_degrees = 10.0
    options.bundle_adjustment_robust_loss_width = 3.0
    options.bundle_adjustment_loss_function_type = pt.sfm.LossFunctionType(1)
    options.subsample_tracks_for_bundle_adjustment = True
    options.filter_relative_translations_with_1dsfm = True
    options.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.FOCAL_LENGTH_RADIAL_DISTORTION

    if reconstructiontype == 'global':
        options.global_position_estimator_type = pt.sfm.GlobalPositionEstimatorType.LEAST_UNSQUARED_DEVIATION
        options.global_rotation_estimator_type = pt.sfm.GlobalRotationEstimatorType.ROBUST_L1L2  
        reconstruction_estimator = pt.sfm.GlobalReconstructionEstimator(options)
    elif reconstructiontype == 'incremental':
        reconstruction_estimator = pt.sfm.IncrementalReconstructionEstimator(options)
    elif reconstructiontype == 'hybrid':
        reconstruction_estimator = pt.sfm.HybridReconstructionEstimator(options)
    recon_sum = reconstruction_estimator.Estimate(view_graph, recon)

    # clean up the reconstruction
    pre_clean_err = reprojection_error(recon)
    print("Final reprojection error before cleaning: {:.3f}px".format(pre_clean_err))   

    pt.sfm.SetOutlierTracksToUnestimated(set(recon.TrackIds()), 3.0, 0.25, recon)
    post_clean_err = reprojection_error(recon)
    print("Final reprojection error after cleaning: {:.3f}px".format(post_clean_err))   

    ## Do one last bundle adjustment
    opts = pt.sfm.BundleAdjustmentOptions()
    opts.robust_loss_width = post_clean_err*1.2
    opts.loss_function_type = pt.sfm.LossFunctionType.HUBER
    opts.use_homogeneous_point_parametrization = True
    ba_sum = pt.sfm.BundleAdjustReconstruction(opts, recon)
    final_repr_err = reprojection_error(recon)
    print("Final reprojection error: {:.3f}px".format(final_repr_err))   

    end_rec_time = time.perf_counter()
    print("Entire reconstruction took {}s to complete.".format(end_rec_time-start_rec_time))
    print('Reconstruction summary message: {}'.format(recon_sum.message))
    pt.io.WritePlyFile(os.path.join(args.output_path,"poster.ply"), recon, (255,0,0), 2)
    pt.io.WriteReconstruction(recon, os.path.join(args.output_path,"poster.recon"))

    # export nerfstudio scene
    res = pt.io.WriteNerfStudio(img_path, recon, 16, os.path.join(args.output_path, "transforms.json"))
    