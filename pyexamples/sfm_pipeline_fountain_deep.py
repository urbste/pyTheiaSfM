# Steffen Urban, 2023

# This simple example demonstrates the use of pytheia to reconstruct the 
# Fountain dataset scene
# In this case, we are using deep feature detection and matching 
# based on the DISK detector and the Lightglue matcher from Kornia


import numpy as np
import os, glob, argparse, time
import pytheia as pt
import torch, kornia
from image_utils import load_image

min_num_inlier_matches = 30

# create correspondences of keypoints locations from indexed feature matches
def correspondence_from_indexed_matches(match_indices, pts1, pts2):
    num_matches = match_indices.shape[0]
    correspondences = [
         pt.matching.FeatureCorrespondence(
            pt.sfm.Feature(pts1[match_indices[m,0],:]), 
            pt.sfm.Feature(pts2[match_indices[m,1],:])) for m in range(num_matches)]

    return correspondences

def match_image_pair(img_i_data, img_j_data, matcher, min_conf):

    with torch.no_grad():
        scales0, scales1 = img_i_data["scales"].to(device), img_j_data["scales"].to(device)

        data = {'image0': img_i_data["image"], 
                'image1': img_j_data["image"]}
        pred = {**{k+'0': torch.from_numpy(v).to(device) for k, v in img_i_data["feats"].items()},
                **{k+'1': torch.from_numpy(v).to(device) for k, v in img_j_data["feats"].items()},
                **data}

        input_dict = {
            "image0": {
                "keypoints": pred["keypoints0"],
                "descriptors": pred["descriptors0"],
                "image_size": torch.tensor(img_i_data["image"].shape[2:]).flip(0).reshape(-1, 2),
            },
            "image1": {
                "keypoints": pred["keypoints1"],
                "descriptors": pred["descriptors1"],
                "image_size":  torch.tensor(img_j_data["image"].shape[2:]).flip(0).reshape(-1, 2),
            },
        }

        start = time.perf_counter()
        result = matcher(input_dict)
        end = time.perf_counter()
        print("Matching took {}s".format((end-start)))

        if scales0 is not None:
            pred['keypoints0'] = (pred['keypoints0'] + 0.5) / scales0[None] - 0.5
        if scales1 is not None:
            pred['keypoints1'] = (pred['keypoints1'] + 0.5) / scales1[None] - 0.5
        torch.cuda.empty_cache()

    if scales0 is not None:
        pred['keypoints0'] = (pred['keypoints0'] + 0.5) / scales0[None] - 0.5
    if scales1 is not None:
        pred['keypoints1'] = (pred['keypoints1'] + 0.5) / scales1[None] - 0.5
    torch.cuda.empty_cache()

    # create match indices
    kpts0, kpts1 = pred['keypoints0'][0], pred['keypoints1'][0]
    matches0, mscores0, mscores1 = result['matches0'][0], result['matching_scores0'][0], result['matching_scores1'][0]
    valid = matches0 > -1
    matches = torch.stack([torch.where(valid)[0], matches0[valid]], -1)

    if matches.shape[0] < min_num_inlier_matches:
        print('Number of putative matches too low!')
        return False, None

    mscores0 = mscores0.cpu().numpy()
    mscores1 = mscores1.cpu().numpy()
    
    kpts0 = kpts0.cpu().numpy()
    kpts1 = kpts1.cpu().numpy()

    correspondences = correspondence_from_indexed_matches(matches, kpts0, kpts1)

    options = pt.sfm.EstimateTwoViewInfoOptions()
    options.ransac_type = pt.sfm.RansacType(0) # pt.sfm.RansacType(1): prosac, pt.sfm.RansacType(2): lmed
    options.use_lo = True # Local Optimization Ransac
    options.use_mle = True 
    options.max_sampson_error_pixels = 2.0

    success, twoview_info, inlier_indices = pt.sfm.EstimateTwoViewInfo(
        options, prior, prior, correspondences)


    if not success:
        print('TwoViewInfo estimation failed!')
        return False, None

    if len(inlier_indices) < min_num_inlier_matches or not success:
        print('Number of putative matches after geometric verification are too low!')
        return False, None
    else:

        twoview_info.num_verified_matches = len(inlier_indices)

        for i in range(len(inlier_indices)):
            corr = correspondences[inlier_indices[i]]
            track_builder.AddFeatureCorrespondence(
                img_i_data["view_id"], corr.feature1, 
                img_j_data["view_id"], corr.feature2)
        
        return True, twoview_info

def extract_features(image, extractor):

    features = extractor(image, n = 2048, 
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

    return {'keypoints': keypoints.to(image),
            'keypoint_scores': scores.to(image),
            'descriptors': descriptors.to(image)}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Argument parser for sfm pipeline')
    parser.add_argument('--path_fountain_dataset', type=str, default=" /home/zosurban/Downloads/fountain/")
    parser.add_argument('--reconstruction', type=str, default='global',
                    help='reconstruction type: global, incremental or hybrid')
    parser.add_argument('--img_ext', default='png')
    parser.add_argument('--device', default="cuda")
    parser.add_argument('--use_fp16', default=True)
    args = parser.parse_args()
    reconstructiontype = args.reconstruction

    extractor = kornia.feature.DISK.from_pretrained("depth").eval()
    matcher = kornia.feature.LightGlue(features="disk").eval()

    device = "cpu"
    dtype = torch.float32
    if torch.cuda.is_available() and args.device == "cuda":
        device = args.device
        matcher = matcher.cuda()
        extractor = extractor.cuda()
        if args.use_fp16 and torch.cuda.is_bf16_supported():
            matcher = matcher.half()
            extractor = extractor.half()
            dtype = torch.float16
    
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

    # add views and extract features
    img_data = {}
    
    for idx, image_name in enumerate(image_names):
        img_name_ext = image_name+"."+args.img_ext
        vid = recon.AddView(img_name_ext, 0, idx)
        v = recon.MutableView(vid)
        v.SetCameraIntrinsicsPrior(prior)
        # load images to torch and resize to max_edge=1024
        image, scale = load_image(os.path.join(img_path, img_name_ext), resize=None)
        image = image.unsqueeze(0) # add batch dimension [b, 3, x, 1024]
        with torch.no_grad():
            start_t = time.perf_counter()
            img_t = image.to(device).to(dtype)
            tmp = extract_features(img_t, extractor)
            print("Feature extraction took {}s".format(time.perf_counter() - start_t))

        feats = {"keypoints": tmp["keypoints"].cpu().numpy(), 
                 "keypoint_scores": tmp["keypoint_scores"].cpu().numpy(), 
                 "descriptors": tmp["descriptors"].cpu().numpy()}

        img_data[img_name_ext] = {
            "view_id": vid, "image" : image, "feats": feats, "scales": scale.to(device).to(dtype)}
    # make sure the intrinsics are all initialized fromt he priors
    pt.sfm.SetCameraIntrinsicsFromPriors(recon) 

    # now match the images using lightglue
    view_ids = sorted(recon.ViewIds())
    for i in range(len(view_ids)):
        img_i_name = recon.View(view_ids[i]).Name()

        for j in range(i+1, len(view_ids)):
            img_j_name = recon.View(view_ids[j]).Name()
            
            success, twoview_info = match_image_pair( 
                img_data[img_i_name], img_data[img_j_name], matcher, 0.8)
            
            nr_matches = twoview_info.num_verified_matches
            if success == True:
                view_id1 = recon.ViewIdFromName(img_i_name)
                view_id2 = recon.ViewIdFromName(img_j_name)
                view_graph.AddEdge(view_id1, view_id2, twoview_info)
                print("{} Matches  between image {} and image {}. ".format(
                    nr_matches, img_i_name, img_j_name))
            else:
                print("Only {} matches between image {} and image {}. Removing from view graph.".format(
                    nr_matches, img_i_name, img_j_name))
    
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
        options.global_position_estimator_type = pt.sfm.GlobalPositionEstimatorType.LEAST_UNSQUARED_DEVIATION
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
