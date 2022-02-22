# // Copyright (C) 2015 The Regents of the University of California (Regents).
# // All rights reserved.
# //
# // Redistribution and use in source and binary forms, with or without
# // modification, are permitted provided that the following conditions are
# // met:
# //
# //     * Redistributions of source code must retain the above copyright
# //       notice, this list of conditions and the following disclaimer.
# //
# //     * Redistributions in binary form must reproduce the above
# //       copyright notice, this list of conditions and the following
# //       disclaimer in the documentation and/or other materials provided
# //       with the distribution.
# //
# //     * Neither the name of The Regents or University of California nor the
# //       names of its contributors may be used to endorse or promote products
# //       derived from this software without specific prior written permission.
# //
# // THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# // AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# // IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# // ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
# // LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# // CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# // SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# // INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# // CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# // ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# // POSSIBILITY OF SUCH DAMAGE.
# //
# // Please contact the author of this library if you have any questions.
# // Author: Steffen Urban (urbse@googlemail.com)

import os
import numpy as np
import cv2 
from cv2.xfeatures2d import matchGMS
import glob
import argparse
import time

import pytheia as pt

min_num_inlier_matches = 150

def remove_prefix_and_suffix(image_file):
    pos = image_file.rfind('/')+1
    pos1 = image_file.rfind('.')
    image_name = image_file[pos:pos1]  
    return image_name

# create correspondences of keypoints locations from indexed feature matches
def correspondence_from_matches(filtered_matches, feat1, feat2):
    correspondences = []
    for match in filtered_matches:
        point1 = np.array(feat1[match.queryIdx].pt)
        point2 = np.array(feat2[match.trainIdx].pt)
        feature_correspondece = pt.matching.FeatureCorrespondence(
            pt.sfm.Feature(point1), pt.sfm.Feature(point2))
        correspondences.append(feature_correspondece)

    return correspondences

def extract_features(image_path, mask_path, featuretype, recon):

    img_name = remove_prefix_and_suffix(image_path)
    view_id = recon.ViewIdFromName(img_name)
    cam = recon.MutableView(view_id).MutableCamera()
    img = cv2.imread(image_path, 0)
    cam.SetImageSize(img.shape[1],img.shape[0])

    img = cv2.resize(img, (cam.ImageWidth,cam.ImageHeight))
    if mask_path:
        mask = cv2.resize(cv2.imread(mask_path, 0), (cam.ImageWidth,cam.ImageHeight))
    else:
        mask = None

    if featuretype == 'akaze':
        feature = cv2.AKAZE_create(cv2.AKAZE_DESCRIPTOR_KAZE, 0, 3, 0.0001, 2, 2)
    elif featuretype == 'sift':
        feature = cv2.SIFT_create(5000)
    elif featuretype == 'akaze_bin':
        feature = cv2.AKAZE_create(cv2.AKAZE_DESCRIPTOR_MLDB, 0, 3, 0.0001, 2, 2)

    kpts1, desc1 = feature.detectAndCompute(img, mask)

    return kpts1, desc1, view_id, img

def match_image_pair(recon, track_builder, features, vid1, vid2, matchertype):
    cross_check = matchertype == "gms"
    if featuretype == 'akaze':
        matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=cross_check)
    elif featuretype == 'sift':
        matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=cross_check)
    elif featuretype == "akaze_bin":
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=cross_check)

    if matchertype == "knn":
        matches = matcher.knnMatch(features[vid1]["descriptors"], features[vid2]["descriptors"], k=2)

        # Apply ratio test
        filtered_matches = []
        for m,n in matches:
            if m.distance < 0.8*n.distance:
                filtered_matches.append(m)

    elif matchertype == "gms":
        matches12 = matcher.match(features[vid1]["descriptors"], features[vid2]["descriptors"])
        filtered_matches = matchGMS(features[vid1]["img_wh"], features[vid2]["img_wh"],
            features[vid1]["keypoints"], features[vid2]["keypoints"], matches12)

    print('Number of putative matches: {}'.format(len(filtered_matches)))

    if len(filtered_matches) < min_num_inlier_matches:
        print('Number of putative matches too low!')
        return False, None, None

    correspondences = correspondence_from_matches(
        filtered_matches, features[vid1]["keypoints"], features[vid2]["keypoints"])

    options = pt.sfm.EstimateTwoViewInfoOptions()
    options.max_sampson_error_pixels = 0.5
    options.max_ransac_iterations = 200
    if ransactype == 'ransac':
        options.ransac_type = pt.sfm.RansacType(0)
    elif ransactype == 'prosac':
        options.ransac_type = pt.sfm.RansacType(1)
    elif ransactype == 'lmed':
        options.ransac_type = pt.sfm.RansacType(2)

    prior1 = recon.View(vid1).Camera().CameraIntrinsicsPriorFromIntrinsics()
    prior2 = recon.View(vid2).Camera().CameraIntrinsicsPriorFromIntrinsics()
    success, twoview_info, inlier_indices = pt.sfm.EstimateTwoViewInfo(options, prior1, prior2, correspondences)

    print('Only {} matches survived after geometric verification'.format(len(inlier_indices)))

    if not success or len(inlier_indices) < min_num_inlier_matches:
        print('Number of putative matches after geometric verification is too low!')
        return False, None, None
    else:
        verified_matches = []
        for i in range(len(inlier_indices)):
            verified_matches.append(filtered_matches[inlier_indices[i]])

        correspondences_verified = correspondence_from_matches(
            verified_matches, features[vid1]["keypoints"], features[vid2]["keypoints"])   

        for i in range(len(verified_matches)):
            track_builder.AddFeatureCorrespondence(vid1, correspondences_verified[i].feature1, 
                                                   vid2, correspondences_verified[i].feature2)

        return True, twoview_info, correspondences_verified

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Argument parser for sfm pipeline')
    parser.add_argument('--feature', type=str, default='akaze_bin',
                    help='feature descriptor type: sift or akaze')
    parser.add_argument('--matcher', type=str, default='gms', choices=["knn", "gms"],
                    help='feature matcher type: knn or gms')
    parser.add_argument('--ransac', type=str, default='ransac', 
                    help='ransac type for estimator: ransac, prosac or lmed')
    parser.add_argument('--reconstruction', type=str, default='global',
                    help='reconstruction type: global, incremental or hybrid')
    parser.add_argument('--image_path', type=str, default="")
    parser.add_argument('--output_path', type=str, default="")
    parser.add_argument('--debug', type=int, default=1)
    parser.add_argument("--fcxcy", nargs=3, default=["5287.877","698.1492", "523.5119"],
                        help="pinhole camera params: focal length, printipcal point cx, cy", type=float)
                    
    args = parser.parse_args()
    print(float(args.fcxcy[0]))
    ransactype = args.ransac
    featuretype = args.feature
    matchertype = args.matcher
    reconstructiontype = args.reconstruction
    print('Configurations: ransactype: {}; featuretype: {}; matchertype: {}; reconstructiontype: {}'.format(
        ransactype, featuretype, matchertype, reconstructiontype))

    #Pipeline starts
    print('Pipeline starts...')

    view_graph = pt.sfm.ViewGraph()
    recon = pt.sfm.Reconstruction()
    track_builder = pt.sfm.TrackBuilder(3, 30)

    prior = pt.sfm.CameraIntrinsicsPrior()
    prior.focal_length.value = [args.focal_length]
    prior.aspect_ratio.value = [1.0]
    prior.principal_point.value = [698.1492792451463, 523.5119897611621]
    prior.radial_distortion.value = [0, 0, 0, 0]
    prior.tangential_distortion.value = [0, 0]
    prior.skew.value = [0]
    # 'PINHOLE_RADIAL_TANGENTIAL', 'DIVISION_UNDISTORTION', 'DOUBLE_SPHERE', 'FOV', 'EXTENDED_UNIFIED', 'FISHEYE
    prior.camera_intrinsics_model_type = 'PINHOLE' 

    # init camera
    camera = pt.sfm.Camera()
    camera.SetFromCameraIntrinsicsPriors(prior)

    # opencv extraction of features from images
    images_files = glob.glob(os.path.join(args.image_path,'*.png'))
    image_names = []
    for image_file in images_files:
        image_names.append(remove_prefix_and_suffix(image_file))
    
    print('{} images have been found'.format(len(images_files)))
    print('Image files: {}'.format(images_files))

    for i, image_name in enumerate(image_names):
        view_id = recon.AddView(image_name, 0, i)
        c = recon.MutableView(view_id).MutableCamera()
        c.DeepCopy(camera)

    features = {}
    num_images = len(images_files)
    for i in range(num_images):
        kpts, desc, view_id, img = extract_features(images_files[i], None, featuretype, recon)
        features[view_id] = {"keypoints" : kpts, "descriptors" : desc, "img": img, "img_wh": img.shape[:2][::-1]}
        print("Extracted {} features from {}".format(len(kpts), images_files[i]))

    view_ids = recon.ViewIds
    for i in range(len(view_ids)):
        for j in range(i+1,len(view_ids)):
            view_id1 = view_ids[i]
            view_id2 = view_ids[j]
            success, two_view_info, correspondences_verified = match_image_pair(recon, track_builder, features, view_id1, view_id2, matchertype)
 
            if success == True:
                if args.debug:
                    img1 = features[view_id1]["img"]
                    img2 = features[view_id2]["img"]
                    images = np.concatenate([img1,img2],1)
                    images = cv2.cvtColor(images, cv2.COLOR_GRAY2BGR)
                    for cor in correspondences_verified:
                        pt1 = (int(cor.feature1.point[0]), int(cor.feature1.point[1]))
                        pt2 = (int(cor.feature2.point[0]+img2.shape[1]), int(cor.feature2.point[1]))
                        images=cv2.line(images, pt1, pt2, (255,0,0), 1)
                    cv2.imshow("matches", images)
                    cv2.waitKey(0)
                view_graph.AddEdge(view_id1, view_id2, two_view_info)

                print("Match between view {} and view {}. ".format(view_id1, view_id2))
            else:
                print("No match between view {} and view {}.\n\n ".format(view_id1, view_id2))
    
    print('{} edges were added to the view graph.'.format(view_graph.NumEdges))
    track_builder.BuildTracks(recon)
    options = pt.sfm.ReconstructionEstimatorOptions()
    options.num_threads = 7
    options.rotation_filtering_max_difference_degrees = 10.0
    options.bundle_adjustment_robust_loss_width = 3.0
    options.bundle_adjustment_loss_function_type = pt.sfm.LossFunctionType(1)
    options.subsample_tracks_for_bundle_adjustment = False
    options.filter_relative_translations_with_1dsfm = True
    options.intrinsics_to_optimize = pt.sfm.OptimizeIntrinsicsType.NONE
    options.min_triangulation_angle_degrees = 2.0
    options.triangulation_method = pt.sfm.TriangulationMethodType(0)
    if reconstructiontype == 'global':
        options.global_position_estimator_type = pt.sfm.GlobalPositionEstimatorType.LEAST_UNSQUARED_DEVIATION
        options.global_rotation_estimator_type = pt.sfm.GlobalRotationEstimatorType.ROBUST_L1L2  
        reconstruction_estimator = pt.sfm.GlobalReconstructionEstimator(options)
    elif reconstructiontype == 'incremental':
        reconstruction_estimator = pt.sfm.IncrementalReconstructionEstimator(options)
    elif reconstructiontype == 'hybrid':
        reconstruction_estimator = pt.sfm.HybridReconstructionEstimator(options)
    recon_sum = reconstruction_estimator.Estimate(view_graph, recon)

    print('Reconstruction summary message: {}'.format(recon_sum.message))
    print("Writing results to ",args.output_path)
    pt.io.WritePlyFile(os.path.join(args.output_path,"test.ply"), recon, [255,0,0],2)
    pt.io.WriteReconstruction(recon, os.path.join(args.output_path,"theia_recon.recon"))
