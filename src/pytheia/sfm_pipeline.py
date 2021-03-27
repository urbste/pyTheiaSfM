import numpy as np
import cv2 
#from camera_calibration import mtx, dist
import glob
import argparse

#  pytheia camera module
from pytheia_sfm import Camera, PinholeRadialTangentialCameraModel, CameraIntrinsicsPrior, CameraIntrinsicsModelType

# 
from pytheia_image import Keypoint, KeypointType
from pytheia_matching import KeypointsAndDescriptors


# pytheia feature matcher
from pytheia_matching import InMemoryFeaturesAndMatchesDatabase, BruteForceFeatureMatcher

# FeatureMatcher options
from pytheia_matching import FeatureMatcherOptions
from pytheia_sfm import TwoViewMatchGeometricVerification

from pytheia_sfm import EstimateTwoViewInfoOptions, EstimateTwoViewInfo, TwoViewInfo

from pytheia_sfm import RansacType

from pytheia_matching import FeatureCorrespondence, IndexedFeatureMatch, ImagePairMatch

from pytheia_sfm import ViewGraph, Reconstruction, TrackBuilder, LossFunctionType, GlobalReconstructionEstimator, IncrementalReconstructionEstimator, HybridReconstructionEstimator, ReconstructionEstimatorSummary

# Reconstruction related import 
from pytheia_matching import MatchingStrategy
from pytheia_image import FeatureDensity
from pytheia_image import DescriptorExtractorType
from pytheia_matching import FeatureMatcherOptions
from pytheia_sfm import ReconstructionEstimatorOptions
from pytheia_sfm import ReconstructionBuilderOptions, ReconstructionBuilder

from pytheia_io import WritePlyFile, WriteReconstruction

def match_image_pair(image_file_path1, image_file_path2, featuretype, matchertype):

    img1_name = remove_prefix_and_suffix(image_file_path1)
    img2_name = remove_prefix_and_suffix(image_file_path2)
    view_id1 = recon.ViewIdFromName(img1_name)
    view_id2 = recon.ViewIdFromName(img2_name)
    print('Matching between image {} and {}  starts...'.format(img1_name, img2_name))

    img1 = cv2.imread(image_file_path1)
    img2 = cv2.imread(image_file_path2)

    if featuretype == 'akaze':
        feature = cv2.AKAZE_create()
    elif featuretype == 'sift':
        feature = cv2.SIFT_create()
    
    

    #orb = cv2.ORB()
    #kp1, des1 = orb.detectAndCompute(img1,None)
    #kp2, des2 = orb.detectAndCompute(img2,None)
    #sift = cv2.SIFT()
    kpts1, desc1 = feature.detectAndCompute(img1, None)
    kpts2, desc2 = feature.detectAndCompute(img2, None)   

    pts1 = [p.pt for p in kpts1]
    pts2 = [p.pt for p in kpts2]

    # create BFMatcher object
    if matchertype == 'bf':
        matcher = cv2.BFMatcher()

    matches = matcher.knnMatch(desc1,desc2,k=2)
    #bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
    #matches = bf.knnMatch(desc1,desc2, k=2)

    # Apply ratio test
    filtered_matches = []
    for m,n in matches:
        if m.distance < 0.8*n.distance:
            filtered_matches.append(m)

    print('Number of putative matches: {}'.format(len(filtered_matches)))

    if len(filtered_matches) < min_num_inlier_matches:
        print('Number of putative matches too low!')
        return False, None

    correspondences = correspondence_from_indexed_matches(filtered_matches, pts1, pts2)

    options = EstimateTwoViewInfoOptions()
    if ransactype == 'ransac':
        options.ransac_type = RansacType(0)
    elif ransactype == 'prosac':
        options.ransac_type = RansacType(1)
    elif ransactype == 'lmed':
        options.ransac_type = RansacType(2)

    success, twoview_info, inlier_indices = EstimateTwoViewInfo(options, prior, prior, correspondences)

    print('Only {} matches survived after geometric verification'.format(len(inlier_indices)))
    if len(inlier_indices) < min_num_inlier_matches:
        print('Number of putative matches after geometric verification are too low!')
        return False, None

    else:
        verified_matches = []
        for i in range(len(inlier_indices)):
            verified_matches.append(filtered_matches[inlier_indices[i]])

        correspondences = correspondence_from_indexed_matches(verified_matches, pts1, pts2)
        twoview_info.num_verified_matches = len(verified_matches)
        imagepair_match = ImagePairMatch()
        imagepair_match.image1 = img1_name
        imagepair_match.image2 = img2_name
        imagepair_match.twoview_info = twoview_info
        imagepair_match.correspondences = correspondences

        for i in range(len(verified_matches)):
            track_builder.AddFeatureCorrespondence(view_id1, correspondences[i].feature1, view_id2, correspondences[i].feature2)
        
        return True, imagepair_match


def remove_prefix_and_suffix(image_file):
    if image_file.endswith('.jpeg'):
        image_name = image_file[5:-5]  # for milk folder
    
    return image_name

min_num_inlier_matches = 30

# create correspondences of keypoints locations from indexed feature matches
def correspondence_from_indexed_matches(filtered_matches, pts1, pts2):
    correspondences = []
    for match in filtered_matches:
        point1 = np.array(pts1[match.queryIdx])
        point2 = np.array(pts2[match.trainIdx])
        feature_correspondece = FeatureCorrespondence(point1, point2)
        correspondences.append(feature_correspondece)

    return correspondences








if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Argument parser for sfm pipeline')
    parser.add_argument('--feature', type=str, default='sift',
                    help='feature descriptor type: sift or akaze')
    parser.add_argument('--matcher', type=str, default='bf',
                    help='feature matcher type: bf or cascadehash')
    parser.add_argument('--ransac', type=str, default='ransac', 
                    help='ransac type for estimator: ransac, prosac or lmed')
    parser.add_argument('--reconstruction', type=str, default='global',
                    help='reconstruction type: global, incremental or hybrid')

    args = parser.parse_args()
    ransactype = args.ransac
    featuretype = args.feature
    matchertype = args.matcher
    reconstructiontype = args.reconstruction
    print('Configurations: ransactype: {}; featuretype: {}; matchertype: {}; reconstructiontype: {}'.format(ransactype, featuretype, matchertype, reconstructiontype))

    #Pipeline starts
    print('Pipeline starts...')

    view_graph = ViewGraph()
    recon = Reconstruction()
    track_builder = TrackBuilder(3, 30)

    # camera intrinsic parameters and distortion coefficients
    cam_matrix = [[3.06661152e+03, 0.00000000e+00, 1.96956075e+03],
    [0.00000000e+00, 3.07017469e+03, 1.57371386e+03],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
    distortion_params = [0.13437347, -0.83580167,  0.00354001,  0.00444744,  1.33033269]
    k1, k2, p1, p2, k3 = distortion_params
    print('distortion coefficients are: {}{}{}{}{}'.format(k1, k2, p1, p2, k3))


    focal_length = cam_matrix[0][0]
    aspect_ratio = cam_matrix[1][1]/cam_matrix[0][0]
    cx = cam_matrix[0][2]
    cy = cam_matrix[1][2]

    prior = CameraIntrinsicsPrior()
    prior.focal_length.value = [focal_length]
    prior.aspect_ratio.value = [aspect_ratio]
    prior.principal_point.value = [cx, cy]
    prior.radial_distortion.value = [k1, k2, k3, 0]
    prior.tangential_distortion.value = [p1, p2]
    prior.skew.value = [0]
    prior.camera_intrinsics_model_type = 'PINHOLE_RADIAL_TANGENTIAL'

    # pinhole radial tangential camera
    camera = Camera(CameraIntrinsicsModelType(1))
    camera.SetFromCameraIntrinsicsPriors(prior)


    # opencv extraction of features from images
    images_files = glob.glob('milk/*.jpeg')
    image_names = []
    for image_file in images_files:
        image_names.append(remove_prefix_and_suffix(image_file))
    
    print('{} images have been found'.format(len(images_files)))
    print('Image files: {}'.format(images_files))

    for image_name in image_names:
        view_id = recon.AddView(image_name, 0)
        c = recon.MutableView(view_id).MutableCamera()
        c.DeepCopy(camera)
        recon.MutableView(view_id).SetCameraIntrinsicsPrior(c.CameraIntrinsicsPriorFromIntrinsics()) 



    num_images = len(images_files)
    for i in range(num_images):
        for j in range(i+1, num_images):
            success, imagepair_match = match_image_pair(images_files[i], images_files[j], featuretype, matchertype)
            if success == True:
                view_id1 = recon.ViewIdFromName(imagepair_match.image1)
                view_id2 = recon.ViewIdFromName(imagepair_match.image2)
                view_graph.AddEdge(view_id1, view_id2, imagepair_match.twoview_info)
                print("Match between image {} and image {}. ".format(imagepair_match.image1, imagepair_match.image2))
                print("Match between image index {} and image index {}.\n\n ".format(i, j))
            else:
                print("No match between image {} and image {}. ".format(remove_prefix_and_suffix(images_files[i]), remove_prefix_and_suffix(images_files[j])))
                print("No match between image index {} and image index {}.\n\n ".format(i, j))
    
    print('{} edges were added to the view graph.'.format(view_graph.NumEdges))
    track_builder.BuildTracks(recon)
    options = ReconstructionEstimatorOptions()
    options.num_threads = 7
    options.rotation_filtering_max_difference_degrees = 10.0
    options.bundle_adjustment_robust_loss_width = 3.0
    options.bundle_adjustment_loss_function_type = LossFunctionType(1)
    options.subsample_tracks_for_bundle_adjustment = True
    options.filter_relative_translations_with_1dsfm = True

    if reconstructiontype == 'global':
        reconstruction_estimator = GlobalReconstructionEstimator(options)
    elif reconstructiontype == 'incremental':
        reconstruction_estimator = IncrementalReconstructionEstimator(options)
    elif reconstructiontype == 'hybrid':
        reconstruction_estimator = HybridReconstructionEstimator(options)
    recon_sum = reconstruction_estimator.Estimate(view_graph, recon)

    print('Reconstruction summary message: {}'.format(recon_sum.message))
    WritePlyFile("test.ply", recon, 2)
    WriteReconstruction(recon, "reconstruction_file")
