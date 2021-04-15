
pyTheia - A Python Structure-from-Motion and Geometric Vision Library
---------------------

pyTheia is based on [TheiaSfM](http://www.theia-sfm.org).
It contains Python bindings for most of the functionalities of TheiaSfM.

With pyTheia you have access to a variety of different camera models, structure-from-motion pipelines and geometric vision algorithms.


## Example

### Create a camera
``` Python
import pytheia as pt
prior = pt.sfm.CameraIntrinsicsPrior()
prior.focal_length.value = [focal_length]
prior.aspect_ratio.value = [aspect_ratio]
prior.principal_point.value = [cx, cy]
prior.radial_distortion.value = [k1, k2, k3, 0]
prior.tangential_distortion.value = [p1, p2]
prior.skew.value = [0]
prior.camera_intrinsics_model_type = 'PINHOLE_RADIAL_TANGENTIAL' 
#'PINHOLE', 'DOUBLE_SPHERE', 'EXTENDED_UNIFIED', 'FISHEYE', 'FOV', 'DIVISION_UNDISTORTION'
camera = pt.sfm.Camera(pt.sfm.CameraIntrinsicsModelType(1))
camera.SetFromCameraIntrinsicsPriors(prior)
```

### Solve for absolute or relative camera pose
``` Python
import pytheia as pt

# absolute pose
pose = pt.sfm.PoseFromThreePoints(pts2D, pts3D)
pose = FourPointsPoseFocalLengthRadialDistortion(pts2D, pts3D)
pose = FourPointPoseAndFocalLength(pts2D, pts3D)
pose = DlsPnp(pts2D, pts3D)

# relative pose
pose = pt.sfm.NormalizedEightPointFundamentalMatrix(pts2D, pts2D)
pose = pt.sfm.FourPointHomography(pts2D, pts2D)
pose = pt.sfm.FivePointRelativePose(pts2D, pts2D)
pose = pt.sfm.SevenPointFundamentalMatrix(pts2D, pts2D)
```

### Create a reconstruction
Have a look at the example: sfm_pipeline.py
``` Python
import pytheia as pt
# use your favourite Feature extractor matcher 
# can also be any deep stuff
view_graph = pt.sfm.ViewGraph()
recon = pt.sfm.Reconstruction()
track_builder = pt.sfm.TrackBuilder(3, 30)

# ... match some features to find putative correspondences
success, twoview_info, inlier_indices = pt.sfm.EstimateTwoViewInfo(options, prior, prior, correspondences)
# ... get filtered feature correspondences and add them to the reconstruction
correspondences = pt.matching.FeatureCorrespondence(
            pt.sfm.Feature(point1), pt.sfm.Feature(point2))
imagepair_match = pt.matching.ImagePairMatch()
imagepair_match.image1 = img1_name
imagepair_match.image2 = img2_name
imagepair_match.twoview_info = twoview_info
imagepair_match.correspondences = correspondences
for i in range(len(verified_matches)):
  track_builder.AddFeatureCorrespondence(view_id1, correspondences[i].feature1, 
                                         view_id2, correspondences[i].feature2)

# ... Build Tracks
track_builder.BuildTracks(recon)

ptions = pt.sfm.ReconstructionEstimatorOptions()
options.num_threads = 7
options.rotation_filtering_max_difference_degrees = 10.0
options.bundle_adjustment_robust_loss_width = 3.0
options.bundle_adjustment_loss_function_type = pt.sfm.LossFunctionType(1)
options.subsample_tracks_for_bundle_adjustment = True

if reconstructiontype == 'global':
  options.filter_relative_translations_with_1dsfm = True
  reconstruction_estimator = pt.sfm.GlobalReconstructionEstimator(options)
elif reconstructiontype == 'incremental':
  reconstruction_estimator = pt.sfm.IncrementalReconstructionEstimator(options)
elif reconstructiontype == 'hybrid':
  reconstruction_estimator = pt.sfm.HybridReconstructionEstimator(options)
recon_sum = reconstruction_estimator.Estimate(view_graph, recon)

pt.io.WritePlyFile("test.ply", recon, [255,0,0],2)
pt.io.WriteReconstruction(recon, "reconstruction_file")
```