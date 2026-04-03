from __future__ import annotations
import typing
__all__: list[str] = ['ACCELERATE_SPARSE', 'ALL', 'ARCTAN', 'ASPECT_RATIO', 'AddFeatureCorrespondencesToTrackBuilder', 'AddFullFeatureCorrespondencesToTrackBuilder', 'AddObservations', 'AddTracks', 'AlignPointCloudsUmeyama', 'AlignPointCloudsUmeyamaWithWeights', 'AlignReconstructions', 'AlignReconstructionsRobust', 'AlignRotations', 'BundleAdjustPartialReconstruction', 'BundleAdjustPartialViewsConstant', 'BundleAdjustReconstruction', 'BundleAdjustTrack', 'BundleAdjustTrackWithCov', 'BundleAdjustTracks', 'BundleAdjustTracksWithCov', 'BundleAdjustTwoViewsAngular', 'BundleAdjustView', 'BundleAdjustViewWithCov', 'BundleAdjustViews', 'BundleAdjustViewsWithCov', 'BundleAdjuster', 'BundleAdjustmentOptions', 'BundleAdjustmentSummary', 'CANONICAL_VIEWS', 'CAUCHY', 'CGNR', 'CLUSTER_JACOBI', 'CLUSTER_TRIDIAGONAL', 'CUDA', 'CUDA_SPARSE', 'CalibratedAbsolutePose', 'CalibrationMatrixToIntrinsics', 'Camera', 'CameraAndFeatureCorrespondence2D3D', 'CameraIntrinsicsModel', 'CameraIntrinsicsModelType', 'CameraIntrinsicsPrior', 'ColorizeReconstruction', 'ComposeFundamentalMatrix', 'ComposeProjectionMatrix', 'ComputeTripletBaselineRatios', 'CreateEstimatedSubreconstruction', 'DENSE_NORMAL_CHOLESKY', 'DENSE_QR', 'DENSE_SCHUR', 'DISTORTION', 'DIVISION_UNDISTORTION', 'DLS', 'DOUBLE_SPHERE', 'DecomposeEssentialMatrix', 'DecomposeProjectionMatrix', 'DenseLinearAlgebraLibraryType', 'DivisionUndistortionCameraModel', 'DlsPnp', 'EIGEN', 'EIGEN_SPARSE', 'EXHAUSTIVE', 'EXTENDED_UNIFIED', 'EssentialMatrixFromFundamentalMatrix', 'EssentialMatrixFromTwoProjectionMatrices', 'EstimateAbsolutePoseWithKnownOrientation', 'EstimateCalibratedAbsolutePose', 'EstimateDominantPlaneFromPoints', 'EstimateEssentialMatrix', 'EstimateFundamentalMatrix', 'EstimateHomography', 'EstimateRadialHomographyMatrix', 'EstimateRelativePose', 'EstimateRelativePoseWithKnownOrientation', 'EstimateRigidTransformation2D3D', 'EstimateRigidTransformation2D3DNormalized', 'EstimateTriangulation', 'EstimateTwoViewInfo', 'EstimateTwoViewInfoOptions', 'EstimateUncalibratedAbsolutePose', 'EstimateUncalibratedRelativePose', 'ExtractMaximallyParallelRigidSubgraph', 'FISHEYE', 'FOCAL_LENGTH', 'FOCAL_LENGTH_DISTORTION', 'FOCAL_LENGTH_RADIAL_DISTORTION', 'FOV', 'FOVCameraModel', 'Feature', 'FeatureCorrespondence2D3D', 'FilterViewGraphCyclesByRotation', 'FilterViewPairsFromOrientation', 'FilterViewPairsFromRelativeTranslation', 'FilterViewPairsFromRelativeTranslationOptions', 'FindCommonTracksByFeatureInReconstructions', 'FindCommonTracksInViews', 'FindCommonViewsByName', 'FisheyeCameraModel', 'FivePointFocalLengthRadialDistortion', 'FivePointRelativePose', 'FocalLengthsFromFundamentalMatrix', 'FourPointHomography', 'FourPointPoseAndFocalLength', 'FourPointRelativePosePartialRotation', 'FourPointsPoseFocalLengthRadialDistortion', 'FundamentalMatrixFromProjectionMatrices', 'GLOBAL', 'GPSConverter', 'GdlsSimilarityTransform', 'GetBestPoseFromEssentialMatrix', 'GetEstimatedTracksFromReconstruction', 'GetEstimatedViewsFromReconstruction', 'GlobalPositionEstimatorType', 'GlobalReconstructionEstimator', 'GlobalRotationEstimatorType', 'HUBER', 'HYBRID', 'HybridReconstructionEstimator', 'HybridRotationEstimator', 'IDENTITY', 'INCREMENTAL', 'INVALID', 'INVERSE_DEPTH', 'ITERATIVE_SCHUR', 'IncrementalReconstructionEstimator', 'IntrinsicsToCalibrationMatrix', 'IsTriangulatedPointInFrontOfCameras', 'JACOBI', 'KNEIP', 'L2_MINIMIZATION', 'LAGRANGE_DUAL', 'LAPACK', 'LEAST_UNSQUARED_DEVIATION', 'LIGT', 'LINEAR', 'LINEAR_TRIPLET', 'LMED', 'LagrangeDualRotationEstimator', 'LeastUnsquaredDeviationPositionEstimator', 'LeastUnsquaredDeviationPositionEstimatorOptions', 'LiGTPositionEstimator', 'LiGTPositionEstimatorOptions', 'LinearPositionEstimator', 'LinearPositionEstimatorOptions', 'LinearRotationEstimator', 'LinearSolverType', 'LocalizeViewToReconstruction', 'LocalizeViewToReconstructionOptions', 'LossFunctionType', 'MIDPOINT', 'MLPnP', 'NONE', 'NONLINEAR', 'NonlinearPositionEstimator', 'NonlinearPositionEstimatorOptions', 'NonlinearRotationEstimator', 'NormalizedEightPointFundamentalMatrix', 'NumEstimatedTracks', 'NumEstimatedViews', 'ORTHOGRAPHIC', 'OptimizeAbsolutePoseOnNormFeatures', 'OptimizeAlignmentSim3', 'OptimizeIntrinsicsType', 'OptimizeRelativePositionWithKnownRotation', 'OrthographicCameraModel', 'PINHOLE', 'PINHOLE_RADIAL_TANGENTIAL', 'POINT_TO_PLANE', 'POINT_TO_POINT', 'PRINCIPAL_POINTS', 'PROSAC', 'PinholeCameraModel', 'PinholeRadialTangentialCameraModel', 'PlanarUncalibratedOrthographicPose', 'Plane', 'PnPType', 'PoseFromThreePoints', 'PositionEstimator', 'PositionFromTwoRays', 'PreconditionerType', 'PriorScalar', 'PriorVector2d', 'PriorVector3d', 'PriorVector4d', 'ProjectionMatricesFromFundamentalMatrix', 'RADIAL_DISTORTION', 'RANSAC', 'ROBUST_L1L2', 'ROBUST_POINT_TO_POINT', 'RadialDistUncalibratedAbsolutePoseMetaData', 'RadialDistortionFeatureCorrespondence', 'RansacType', 'Reconstruction', 'ReconstructionBuilder', 'ReconstructionBuilderOptions', 'ReconstructionEstimator', 'ReconstructionEstimatorOptions', 'ReconstructionEstimatorSummary', 'ReconstructionEstimatorType', 'RelativePose', 'RelativePoseFromTwoPointsWithKnownRotation', 'RelativeRotationsFromViewGraph', 'RemoveDisconnectedViewPairs', 'RigidTransformation', 'RobustRotationEstimator', 'RobustRotationEstimatorOptions', 'RotationEstimator', 'SCHUR_JACOBI', 'SINGLE_LINKAGE', 'SKEW', 'SOFTLONE', 'SPARSE_NORMAL_CHOLESKY', 'SPARSE_SCHUR', 'SQPnP', 'SUITE_SPARSE', 'SVD', 'SelectGoodTracksForBundleAdjustment', 'SetCameraIntrinsicsFromPriors', 'SetOutlierTracksToUnestimated', 'SetReconstructionFromEstimatedPoses', 'SetUnderconstrainedTracksToUnestimated', 'SetUnderconstrainedViewsToUnestimated', 'SevenPointFundamentalMatrix', 'SharedFocalLengthsFromFundamentalMatrix', 'Sim3AlignmentOptions', 'Sim3AlignmentSummary', 'Sim3AlignmentType', 'Sim3FromRotationTranslationScale', 'Sim3ToHomogeneousMatrix', 'Sim3ToRotationTranslationScale', 'SimTransformPartialRotation', 'SimilarityTransformation', 'SparseLinearAlgebraLibraryType', 'SufficientTriangulationAngle', 'SwapCameras', 'TANGENTIAL_DISTORTION', 'TRIVIAL', 'TUKEY', 'ThreePointRelativePosePartialRotation', 'Track', 'TrackBuilder', 'TrackEstimator', 'TrackEstimatorOptions', 'TrackEstimatorSummary', 'TrackParametrizationType', 'TransformReconstruction', 'TransformReconstruction4', 'Triangulate', 'TriangulateDLT', 'TriangulateMidpoint', 'TriangulateNView', 'TriangulateNViewSVD', 'TriangulationMethodType', 'TwoPointPosePartialRotation', 'TwoViewBundleAdjustmentOptions', 'TwoViewInfo', 'UncalibratedAbsolutePose', 'UncalibratedRelativePose', 'UpdateFeaturesInView', 'View', 'ViewGraph', 'VisibilityClusteringType', 'VisibilityPyramid', 'XYZW', 'XYZW_MANIFOLD', 'kInvalidTrackId', 'kInvalidViewId']
class BundleAdjuster:
    @staticmethod
    def AddTrack(*args, **kwargs):
        ...
    @staticmethod
    def AddView(*args, **kwargs):
        ...
    @staticmethod
    def Optimize(*args, **kwargs):
        ...
class BundleAdjustmentOptions:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def constant_camera_orientation(*args, **kwargs):
        ...
    @constant_camera_orientation.setter
    def constant_camera_orientation(*args, **kwargs):
        ...
    @property
    def constant_camera_position(*args, **kwargs):
        ...
    @constant_camera_position.setter
    def constant_camera_position(*args, **kwargs):
        ...
    @property
    def dense_linear_algebra_library_type(*args, **kwargs):
        ...
    @dense_linear_algebra_library_type.setter
    def dense_linear_algebra_library_type(*args, **kwargs):
        ...
    @property
    def function_tolerance(*args, **kwargs):
        ...
    @function_tolerance.setter
    def function_tolerance(*args, **kwargs):
        ...
    @property
    def gradient_tolerance(*args, **kwargs):
        ...
    @gradient_tolerance.setter
    def gradient_tolerance(*args, **kwargs):
        ...
    @property
    def intrinsics_to_optimize(*args, **kwargs):
        ...
    @intrinsics_to_optimize.setter
    def intrinsics_to_optimize(*args, **kwargs):
        ...
    @property
    def linear_solver_type(*args, **kwargs):
        ...
    @linear_solver_type.setter
    def linear_solver_type(*args, **kwargs):
        ...
    @property
    def loss_function_type(*args, **kwargs):
        ...
    @loss_function_type.setter
    def loss_function_type(*args, **kwargs):
        ...
    @property
    def max_num_iterations(*args, **kwargs):
        ...
    @max_num_iterations.setter
    def max_num_iterations(*args, **kwargs):
        ...
    @property
    def max_num_refinement_iterations(*args, **kwargs):
        ...
    @max_num_refinement_iterations.setter
    def max_num_refinement_iterations(*args, **kwargs):
        ...
    @property
    def max_solver_time_in_seconds(*args, **kwargs):
        ...
    @max_solver_time_in_seconds.setter
    def max_solver_time_in_seconds(*args, **kwargs):
        ...
    @property
    def max_trust_region_radius(*args, **kwargs):
        ...
    @max_trust_region_radius.setter
    def max_trust_region_radius(*args, **kwargs):
        ...
    @property
    def num_threads(*args, **kwargs):
        ...
    @num_threads.setter
    def num_threads(*args, **kwargs):
        ...
    @property
    def optimize_for_forward_facing_trajectory(*args, **kwargs):
        ...
    @optimize_for_forward_facing_trajectory.setter
    def optimize_for_forward_facing_trajectory(*args, **kwargs):
        ...
    @property
    def orthographic_camera(*args, **kwargs):
        ...
    @orthographic_camera.setter
    def orthographic_camera(*args, **kwargs):
        ...
    @property
    def parameter_tolerance(*args, **kwargs):
        ...
    @parameter_tolerance.setter
    def parameter_tolerance(*args, **kwargs):
        ...
    @property
    def preconditioner_type(*args, **kwargs):
        ...
    @preconditioner_type.setter
    def preconditioner_type(*args, **kwargs):
        ...
    @property
    def robust_loss_width(*args, **kwargs):
        ...
    @robust_loss_width.setter
    def robust_loss_width(*args, **kwargs):
        ...
    @property
    def robust_loss_width_depth_prior(*args, **kwargs):
        ...
    @robust_loss_width_depth_prior.setter
    def robust_loss_width_depth_prior(*args, **kwargs):
        ...
    @property
    def sparse_linear_algebra_library_type(*args, **kwargs):
        ...
    @sparse_linear_algebra_library_type.setter
    def sparse_linear_algebra_library_type(*args, **kwargs):
        ...
    @property
    def use_depth_priors(*args, **kwargs):
        ...
    @use_depth_priors.setter
    def use_depth_priors(*args, **kwargs):
        ...
    @property
    def use_gravity_priors(*args, **kwargs):
        ...
    @use_gravity_priors.setter
    def use_gravity_priors(*args, **kwargs):
        ...
    @property
    def use_homogeneous_point_parametrization(*args, **kwargs):
        ...
    @use_homogeneous_point_parametrization.setter
    def use_homogeneous_point_parametrization(*args, **kwargs):
        ...
    @property
    def use_inner_iterations(*args, **kwargs):
        ...
    @use_inner_iterations.setter
    def use_inner_iterations(*args, **kwargs):
        ...
    @property
    def use_inverse_depth_parametrization(*args, **kwargs):
        ...
    @use_inverse_depth_parametrization.setter
    def use_inverse_depth_parametrization(*args, **kwargs):
        ...
    @property
    def use_mixed_precision_solves(*args, **kwargs):
        ...
    @use_mixed_precision_solves.setter
    def use_mixed_precision_solves(*args, **kwargs):
        ...
    @property
    def use_orientation_priors(*args, **kwargs):
        ...
    @use_orientation_priors.setter
    def use_orientation_priors(*args, **kwargs):
        ...
    @property
    def use_position_priors(*args, **kwargs):
        ...
    @use_position_priors.setter
    def use_position_priors(*args, **kwargs):
        ...
    @property
    def verbose(*args, **kwargs):
        ...
    @verbose.setter
    def verbose(*args, **kwargs):
        ...
    @property
    def visibility_clustering_type(*args, **kwargs):
        ...
    @visibility_clustering_type.setter
    def visibility_clustering_type(*args, **kwargs):
        ...
class BundleAdjustmentSummary:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def final_cost(*args, **kwargs):
        ...
    @final_cost.setter
    def final_cost(*args, **kwargs):
        ...
    @property
    def initial_cost(*args, **kwargs):
        ...
    @initial_cost.setter
    def initial_cost(*args, **kwargs):
        ...
    @property
    def setup_time_in_seconds(*args, **kwargs):
        ...
    @setup_time_in_seconds.setter
    def setup_time_in_seconds(*args, **kwargs):
        ...
    @property
    def solve_time_in_seconds(*args, **kwargs):
        ...
    @solve_time_in_seconds.setter
    def solve_time_in_seconds(*args, **kwargs):
        ...
    @property
    def success(*args, **kwargs):
        ...
    @success.setter
    def success(*args, **kwargs):
        ...
class CalibratedAbsolutePose:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def position(*args, **kwargs):
        ...
    @position.setter
    def position(*args, **kwargs):
        ...
    @property
    def rotation(*args, **kwargs):
        ...
    @rotation.setter
    def rotation(*args, **kwargs):
        ...
class Camera:
    @staticmethod
    def CameraIntrinsics(*args, **kwargs):
        ...
    @staticmethod
    def CameraIntrinsicsPriorFromIntrinsics(*args, **kwargs):
        ...
    @staticmethod
    def DeepCopy(*args, **kwargs):
        ...
    @staticmethod
    def FocalLength(*args, **kwargs):
        ...
    @staticmethod
    def GetCalibrationMatrix(*args, **kwargs):
        ...
    @staticmethod
    def GetCameraIntrinsicsModelType(*args, **kwargs):
        ...
    @staticmethod
    def GetOrientationAsAngleAxis(*args, **kwargs):
        ...
    @staticmethod
    def GetOrientationAsRotationMatrix(*args, **kwargs):
        ...
    @staticmethod
    def GetPosition(*args, **kwargs):
        ...
    @staticmethod
    def GetProjectionMatrix(*args, **kwargs):
        ...
    @staticmethod
    def ImageHeight(*args, **kwargs):
        ...
    @staticmethod
    def ImageWidth(*args, **kwargs):
        ...
    @staticmethod
    def InitializeFromProjectionMatrix(*args, **kwargs):
        ...
    @staticmethod
    def PixelToNormalizedCoordinates(*args, **kwargs):
        ...
    @staticmethod
    def PixelToUnitDepthRay(*args, **kwargs):
        ...
    @staticmethod
    def PrincipalPointX(*args, **kwargs):
        ...
    @staticmethod
    def PrincipalPointY(*args, **kwargs):
        ...
    @staticmethod
    def PrintCameraIntrinsics(*args, **kwargs):
        ...
    @staticmethod
    def ProjectPoint(*args, **kwargs):
        ...
    @staticmethod
    def SetCameraIntrinsicsModelType(*args, **kwargs):
        ...
    @staticmethod
    def SetFocalLength(*args, **kwargs):
        ...
    @staticmethod
    def SetFromCameraIntrinsicsPriors(*args, **kwargs):
        ...
    @staticmethod
    def SetImageSize(*args, **kwargs):
        ...
    @staticmethod
    def SetOrientationFromAngleAxis(*args, **kwargs):
        ...
    @staticmethod
    def SetOrientationFromRotationMatrix(*args, **kwargs):
        ...
    @staticmethod
    def SetPosition(*args, **kwargs):
        ...
    @staticmethod
    def SetPrincipalPoint(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class CameraAndFeatureCorrespondence2D3D:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def camera(*args, **kwargs):
        ...
    @camera.setter
    def camera(*args, **kwargs):
        ...
    @property
    def observation(*args, **kwargs):
        ...
    @observation.setter
    def observation(*args, **kwargs):
        ...
    @property
    def point3d(*args, **kwargs):
        ...
    @point3d.setter
    def point3d(*args, **kwargs):
        ...
class CameraIntrinsicsModel:
    @staticmethod
    def CameraToImageCoordinates(*args, **kwargs):
        ...
    @staticmethod
    def FocalLength(*args, **kwargs):
        ...
    @staticmethod
    def GetParameter(*args, **kwargs):
        ...
    @staticmethod
    def ImageToCameraCoordinates(*args, **kwargs):
        ...
    @staticmethod
    def PrincipalPointX(*args, **kwargs):
        ...
    @staticmethod
    def PrincipalPointY(*args, **kwargs):
        ...
    @staticmethod
    def SetFocalLength(*args, **kwargs):
        ...
    @staticmethod
    def SetParameter(*args, **kwargs):
        ...
    @staticmethod
    def SetPrincipalPoint(*args, **kwargs):
        ...
class CameraIntrinsicsModelType:
    """
    Members:
    
      INVALID
    
      PINHOLE
    
      PINHOLE_RADIAL_TANGENTIAL
    
      FISHEYE
    
      FOV
    
      DIVISION_UNDISTORTION
    
      DOUBLE_SPHERE
    
      EXTENDED_UNIFIED
    
      ORTHOGRAPHIC
    """
    DIVISION_UNDISTORTION: typing.ClassVar[CameraIntrinsicsModelType]  # value = <CameraIntrinsicsModelType.DIVISION_UNDISTORTION: 4>
    DOUBLE_SPHERE: typing.ClassVar[CameraIntrinsicsModelType]  # value = <CameraIntrinsicsModelType.DOUBLE_SPHERE: 5>
    EXTENDED_UNIFIED: typing.ClassVar[CameraIntrinsicsModelType]  # value = <CameraIntrinsicsModelType.EXTENDED_UNIFIED: 6>
    FISHEYE: typing.ClassVar[CameraIntrinsicsModelType]  # value = <CameraIntrinsicsModelType.FISHEYE: 2>
    FOV: typing.ClassVar[CameraIntrinsicsModelType]  # value = <CameraIntrinsicsModelType.FOV: 3>
    INVALID: typing.ClassVar[CameraIntrinsicsModelType]  # value = <CameraIntrinsicsModelType.INVALID: -1>
    ORTHOGRAPHIC: typing.ClassVar[CameraIntrinsicsModelType]  # value = <CameraIntrinsicsModelType.ORTHOGRAPHIC: 7>
    PINHOLE: typing.ClassVar[CameraIntrinsicsModelType]  # value = <CameraIntrinsicsModelType.PINHOLE: 0>
    PINHOLE_RADIAL_TANGENTIAL: typing.ClassVar[CameraIntrinsicsModelType]  # value = <CameraIntrinsicsModelType.PINHOLE_RADIAL_TANGENTIAL: 1>
    __members__: typing.ClassVar[dict[str, CameraIntrinsicsModelType]]  # value = {'INVALID': <CameraIntrinsicsModelType.INVALID: -1>, 'PINHOLE': <CameraIntrinsicsModelType.PINHOLE: 0>, 'PINHOLE_RADIAL_TANGENTIAL': <CameraIntrinsicsModelType.PINHOLE_RADIAL_TANGENTIAL: 1>, 'FISHEYE': <CameraIntrinsicsModelType.FISHEYE: 2>, 'FOV': <CameraIntrinsicsModelType.FOV: 3>, 'DIVISION_UNDISTORTION': <CameraIntrinsicsModelType.DIVISION_UNDISTORTION: 4>, 'DOUBLE_SPHERE': <CameraIntrinsicsModelType.DOUBLE_SPHERE: 5>, 'EXTENDED_UNIFIED': <CameraIntrinsicsModelType.EXTENDED_UNIFIED: 6>, 'ORTHOGRAPHIC': <CameraIntrinsicsModelType.ORTHOGRAPHIC: 7>}
    @staticmethod
    def __eq__(*args, **kwargs):
        ...
    @staticmethod
    def __getstate__(*args, **kwargs):
        ...
    @staticmethod
    def __hash__(*args, **kwargs):
        ...
    @staticmethod
    def __index__(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @staticmethod
    def __int__(*args, **kwargs):
        ...
    @staticmethod
    def __ne__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        ...
    @staticmethod
    def __setstate__(*args, **kwargs):
        ...
    @staticmethod
    def __str__(*args, **kwargs):
        ...
    @property
    def name(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
class CameraIntrinsicsPrior:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def altitude(*args, **kwargs):
        ...
    @altitude.setter
    def altitude(*args, **kwargs):
        ...
    @property
    def aspect_ratio(*args, **kwargs):
        ...
    @aspect_ratio.setter
    def aspect_ratio(*args, **kwargs):
        ...
    @property
    def camera_intrinsics_model_type(*args, **kwargs):
        ...
    @camera_intrinsics_model_type.setter
    def camera_intrinsics_model_type(*args, **kwargs):
        ...
    @property
    def focal_length(*args, **kwargs):
        ...
    @focal_length.setter
    def focal_length(*args, **kwargs):
        ...
    @property
    def image_height(*args, **kwargs):
        ...
    @image_height.setter
    def image_height(*args, **kwargs):
        ...
    @property
    def image_width(*args, **kwargs):
        ...
    @image_width.setter
    def image_width(*args, **kwargs):
        ...
    @property
    def latitude(*args, **kwargs):
        ...
    @latitude.setter
    def latitude(*args, **kwargs):
        ...
    @property
    def longitude(*args, **kwargs):
        ...
    @longitude.setter
    def longitude(*args, **kwargs):
        ...
    @property
    def orientation(*args, **kwargs):
        ...
    @orientation.setter
    def orientation(*args, **kwargs):
        ...
    @property
    def position(*args, **kwargs):
        ...
    @position.setter
    def position(*args, **kwargs):
        ...
    @property
    def principal_point(*args, **kwargs):
        ...
    @principal_point.setter
    def principal_point(*args, **kwargs):
        ...
    @property
    def radial_distortion(*args, **kwargs):
        ...
    @radial_distortion.setter
    def radial_distortion(*args, **kwargs):
        ...
    @property
    def skew(*args, **kwargs):
        ...
    @skew.setter
    def skew(*args, **kwargs):
        ...
    @property
    def tangential_distortion(*args, **kwargs):
        ...
    @tangential_distortion.setter
    def tangential_distortion(*args, **kwargs):
        ...
class DenseLinearAlgebraLibraryType:
    """
    Members:
    
      EIGEN
    
      LAPACK
    
      CUDA
    """
    CUDA: typing.ClassVar[DenseLinearAlgebraLibraryType]  # value = <DenseLinearAlgebraLibraryType.CUDA: 2>
    EIGEN: typing.ClassVar[DenseLinearAlgebraLibraryType]  # value = <DenseLinearAlgebraLibraryType.EIGEN: 0>
    LAPACK: typing.ClassVar[DenseLinearAlgebraLibraryType]  # value = <DenseLinearAlgebraLibraryType.LAPACK: 1>
    __members__: typing.ClassVar[dict[str, DenseLinearAlgebraLibraryType]]  # value = {'EIGEN': <DenseLinearAlgebraLibraryType.EIGEN: 0>, 'LAPACK': <DenseLinearAlgebraLibraryType.LAPACK: 1>, 'CUDA': <DenseLinearAlgebraLibraryType.CUDA: 2>}
    @staticmethod
    def __eq__(*args, **kwargs):
        ...
    @staticmethod
    def __getstate__(*args, **kwargs):
        ...
    @staticmethod
    def __hash__(*args, **kwargs):
        ...
    @staticmethod
    def __index__(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @staticmethod
    def __int__(*args, **kwargs):
        ...
    @staticmethod
    def __ne__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        ...
    @staticmethod
    def __setstate__(*args, **kwargs):
        ...
    @staticmethod
    def __str__(*args, **kwargs):
        ...
    @property
    def name(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
class DivisionUndistortionCameraModel(CameraIntrinsicsModel):
    @staticmethod
    def AspectRatio(*args, **kwargs):
        ...
    @staticmethod
    def CameraIntrinsicsPriorFromIntrinsics(*args, **kwargs):
        ...
    @staticmethod
    def GetCalibrationMatrix(*args, **kwargs):
        ...
    @staticmethod
    def GetSubsetFromOptimizeIntrinsicsType(*args, **kwargs):
        ...
    @staticmethod
    def NumParameters(*args, **kwargs):
        ...
    @staticmethod
    def PrintIntrinsics(*args, **kwargs):
        ...
    @staticmethod
    def RadialDistortion1(*args, **kwargs):
        ...
    @staticmethod
    def SetAspectRatio(*args, **kwargs):
        ...
    @staticmethod
    def SetFromCameraIntrinsicsPriors(*args, **kwargs):
        ...
    @staticmethod
    def SetRadialDistortion(*args, **kwargs):
        ...
    @staticmethod
    def Type(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def kIntrinsicsSize(*args, **kwargs):
        ...
class EstimateTwoViewInfoOptions:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def expected_ransac_confidence(*args, **kwargs):
        ...
    @expected_ransac_confidence.setter
    def expected_ransac_confidence(*args, **kwargs):
        ...
    @property
    def lo_start_iterations(*args, **kwargs):
        ...
    @lo_start_iterations.setter
    def lo_start_iterations(*args, **kwargs):
        ...
    @property
    def max_focal_length(*args, **kwargs):
        ...
    @max_focal_length.setter
    def max_focal_length(*args, **kwargs):
        ...
    @property
    def max_ransac_iterations(*args, **kwargs):
        ...
    @max_ransac_iterations.setter
    def max_ransac_iterations(*args, **kwargs):
        ...
    @property
    def max_sampson_error_pixels(*args, **kwargs):
        ...
    @max_sampson_error_pixels.setter
    def max_sampson_error_pixels(*args, **kwargs):
        ...
    @property
    def min_focal_length(*args, **kwargs):
        ...
    @min_focal_length.setter
    def min_focal_length(*args, **kwargs):
        ...
    @property
    def min_ransac_iterations(*args, **kwargs):
        ...
    @min_ransac_iterations.setter
    def min_ransac_iterations(*args, **kwargs):
        ...
    @property
    def ransac_type(*args, **kwargs):
        ...
    @ransac_type.setter
    def ransac_type(*args, **kwargs):
        ...
    @property
    def use_lo(*args, **kwargs):
        ...
    @use_lo.setter
    def use_lo(*args, **kwargs):
        ...
    @property
    def use_mle(*args, **kwargs):
        ...
    @use_mle.setter
    def use_mle(*args, **kwargs):
        ...
class FOVCameraModel(CameraIntrinsicsModel):
    @staticmethod
    def AspectRatio(*args, **kwargs):
        ...
    @staticmethod
    def CameraIntrinsicsPriorFromIntrinsics(*args, **kwargs):
        ...
    @staticmethod
    def GetCalibrationMatrix(*args, **kwargs):
        ...
    @staticmethod
    def GetSubsetFromOptimizeIntrinsicsType(*args, **kwargs):
        ...
    @staticmethod
    def NumParameters(*args, **kwargs):
        ...
    @staticmethod
    def PrintIntrinsics(*args, **kwargs):
        ...
    @staticmethod
    def RadialDistortion1(*args, **kwargs):
        ...
    @staticmethod
    def SetAspectRatio(*args, **kwargs):
        ...
    @staticmethod
    def SetFromCameraIntrinsicsPriors(*args, **kwargs):
        ...
    @staticmethod
    def SetRadialDistortion(*args, **kwargs):
        ...
    @staticmethod
    def Type(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def kIntrinsicsSize(*args, **kwargs):
        ...
class Feature:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @staticmethod
    def get_depth_prior(*args, **kwargs):
        ...
    @staticmethod
    def get_depth_prior_variance(*args, **kwargs):
        ...
    @staticmethod
    def x(*args, **kwargs):
        ...
    @staticmethod
    def y(*args, **kwargs):
        ...
    @property
    def covariance(*args, **kwargs):
        ...
    @covariance.setter
    def covariance(*args, **kwargs):
        ...
    @property
    def depth_prior(*args, **kwargs):
        ...
    @depth_prior.setter
    def depth_prior(*args, **kwargs):
        ...
    @property
    def depth_prior_variance(*args, **kwargs):
        ...
    @depth_prior_variance.setter
    def depth_prior_variance(*args, **kwargs):
        ...
    @property
    def point(*args, **kwargs):
        ...
    @point.setter
    def point(*args, **kwargs):
        ...
class FeatureCorrespondence2D3D:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def feature(*args, **kwargs):
        ...
    @feature.setter
    def feature(*args, **kwargs):
        ...
    @property
    def world_point(*args, **kwargs):
        ...
    @world_point.setter
    def world_point(*args, **kwargs):
        ...
class FilterViewPairsFromRelativeTranslationOptions:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def num_iterations(*args, **kwargs):
        ...
    @num_iterations.setter
    def num_iterations(*args, **kwargs):
        ...
    @property
    def num_threads(*args, **kwargs):
        ...
    @num_threads.setter
    def num_threads(*args, **kwargs):
        ...
    @property
    def translation_projection_tolerance(*args, **kwargs):
        ...
    @translation_projection_tolerance.setter
    def translation_projection_tolerance(*args, **kwargs):
        ...
class FisheyeCameraModel(CameraIntrinsicsModel):
    @staticmethod
    def AspectRatio(*args, **kwargs):
        ...
    @staticmethod
    def CameraIntrinsicsPriorFromIntrinsics(*args, **kwargs):
        ...
    @staticmethod
    def GetCalibrationMatrix(*args, **kwargs):
        ...
    @staticmethod
    def GetSubsetFromOptimizeIntrinsicsType(*args, **kwargs):
        ...
    @staticmethod
    def NumParameters(*args, **kwargs):
        ...
    @staticmethod
    def PrintIntrinsics(*args, **kwargs):
        ...
    @staticmethod
    def RadialDistortion1(*args, **kwargs):
        ...
    @staticmethod
    def RadialDistortion2(*args, **kwargs):
        ...
    @staticmethod
    def RadialDistortion3(*args, **kwargs):
        ...
    @staticmethod
    def RadialDistortion4(*args, **kwargs):
        ...
    @staticmethod
    def SetAspectRatio(*args, **kwargs):
        ...
    @staticmethod
    def SetFromCameraIntrinsicsPriors(*args, **kwargs):
        ...
    @staticmethod
    def SetRadialDistortion(*args, **kwargs):
        ...
    @staticmethod
    def SetSkew(*args, **kwargs):
        ...
    @staticmethod
    def Skew(*args, **kwargs):
        ...
    @staticmethod
    def Type(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def kIntrinsicsSize(*args, **kwargs):
        ...
class GPSConverter:
    @staticmethod
    def ECEFToLLA(*args, **kwargs):
        ...
    @staticmethod
    def LLAToECEF(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class GlobalPositionEstimatorType:
    """
    Members:
    
      NONLINEAR
    
      LINEAR_TRIPLET
    
      LEAST_UNSQUARED_DEVIATION
    
      LIGT
    """
    LEAST_UNSQUARED_DEVIATION: typing.ClassVar[GlobalPositionEstimatorType]  # value = <GlobalPositionEstimatorType.LEAST_UNSQUARED_DEVIATION: 2>
    LIGT: typing.ClassVar[GlobalPositionEstimatorType]  # value = <GlobalPositionEstimatorType.LIGT: 3>
    LINEAR_TRIPLET: typing.ClassVar[GlobalPositionEstimatorType]  # value = <GlobalPositionEstimatorType.LINEAR_TRIPLET: 1>
    NONLINEAR: typing.ClassVar[GlobalPositionEstimatorType]  # value = <GlobalPositionEstimatorType.NONLINEAR: 0>
    __members__: typing.ClassVar[dict[str, GlobalPositionEstimatorType]]  # value = {'NONLINEAR': <GlobalPositionEstimatorType.NONLINEAR: 0>, 'LINEAR_TRIPLET': <GlobalPositionEstimatorType.LINEAR_TRIPLET: 1>, 'LEAST_UNSQUARED_DEVIATION': <GlobalPositionEstimatorType.LEAST_UNSQUARED_DEVIATION: 2>, 'LIGT': <GlobalPositionEstimatorType.LIGT: 3>}
    @staticmethod
    def __eq__(*args, **kwargs):
        ...
    @staticmethod
    def __getstate__(*args, **kwargs):
        ...
    @staticmethod
    def __hash__(*args, **kwargs):
        ...
    @staticmethod
    def __index__(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @staticmethod
    def __int__(*args, **kwargs):
        ...
    @staticmethod
    def __ne__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        ...
    @staticmethod
    def __setstate__(*args, **kwargs):
        ...
    @staticmethod
    def __str__(*args, **kwargs):
        ...
    @property
    def name(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
class GlobalReconstructionEstimator(ReconstructionEstimator):
    @staticmethod
    def Estimate(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class GlobalRotationEstimatorType:
    """
    Members:
    
      ROBUST_L1L2
    
      NONLINEAR
    
      LINEAR
    
      LAGRANGE_DUAL
    
      HYBRID
    """
    HYBRID: typing.ClassVar[GlobalRotationEstimatorType]  # value = <GlobalRotationEstimatorType.HYBRID: 4>
    LAGRANGE_DUAL: typing.ClassVar[GlobalRotationEstimatorType]  # value = <GlobalRotationEstimatorType.LAGRANGE_DUAL: 3>
    LINEAR: typing.ClassVar[GlobalRotationEstimatorType]  # value = <GlobalRotationEstimatorType.LINEAR: 2>
    NONLINEAR: typing.ClassVar[GlobalRotationEstimatorType]  # value = <GlobalRotationEstimatorType.NONLINEAR: 1>
    ROBUST_L1L2: typing.ClassVar[GlobalRotationEstimatorType]  # value = <GlobalRotationEstimatorType.ROBUST_L1L2: 0>
    __members__: typing.ClassVar[dict[str, GlobalRotationEstimatorType]]  # value = {'ROBUST_L1L2': <GlobalRotationEstimatorType.ROBUST_L1L2: 0>, 'NONLINEAR': <GlobalRotationEstimatorType.NONLINEAR: 1>, 'LINEAR': <GlobalRotationEstimatorType.LINEAR: 2>, 'LAGRANGE_DUAL': <GlobalRotationEstimatorType.LAGRANGE_DUAL: 3>, 'HYBRID': <GlobalRotationEstimatorType.HYBRID: 4>}
    @staticmethod
    def __eq__(*args, **kwargs):
        ...
    @staticmethod
    def __getstate__(*args, **kwargs):
        ...
    @staticmethod
    def __hash__(*args, **kwargs):
        ...
    @staticmethod
    def __index__(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @staticmethod
    def __int__(*args, **kwargs):
        ...
    @staticmethod
    def __ne__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        ...
    @staticmethod
    def __setstate__(*args, **kwargs):
        ...
    @staticmethod
    def __str__(*args, **kwargs):
        ...
    @property
    def name(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
class HybridReconstructionEstimator(ReconstructionEstimator):
    @staticmethod
    def Estimate(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class HybridRotationEstimator(RotationEstimator):
    @staticmethod
    def EstimateRotations(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class IncrementalReconstructionEstimator(ReconstructionEstimator):
    @staticmethod
    def Estimate(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class LagrangeDualRotationEstimator(RotationEstimator):
    @staticmethod
    def EstimateRotations(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class LeastUnsquaredDeviationPositionEstimator(PositionEstimator):
    @staticmethod
    def EstimatePositions(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class LeastUnsquaredDeviationPositionEstimatorOptions:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def convergence_criterion(*args, **kwargs):
        ...
    @convergence_criterion.setter
    def convergence_criterion(*args, **kwargs):
        ...
    @property
    def max_num_iterations(*args, **kwargs):
        ...
    @max_num_iterations.setter
    def max_num_iterations(*args, **kwargs):
        ...
    @property
    def max_num_reweighted_iterations(*args, **kwargs):
        ...
    @max_num_reweighted_iterations.setter
    def max_num_reweighted_iterations(*args, **kwargs):
        ...
class LiGTPositionEstimator(PositionEstimator):
    @staticmethod
    def EstimatePositions(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class LiGTPositionEstimatorOptions:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def eigensolver_threshold(*args, **kwargs):
        ...
    @eigensolver_threshold.setter
    def eigensolver_threshold(*args, **kwargs):
        ...
    @property
    def max_num_views_svd(*args, **kwargs):
        ...
    @max_num_views_svd.setter
    def max_num_views_svd(*args, **kwargs):
        ...
    @property
    def max_power_iterations(*args, **kwargs):
        ...
    @max_power_iterations.setter
    def max_power_iterations(*args, **kwargs):
        ...
    @property
    def num_threads(*args, **kwargs):
        ...
    @num_threads.setter
    def num_threads(*args, **kwargs):
        ...
class LinearPositionEstimator(PositionEstimator):
    @staticmethod
    def EstimatePositions(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class LinearPositionEstimatorOptions:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def eigensolver_threshold(*args, **kwargs):
        ...
    @eigensolver_threshold.setter
    def eigensolver_threshold(*args, **kwargs):
        ...
    @property
    def max_power_iterations(*args, **kwargs):
        ...
    @max_power_iterations.setter
    def max_power_iterations(*args, **kwargs):
        ...
    @property
    def num_threads(*args, **kwargs):
        ...
    @num_threads.setter
    def num_threads(*args, **kwargs):
        ...
class LinearRotationEstimator(RotationEstimator):
    @staticmethod
    def AddRelativeRotationConstraint(*args, **kwargs):
        ...
    @staticmethod
    def EstimateRotations(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class LinearSolverType:
    """
    Members:
    
      DENSE_QR
    
      DENSE_NORMAL_CHOLESKY
    
      DENSE_SCHUR
    
      SPARSE_NORMAL_CHOLESKY
    
      SPARSE_SCHUR
    
      ITERATIVE_SCHUR
    
      CGNR
    """
    CGNR: typing.ClassVar[LinearSolverType]  # value = <LinearSolverType.CGNR: 6>
    DENSE_NORMAL_CHOLESKY: typing.ClassVar[LinearSolverType]  # value = <LinearSolverType.DENSE_NORMAL_CHOLESKY: 0>
    DENSE_QR: typing.ClassVar[LinearSolverType]  # value = <LinearSolverType.DENSE_QR: 1>
    DENSE_SCHUR: typing.ClassVar[LinearSolverType]  # value = <LinearSolverType.DENSE_SCHUR: 3>
    ITERATIVE_SCHUR: typing.ClassVar[LinearSolverType]  # value = <LinearSolverType.ITERATIVE_SCHUR: 5>
    SPARSE_NORMAL_CHOLESKY: typing.ClassVar[LinearSolverType]  # value = <LinearSolverType.SPARSE_NORMAL_CHOLESKY: 2>
    SPARSE_SCHUR: typing.ClassVar[LinearSolverType]  # value = <LinearSolverType.SPARSE_SCHUR: 4>
    __members__: typing.ClassVar[dict[str, LinearSolverType]]  # value = {'DENSE_QR': <LinearSolverType.DENSE_QR: 1>, 'DENSE_NORMAL_CHOLESKY': <LinearSolverType.DENSE_NORMAL_CHOLESKY: 0>, 'DENSE_SCHUR': <LinearSolverType.DENSE_SCHUR: 3>, 'SPARSE_NORMAL_CHOLESKY': <LinearSolverType.SPARSE_NORMAL_CHOLESKY: 2>, 'SPARSE_SCHUR': <LinearSolverType.SPARSE_SCHUR: 4>, 'ITERATIVE_SCHUR': <LinearSolverType.ITERATIVE_SCHUR: 5>, 'CGNR': <LinearSolverType.CGNR: 6>}
    @staticmethod
    def __eq__(*args, **kwargs):
        ...
    @staticmethod
    def __getstate__(*args, **kwargs):
        ...
    @staticmethod
    def __hash__(*args, **kwargs):
        ...
    @staticmethod
    def __index__(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @staticmethod
    def __int__(*args, **kwargs):
        ...
    @staticmethod
    def __ne__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        ...
    @staticmethod
    def __setstate__(*args, **kwargs):
        ...
    @staticmethod
    def __str__(*args, **kwargs):
        ...
    @property
    def name(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
class LocalizeViewToReconstructionOptions:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def assume_known_orientation(*args, **kwargs):
        ...
    @assume_known_orientation.setter
    def assume_known_orientation(*args, **kwargs):
        ...
    @property
    def ba_options(*args, **kwargs):
        ...
    @ba_options.setter
    def ba_options(*args, **kwargs):
        ...
    @property
    def bundle_adjust_view(*args, **kwargs):
        ...
    @bundle_adjust_view.setter
    def bundle_adjust_view(*args, **kwargs):
        ...
    @property
    def min_num_inliers(*args, **kwargs):
        ...
    @min_num_inliers.setter
    def min_num_inliers(*args, **kwargs):
        ...
    @property
    def pnp_type(*args, **kwargs):
        ...
    @pnp_type.setter
    def pnp_type(*args, **kwargs):
        ...
    @property
    def ransac_params(*args, **kwargs):
        ...
    @ransac_params.setter
    def ransac_params(*args, **kwargs):
        ...
    @property
    def reprojection_error_threshold_pixels(*args, **kwargs):
        ...
    @reprojection_error_threshold_pixels.setter
    def reprojection_error_threshold_pixels(*args, **kwargs):
        ...
class LossFunctionType:
    """
    Members:
    
      TRIVIAL
    
      HUBER
    
      SOFTLONE
    
      CAUCHY
    
      ARCTAN
    
      TUKEY
    """
    ARCTAN: typing.ClassVar[LossFunctionType]  # value = <LossFunctionType.ARCTAN: 4>
    CAUCHY: typing.ClassVar[LossFunctionType]  # value = <LossFunctionType.CAUCHY: 3>
    HUBER: typing.ClassVar[LossFunctionType]  # value = <LossFunctionType.HUBER: 1>
    SOFTLONE: typing.ClassVar[LossFunctionType]  # value = <LossFunctionType.SOFTLONE: 2>
    TRIVIAL: typing.ClassVar[LossFunctionType]  # value = <LossFunctionType.TRIVIAL: 0>
    TUKEY: typing.ClassVar[LossFunctionType]  # value = <LossFunctionType.TUKEY: 5>
    __members__: typing.ClassVar[dict[str, LossFunctionType]]  # value = {'TRIVIAL': <LossFunctionType.TRIVIAL: 0>, 'HUBER': <LossFunctionType.HUBER: 1>, 'SOFTLONE': <LossFunctionType.SOFTLONE: 2>, 'CAUCHY': <LossFunctionType.CAUCHY: 3>, 'ARCTAN': <LossFunctionType.ARCTAN: 4>, 'TUKEY': <LossFunctionType.TUKEY: 5>}
    @staticmethod
    def __eq__(*args, **kwargs):
        ...
    @staticmethod
    def __getstate__(*args, **kwargs):
        ...
    @staticmethod
    def __hash__(*args, **kwargs):
        ...
    @staticmethod
    def __index__(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @staticmethod
    def __int__(*args, **kwargs):
        ...
    @staticmethod
    def __ne__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        ...
    @staticmethod
    def __setstate__(*args, **kwargs):
        ...
    @staticmethod
    def __str__(*args, **kwargs):
        ...
    @property
    def name(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
class NonlinearPositionEstimator(PositionEstimator):
    @staticmethod
    def EstimatePositions(*args, **kwargs):
        ...
    @staticmethod
    def EstimateRemainingPositionsInRecon(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class NonlinearPositionEstimatorOptions:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def eigensolver_threshold(*args, **kwargs):
        ...
    @eigensolver_threshold.setter
    def eigensolver_threshold(*args, **kwargs):
        ...
    @property
    def max_power_iterations(*args, **kwargs):
        ...
    @max_power_iterations.setter
    def max_power_iterations(*args, **kwargs):
        ...
    @property
    def min_num_points_per_view(*args, **kwargs):
        ...
    @min_num_points_per_view.setter
    def min_num_points_per_view(*args, **kwargs):
        ...
    @property
    def num_threads(*args, **kwargs):
        ...
    @num_threads.setter
    def num_threads(*args, **kwargs):
        ...
    @property
    def point_to_camera_weight(*args, **kwargs):
        ...
    @point_to_camera_weight.setter
    def point_to_camera_weight(*args, **kwargs):
        ...
class NonlinearRotationEstimator(RotationEstimator):
    @staticmethod
    def EstimateRotations(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class OptimizeIntrinsicsType:
    """
    Members:
    
      NONE
    
      FOCAL_LENGTH
    
      ASPECT_RATIO
    
      SKEW
    
      PRINCIPAL_POINTS
    
      RADIAL_DISTORTION
    
      TANGENTIAL_DISTORTION
    
      DISTORTION
    
      FOCAL_LENGTH_DISTORTION
    
      FOCAL_LENGTH_RADIAL_DISTORTION
    
      ALL
    """
    ALL: typing.ClassVar[OptimizeIntrinsicsType]  # value = <OptimizeIntrinsicsType.ALL: 63>
    ASPECT_RATIO: typing.ClassVar[OptimizeIntrinsicsType]  # value = <OptimizeIntrinsicsType.ASPECT_RATIO: 2>
    DISTORTION: typing.ClassVar[OptimizeIntrinsicsType]  # value = <OptimizeIntrinsicsType.DISTORTION: 48>
    FOCAL_LENGTH: typing.ClassVar[OptimizeIntrinsicsType]  # value = <OptimizeIntrinsicsType.FOCAL_LENGTH: 1>
    FOCAL_LENGTH_DISTORTION: typing.ClassVar[OptimizeIntrinsicsType]  # value = <OptimizeIntrinsicsType.FOCAL_LENGTH_DISTORTION: 49>
    FOCAL_LENGTH_RADIAL_DISTORTION: typing.ClassVar[OptimizeIntrinsicsType]  # value = <OptimizeIntrinsicsType.FOCAL_LENGTH_RADIAL_DISTORTION: 17>
    NONE: typing.ClassVar[OptimizeIntrinsicsType]  # value = <OptimizeIntrinsicsType.NONE: 0>
    PRINCIPAL_POINTS: typing.ClassVar[OptimizeIntrinsicsType]  # value = <OptimizeIntrinsicsType.PRINCIPAL_POINTS: 8>
    RADIAL_DISTORTION: typing.ClassVar[OptimizeIntrinsicsType]  # value = <OptimizeIntrinsicsType.RADIAL_DISTORTION: 16>
    SKEW: typing.ClassVar[OptimizeIntrinsicsType]  # value = <OptimizeIntrinsicsType.SKEW: 4>
    TANGENTIAL_DISTORTION: typing.ClassVar[OptimizeIntrinsicsType]  # value = <OptimizeIntrinsicsType.TANGENTIAL_DISTORTION: 32>
    __members__: typing.ClassVar[dict[str, OptimizeIntrinsicsType]]  # value = {'NONE': <OptimizeIntrinsicsType.NONE: 0>, 'FOCAL_LENGTH': <OptimizeIntrinsicsType.FOCAL_LENGTH: 1>, 'ASPECT_RATIO': <OptimizeIntrinsicsType.ASPECT_RATIO: 2>, 'SKEW': <OptimizeIntrinsicsType.SKEW: 4>, 'PRINCIPAL_POINTS': <OptimizeIntrinsicsType.PRINCIPAL_POINTS: 8>, 'RADIAL_DISTORTION': <OptimizeIntrinsicsType.RADIAL_DISTORTION: 16>, 'TANGENTIAL_DISTORTION': <OptimizeIntrinsicsType.TANGENTIAL_DISTORTION: 32>, 'DISTORTION': <OptimizeIntrinsicsType.DISTORTION: 48>, 'FOCAL_LENGTH_DISTORTION': <OptimizeIntrinsicsType.FOCAL_LENGTH_DISTORTION: 49>, 'FOCAL_LENGTH_RADIAL_DISTORTION': <OptimizeIntrinsicsType.FOCAL_LENGTH_RADIAL_DISTORTION: 17>, 'ALL': <OptimizeIntrinsicsType.ALL: 63>}
    @staticmethod
    def __eq__(*args, **kwargs):
        ...
    @staticmethod
    def __getstate__(*args, **kwargs):
        ...
    @staticmethod
    def __hash__(*args, **kwargs):
        ...
    @staticmethod
    def __index__(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @staticmethod
    def __int__(*args, **kwargs):
        ...
    @staticmethod
    def __ne__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        ...
    @staticmethod
    def __setstate__(*args, **kwargs):
        ...
    @staticmethod
    def __str__(*args, **kwargs):
        ...
    @property
    def name(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
class OrthographicCameraModel(CameraIntrinsicsModel):
    @staticmethod
    def AspectRatio(*args, **kwargs):
        ...
    @staticmethod
    def CameraIntrinsicsPriorFromIntrinsics(*args, **kwargs):
        ...
    @staticmethod
    def GetCalibrationMatrix(*args, **kwargs):
        ...
    @staticmethod
    def GetSubsetFromOptimizeIntrinsicsType(*args, **kwargs):
        ...
    @staticmethod
    def NumParameters(*args, **kwargs):
        ...
    @staticmethod
    def PrintIntrinsics(*args, **kwargs):
        ...
    @staticmethod
    def RadialDistortion1(*args, **kwargs):
        ...
    @staticmethod
    def RadialDistortion2(*args, **kwargs):
        ...
    @staticmethod
    def SetAspectRatio(*args, **kwargs):
        ...
    @staticmethod
    def SetFromCameraIntrinsicsPriors(*args, **kwargs):
        ...
    @staticmethod
    def SetRadialDistortion(*args, **kwargs):
        ...
    @staticmethod
    def SetSkew(*args, **kwargs):
        ...
    @staticmethod
    def Skew(*args, **kwargs):
        ...
    @staticmethod
    def Type(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def kIntrinsicsSize(*args, **kwargs):
        ...
class PinholeCameraModel(CameraIntrinsicsModel):
    @staticmethod
    def AspectRatio(*args, **kwargs):
        ...
    @staticmethod
    def CameraIntrinsicsPriorFromIntrinsics(*args, **kwargs):
        ...
    @staticmethod
    def GetCalibrationMatrix(*args, **kwargs):
        ...
    @staticmethod
    def GetSubsetFromOptimizeIntrinsicsType(*args, **kwargs):
        ...
    @staticmethod
    def NumParameters(*args, **kwargs):
        ...
    @staticmethod
    def PrintIntrinsics(*args, **kwargs):
        ...
    @staticmethod
    def RadialDistortion1(*args, **kwargs):
        ...
    @staticmethod
    def RadialDistortion2(*args, **kwargs):
        ...
    @staticmethod
    def SetAspectRatio(*args, **kwargs):
        ...
    @staticmethod
    def SetFromCameraIntrinsicsPriors(*args, **kwargs):
        ...
    @staticmethod
    def SetRadialDistortion(*args, **kwargs):
        ...
    @staticmethod
    def SetSkew(*args, **kwargs):
        ...
    @staticmethod
    def Skew(*args, **kwargs):
        ...
    @staticmethod
    def Type(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def kIntrinsicsSize(*args, **kwargs):
        ...
class PinholeRadialTangentialCameraModel(CameraIntrinsicsModel):
    @staticmethod
    def AspectRatio(*args, **kwargs):
        ...
    @staticmethod
    def CameraIntrinsicsPriorFromIntrinsics(*args, **kwargs):
        ...
    @staticmethod
    def GetCalibrationMatrix(*args, **kwargs):
        ...
    @staticmethod
    def GetSubsetFromOptimizeIntrinsicsType(*args, **kwargs):
        ...
    @staticmethod
    def NumParameters(*args, **kwargs):
        ...
    @staticmethod
    def PrintIntrinsics(*args, **kwargs):
        ...
    @staticmethod
    def RadialDistortion1(*args, **kwargs):
        ...
    @staticmethod
    def RadialDistortion2(*args, **kwargs):
        ...
    @staticmethod
    def RadialDistortion3(*args, **kwargs):
        ...
    @staticmethod
    def SetAspectRatio(*args, **kwargs):
        ...
    @staticmethod
    def SetFromCameraIntrinsicsPriors(*args, **kwargs):
        ...
    @staticmethod
    def SetRadialDistortion(*args, **kwargs):
        ...
    @staticmethod
    def SetSkew(*args, **kwargs):
        ...
    @staticmethod
    def SetTangentialDistortion(*args, **kwargs):
        ...
    @staticmethod
    def Skew(*args, **kwargs):
        ...
    @staticmethod
    def TangentialDistortion1(*args, **kwargs):
        ...
    @staticmethod
    def TangentialDistortion2(*args, **kwargs):
        ...
    @staticmethod
    def Type(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def kIntrinsicsSize(*args, **kwargs):
        ...
class Plane:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def point(*args, **kwargs):
        ...
    @point.setter
    def point(*args, **kwargs):
        ...
    @property
    def unit_normal(*args, **kwargs):
        ...
    @unit_normal.setter
    def unit_normal(*args, **kwargs):
        ...
class PnPType:
    """
    Members:
    
      KNEIP
    
      DLS
    
      SQPnP
    """
    DLS: typing.ClassVar[PnPType]  # value = <PnPType.DLS: 2>
    KNEIP: typing.ClassVar[PnPType]  # value = <PnPType.KNEIP: 0>
    SQPnP: typing.ClassVar[PnPType]  # value = <PnPType.SQPnP: 1>
    __members__: typing.ClassVar[dict[str, PnPType]]  # value = {'KNEIP': <PnPType.KNEIP: 0>, 'DLS': <PnPType.DLS: 2>, 'SQPnP': <PnPType.SQPnP: 1>}
    @staticmethod
    def __eq__(*args, **kwargs):
        ...
    @staticmethod
    def __getstate__(*args, **kwargs):
        ...
    @staticmethod
    def __hash__(*args, **kwargs):
        ...
    @staticmethod
    def __index__(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @staticmethod
    def __int__(*args, **kwargs):
        ...
    @staticmethod
    def __ne__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        ...
    @staticmethod
    def __setstate__(*args, **kwargs):
        ...
    @staticmethod
    def __str__(*args, **kwargs):
        ...
    @property
    def name(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
class PositionEstimator:
    pass
class PreconditionerType:
    """
    Members:
    
      IDENTITY
    
      JACOBI
    
      SCHUR_JACOBI
    
      CLUSTER_JACOBI
    
      CLUSTER_TRIDIAGONAL
    """
    CLUSTER_JACOBI: typing.ClassVar[PreconditionerType]  # value = <PreconditionerType.CLUSTER_JACOBI: 4>
    CLUSTER_TRIDIAGONAL: typing.ClassVar[PreconditionerType]  # value = <PreconditionerType.CLUSTER_TRIDIAGONAL: 5>
    IDENTITY: typing.ClassVar[PreconditionerType]  # value = <PreconditionerType.IDENTITY: 0>
    JACOBI: typing.ClassVar[PreconditionerType]  # value = <PreconditionerType.JACOBI: 1>
    SCHUR_JACOBI: typing.ClassVar[PreconditionerType]  # value = <PreconditionerType.SCHUR_JACOBI: 2>
    __members__: typing.ClassVar[dict[str, PreconditionerType]]  # value = {'IDENTITY': <PreconditionerType.IDENTITY: 0>, 'JACOBI': <PreconditionerType.JACOBI: 1>, 'SCHUR_JACOBI': <PreconditionerType.SCHUR_JACOBI: 2>, 'CLUSTER_JACOBI': <PreconditionerType.CLUSTER_JACOBI: 4>, 'CLUSTER_TRIDIAGONAL': <PreconditionerType.CLUSTER_TRIDIAGONAL: 5>}
    @staticmethod
    def __eq__(*args, **kwargs):
        ...
    @staticmethod
    def __getstate__(*args, **kwargs):
        ...
    @staticmethod
    def __hash__(*args, **kwargs):
        ...
    @staticmethod
    def __index__(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @staticmethod
    def __int__(*args, **kwargs):
        ...
    @staticmethod
    def __ne__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        ...
    @staticmethod
    def __setstate__(*args, **kwargs):
        ...
    @staticmethod
    def __str__(*args, **kwargs):
        ...
    @property
    def name(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
class PriorScalar:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def is_set(*args, **kwargs):
        ...
    @is_set.setter
    def is_set(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
    @value.setter
    def value(*args, **kwargs):
        ...
class PriorVector2d:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def is_set(*args, **kwargs):
        ...
    @is_set.setter
    def is_set(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
    @value.setter
    def value(*args, **kwargs):
        ...
class PriorVector3d:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def is_set(*args, **kwargs):
        ...
    @is_set.setter
    def is_set(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
    @value.setter
    def value(*args, **kwargs):
        ...
class PriorVector4d:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def is_set(*args, **kwargs):
        ...
    @is_set.setter
    def is_set(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
    @value.setter
    def value(*args, **kwargs):
        ...
class RadialDistUncalibratedAbsolutePoseMetaData:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def max_focal_length(*args, **kwargs):
        ...
    @max_focal_length.setter
    def max_focal_length(*args, **kwargs):
        ...
    @property
    def max_radial_distortion(*args, **kwargs):
        ...
    @max_radial_distortion.setter
    def max_radial_distortion(*args, **kwargs):
        ...
    @property
    def min_focal_length(*args, **kwargs):
        ...
    @min_focal_length.setter
    def min_focal_length(*args, **kwargs):
        ...
    @property
    def min_radial_distortion(*args, **kwargs):
        ...
    @min_radial_distortion.setter
    def min_radial_distortion(*args, **kwargs):
        ...
class RadialDistortionFeatureCorrespondence:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def feature_left(*args, **kwargs):
        ...
    @feature_left.setter
    def feature_left(*args, **kwargs):
        ...
    @property
    def feature_right(*args, **kwargs):
        ...
    @feature_right.setter
    def feature_right(*args, **kwargs):
        ...
    @property
    def focal_length_estimate_left(*args, **kwargs):
        ...
    @focal_length_estimate_left.setter
    def focal_length_estimate_left(*args, **kwargs):
        ...
    @property
    def focal_length_estimate_right(*args, **kwargs):
        ...
    @focal_length_estimate_right.setter
    def focal_length_estimate_right(*args, **kwargs):
        ...
    @property
    def max_radial_distortion(*args, **kwargs):
        ...
    @max_radial_distortion.setter
    def max_radial_distortion(*args, **kwargs):
        ...
    @property
    def min_radial_distortion(*args, **kwargs):
        ...
    @min_radial_distortion.setter
    def min_radial_distortion(*args, **kwargs):
        ...
    @property
    def normalized_feature_left(*args, **kwargs):
        ...
    @normalized_feature_left.setter
    def normalized_feature_left(*args, **kwargs):
        ...
    @property
    def normalized_feature_right(*args, **kwargs):
        ...
    @normalized_feature_right.setter
    def normalized_feature_right(*args, **kwargs):
        ...
class RansacType:
    """
    Members:
    
      RANSAC
    
      PROSAC
    
      LMED
    
      EXHAUSTIVE
    """
    EXHAUSTIVE: typing.ClassVar[RansacType]  # value = <RansacType.EXHAUSTIVE: 3>
    LMED: typing.ClassVar[RansacType]  # value = <RansacType.LMED: 2>
    PROSAC: typing.ClassVar[RansacType]  # value = <RansacType.PROSAC: 1>
    RANSAC: typing.ClassVar[RansacType]  # value = <RansacType.RANSAC: 0>
    __members__: typing.ClassVar[dict[str, RansacType]]  # value = {'RANSAC': <RansacType.RANSAC: 0>, 'PROSAC': <RansacType.PROSAC: 1>, 'LMED': <RansacType.LMED: 2>, 'EXHAUSTIVE': <RansacType.EXHAUSTIVE: 3>}
    @staticmethod
    def __eq__(*args, **kwargs):
        ...
    @staticmethod
    def __getstate__(*args, **kwargs):
        ...
    @staticmethod
    def __hash__(*args, **kwargs):
        ...
    @staticmethod
    def __index__(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @staticmethod
    def __int__(*args, **kwargs):
        ...
    @staticmethod
    def __ne__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        ...
    @staticmethod
    def __setstate__(*args, **kwargs):
        ...
    @staticmethod
    def __str__(*args, **kwargs):
        ...
    @property
    def name(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
class Reconstruction:
    @staticmethod
    def AddObservation(*args, **kwargs):
        ...
    @staticmethod
    def AddTrack(*args, **kwargs):
        ...
    @staticmethod
    def AddView(*args, **kwargs):
        ...
    @staticmethod
    def CameraIntrinsicsGroupIdFromViewId(*args, **kwargs):
        ...
    @staticmethod
    def CameraIntrinsicsGroupIds(*args, **kwargs):
        ...
    @staticmethod
    def GetViewsInCameraIntrinsicGroup(*args, **kwargs):
        ...
    @staticmethod
    def InitializeInverseDepth(*args, **kwargs):
        ...
    @staticmethod
    def MutableTrack(*args, **kwargs):
        ...
    @staticmethod
    def MutableView(*args, **kwargs):
        ...
    @staticmethod
    def Normalize(*args, **kwargs):
        ...
    @staticmethod
    def NumCameraIntrinsicGroups(*args, **kwargs):
        ...
    @staticmethod
    def NumTracks(*args, **kwargs):
        ...
    @staticmethod
    def NumViews(*args, **kwargs):
        ...
    @staticmethod
    def RemoveTrack(*args, **kwargs):
        ...
    @staticmethod
    def RemoveView(*args, **kwargs):
        ...
    @staticmethod
    def Track(*args, **kwargs):
        ...
    @staticmethod
    def TrackIds(*args, **kwargs):
        ...
    @staticmethod
    def View(*args, **kwargs):
        ...
    @staticmethod
    def ViewIdFromName(*args, **kwargs):
        ...
    @staticmethod
    def ViewIds(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class ReconstructionBuilder:
    @staticmethod
    def AddImage(*args, **kwargs):
        ...
    @staticmethod
    def AddImageWithCameraIntrinsicsPrior(*args, **kwargs):
        ...
    @staticmethod
    def AddMaskForFeaturesExtraction(*args, **kwargs):
        ...
    @staticmethod
    def ExtractAndMatchFeatures(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class ReconstructionBuilderOptions:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def features_and_matches_database_directory(*args, **kwargs):
        ...
    @features_and_matches_database_directory.setter
    def features_and_matches_database_directory(*args, **kwargs):
        ...
    @property
    def matching_options(*args, **kwargs):
        ...
    @matching_options.setter
    def matching_options(*args, **kwargs):
        ...
    @property
    def matching_strategy(*args, **kwargs):
        ...
    @matching_strategy.setter
    def matching_strategy(*args, **kwargs):
        ...
    @property
    def max_track_length(*args, **kwargs):
        ...
    @max_track_length.setter
    def max_track_length(*args, **kwargs):
        ...
    @property
    def min_num_inlier_matches(*args, **kwargs):
        ...
    @min_num_inlier_matches.setter
    def min_num_inlier_matches(*args, **kwargs):
        ...
    @property
    def min_track_length(*args, **kwargs):
        ...
    @min_track_length.setter
    def min_track_length(*args, **kwargs):
        ...
    @property
    def only_calibrated_views(*args, **kwargs):
        ...
    @only_calibrated_views.setter
    def only_calibrated_views(*args, **kwargs):
        ...
    @property
    def reconstruct_largest_connected_component(*args, **kwargs):
        ...
    @reconstruct_largest_connected_component.setter
    def reconstruct_largest_connected_component(*args, **kwargs):
        ...
    @property
    def reconstruction_estimator_options(*args, **kwargs):
        ...
    @reconstruction_estimator_options.setter
    def reconstruction_estimator_options(*args, **kwargs):
        ...
class ReconstructionEstimator:
    @staticmethod
    def Create(*args, **kwargs):
        ...
class ReconstructionEstimatorOptions:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def absolute_pose_reprojection_error_threshold(*args, **kwargs):
        ...
    @absolute_pose_reprojection_error_threshold.setter
    def absolute_pose_reprojection_error_threshold(*args, **kwargs):
        ...
    @property
    def bundle_adjust_tracks(*args, **kwargs):
        ...
    @bundle_adjust_tracks.setter
    def bundle_adjust_tracks(*args, **kwargs):
        ...
    @property
    def bundle_adjustment_loss_function_type(*args, **kwargs):
        ...
    @bundle_adjustment_loss_function_type.setter
    def bundle_adjustment_loss_function_type(*args, **kwargs):
        ...
    @property
    def bundle_adjustment_robust_loss_width(*args, **kwargs):
        ...
    @bundle_adjustment_robust_loss_width.setter
    def bundle_adjustment_robust_loss_width(*args, **kwargs):
        ...
    @property
    def dense_linear_algebra_library_type(*args, **kwargs):
        ...
    @dense_linear_algebra_library_type.setter
    def dense_linear_algebra_library_type(*args, **kwargs):
        ...
    @property
    def extract_maximal_rigid_subgraph(*args, **kwargs):
        ...
    @extract_maximal_rigid_subgraph.setter
    def extract_maximal_rigid_subgraph(*args, **kwargs):
        ...
    @property
    def filter_relative_translations_with_1dsfm(*args, **kwargs):
        ...
    @filter_relative_translations_with_1dsfm.setter
    def filter_relative_translations_with_1dsfm(*args, **kwargs):
        ...
    @property
    def full_bundle_adjustment_growth_percent(*args, **kwargs):
        ...
    @full_bundle_adjustment_growth_percent.setter
    def full_bundle_adjustment_growth_percent(*args, **kwargs):
        ...
    @property
    def global_position_estimator_type(*args, **kwargs):
        ...
    @global_position_estimator_type.setter
    def global_position_estimator_type(*args, **kwargs):
        ...
    @property
    def global_rotation_estimator_type(*args, **kwargs):
        ...
    @global_rotation_estimator_type.setter
    def global_rotation_estimator_type(*args, **kwargs):
        ...
    @property
    def intrinsics_to_optimize(*args, **kwargs):
        ...
    @intrinsics_to_optimize.setter
    def intrinsics_to_optimize(*args, **kwargs):
        ...
    @property
    def least_unsquared_deviation_position_estimator_options(*args, **kwargs):
        ...
    @least_unsquared_deviation_position_estimator_options.setter
    def least_unsquared_deviation_position_estimator_options(*args, **kwargs):
        ...
    @property
    def linear_solver_type(*args, **kwargs):
        ...
    @linear_solver_type.setter
    def linear_solver_type(*args, **kwargs):
        ...
    @property
    def linear_triplet_position_estimator_options(*args, **kwargs):
        ...
    @linear_triplet_position_estimator_options.setter
    def linear_triplet_position_estimator_options(*args, **kwargs):
        ...
    @property
    def localization_pnp_type(*args, **kwargs):
        ...
    @localization_pnp_type.setter
    def localization_pnp_type(*args, **kwargs):
        ...
    @property
    def max_num_iterations(*args, **kwargs):
        ...
    @max_num_iterations.setter
    def max_num_iterations(*args, **kwargs):
        ...
    @property
    def max_reprojection_error_in_pixels(*args, **kwargs):
        ...
    @max_reprojection_error_in_pixels.setter
    def max_reprojection_error_in_pixels(*args, **kwargs):
        ...
    @property
    def min_cameras_for_iterative_solver(*args, **kwargs):
        ...
    @min_cameras_for_iterative_solver.setter
    def min_cameras_for_iterative_solver(*args, **kwargs):
        ...
    @property
    def min_num_absolute_pose_inliers(*args, **kwargs):
        ...
    @min_num_absolute_pose_inliers.setter
    def min_num_absolute_pose_inliers(*args, **kwargs):
        ...
    @property
    def min_num_optimized_tracks_per_view(*args, **kwargs):
        ...
    @min_num_optimized_tracks_per_view.setter
    def min_num_optimized_tracks_per_view(*args, **kwargs):
        ...
    @property
    def min_num_two_view_inliers(*args, **kwargs):
        ...
    @min_num_two_view_inliers.setter
    def min_num_two_view_inliers(*args, **kwargs):
        ...
    @property
    def min_triangulation_angle_degrees(*args, **kwargs):
        ...
    @min_triangulation_angle_degrees.setter
    def min_triangulation_angle_degrees(*args, **kwargs):
        ...
    @property
    def multiple_view_localization_ratio(*args, **kwargs):
        ...
    @multiple_view_localization_ratio.setter
    def multiple_view_localization_ratio(*args, **kwargs):
        ...
    @property
    def nonlinear_position_estimator_options(*args, **kwargs):
        ...
    @nonlinear_position_estimator_options.setter
    def nonlinear_position_estimator_options(*args, **kwargs):
        ...
    @property
    def num_retriangulation_iterations(*args, **kwargs):
        ...
    @num_retriangulation_iterations.setter
    def num_retriangulation_iterations(*args, **kwargs):
        ...
    @property
    def num_threads(*args, **kwargs):
        ...
    @num_threads.setter
    def num_threads(*args, **kwargs):
        ...
    @property
    def optimize_for_forward_facing_trajectory(*args, **kwargs):
        ...
    @optimize_for_forward_facing_trajectory.setter
    def optimize_for_forward_facing_trajectory(*args, **kwargs):
        ...
    @property
    def partial_bundle_adjustment_num_views(*args, **kwargs):
        ...
    @partial_bundle_adjustment_num_views.setter
    def partial_bundle_adjustment_num_views(*args, **kwargs):
        ...
    @property
    def preconditioner_type(*args, **kwargs):
        ...
    @preconditioner_type.setter
    def preconditioner_type(*args, **kwargs):
        ...
    @property
    def ransac_confidence(*args, **kwargs):
        ...
    @ransac_confidence.setter
    def ransac_confidence(*args, **kwargs):
        ...
    @property
    def ransac_lo_start_iterations(*args, **kwargs):
        ...
    @ransac_lo_start_iterations.setter
    def ransac_lo_start_iterations(*args, **kwargs):
        ...
    @property
    def ransac_max_iterations(*args, **kwargs):
        ...
    @ransac_max_iterations.setter
    def ransac_max_iterations(*args, **kwargs):
        ...
    @property
    def ransac_min_iterations(*args, **kwargs):
        ...
    @ransac_min_iterations.setter
    def ransac_min_iterations(*args, **kwargs):
        ...
    @property
    def ransac_use_lo(*args, **kwargs):
        ...
    @ransac_use_lo.setter
    def ransac_use_lo(*args, **kwargs):
        ...
    @property
    def ransac_use_mle(*args, **kwargs):
        ...
    @ransac_use_mle.setter
    def ransac_use_mle(*args, **kwargs):
        ...
    @property
    def reconstruction_estimator_type(*args, **kwargs):
        ...
    @reconstruction_estimator_type.setter
    def reconstruction_estimator_type(*args, **kwargs):
        ...
    @property
    def refine_camera_positions_and_points_after_position_estimation(*args, **kwargs):
        ...
    @refine_camera_positions_and_points_after_position_estimation.setter
    def refine_camera_positions_and_points_after_position_estimation(*args, **kwargs):
        ...
    @property
    def refine_relative_translations_after_rotation_estimation(*args, **kwargs):
        ...
    @refine_relative_translations_after_rotation_estimation.setter
    def refine_relative_translations_after_rotation_estimation(*args, **kwargs):
        ...
    @property
    def relative_position_estimation_max_sampson_error_pixels(*args, **kwargs):
        ...
    @relative_position_estimation_max_sampson_error_pixels.setter
    def relative_position_estimation_max_sampson_error_pixels(*args, **kwargs):
        ...
    @property
    def rotation_estimation_robust_loss_scale(*args, **kwargs):
        ...
    @rotation_estimation_robust_loss_scale.setter
    def rotation_estimation_robust_loss_scale(*args, **kwargs):
        ...
    @property
    def rotation_filtering_max_difference_degrees(*args, **kwargs):
        ...
    @rotation_filtering_max_difference_degrees.setter
    def rotation_filtering_max_difference_degrees(*args, **kwargs):
        ...
    @property
    def sparse_linear_algebra_library_type(*args, **kwargs):
        ...
    @sparse_linear_algebra_library_type.setter
    def sparse_linear_algebra_library_type(*args, **kwargs):
        ...
    @property
    def subsample_tracks_for_bundle_adjustment(*args, **kwargs):
        ...
    @subsample_tracks_for_bundle_adjustment.setter
    def subsample_tracks_for_bundle_adjustment(*args, **kwargs):
        ...
    @property
    def track_parametrization_type(*args, **kwargs):
        ...
    @track_parametrization_type.setter
    def track_parametrization_type(*args, **kwargs):
        ...
    @property
    def track_selection_image_grid_cell_size_pixels(*args, **kwargs):
        ...
    @track_selection_image_grid_cell_size_pixels.setter
    def track_selection_image_grid_cell_size_pixels(*args, **kwargs):
        ...
    @property
    def track_subset_selection_long_track_length_threshold(*args, **kwargs):
        ...
    @track_subset_selection_long_track_length_threshold.setter
    def track_subset_selection_long_track_length_threshold(*args, **kwargs):
        ...
    @property
    def translation_filtering_num_iterations(*args, **kwargs):
        ...
    @translation_filtering_num_iterations.setter
    def translation_filtering_num_iterations(*args, **kwargs):
        ...
    @property
    def translation_filtering_projection_tolerance(*args, **kwargs):
        ...
    @translation_filtering_projection_tolerance.setter
    def translation_filtering_projection_tolerance(*args, **kwargs):
        ...
    @property
    def triangulation_method(*args, **kwargs):
        ...
    @triangulation_method.setter
    def triangulation_method(*args, **kwargs):
        ...
    @property
    def use_inner_iterations(*args, **kwargs):
        ...
    @use_inner_iterations.setter
    def use_inner_iterations(*args, **kwargs):
        ...
    @property
    def visibility_clustering_type(*args, **kwargs):
        ...
    @visibility_clustering_type.setter
    def visibility_clustering_type(*args, **kwargs):
        ...
class ReconstructionEstimatorSummary:
    @property
    def bundle_adjustment_time(*args, **kwargs):
        ...
    @bundle_adjustment_time.setter
    def bundle_adjustment_time(*args, **kwargs):
        ...
    @property
    def camera_intrinsics_calibration_time(*args, **kwargs):
        ...
    @camera_intrinsics_calibration_time.setter
    def camera_intrinsics_calibration_time(*args, **kwargs):
        ...
    @property
    def estimated_tracks(*args, **kwargs):
        ...
    @estimated_tracks.setter
    def estimated_tracks(*args, **kwargs):
        ...
    @property
    def estimated_views(*args, **kwargs):
        ...
    @estimated_views.setter
    def estimated_views(*args, **kwargs):
        ...
    @property
    def message(*args, **kwargs):
        ...
    @message.setter
    def message(*args, **kwargs):
        ...
    @property
    def pose_estimation_time(*args, **kwargs):
        ...
    @pose_estimation_time.setter
    def pose_estimation_time(*args, **kwargs):
        ...
    @property
    def success(*args, **kwargs):
        ...
    @success.setter
    def success(*args, **kwargs):
        ...
    @property
    def total_time(*args, **kwargs):
        ...
    @total_time.setter
    def total_time(*args, **kwargs):
        ...
    @property
    def triangulation_time(*args, **kwargs):
        ...
    @triangulation_time.setter
    def triangulation_time(*args, **kwargs):
        ...
class ReconstructionEstimatorType:
    """
    Members:
    
      GLOBAL
    
      INCREMENTAL
    
      HYBRID
    """
    GLOBAL: typing.ClassVar[ReconstructionEstimatorType]  # value = <ReconstructionEstimatorType.GLOBAL: 0>
    HYBRID: typing.ClassVar[ReconstructionEstimatorType]  # value = <ReconstructionEstimatorType.HYBRID: 2>
    INCREMENTAL: typing.ClassVar[ReconstructionEstimatorType]  # value = <ReconstructionEstimatorType.INCREMENTAL: 1>
    __members__: typing.ClassVar[dict[str, ReconstructionEstimatorType]]  # value = {'GLOBAL': <ReconstructionEstimatorType.GLOBAL: 0>, 'INCREMENTAL': <ReconstructionEstimatorType.INCREMENTAL: 1>, 'HYBRID': <ReconstructionEstimatorType.HYBRID: 2>}
    @staticmethod
    def __eq__(*args, **kwargs):
        ...
    @staticmethod
    def __getstate__(*args, **kwargs):
        ...
    @staticmethod
    def __hash__(*args, **kwargs):
        ...
    @staticmethod
    def __index__(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @staticmethod
    def __int__(*args, **kwargs):
        ...
    @staticmethod
    def __ne__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        ...
    @staticmethod
    def __setstate__(*args, **kwargs):
        ...
    @staticmethod
    def __str__(*args, **kwargs):
        ...
    @property
    def name(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
class RelativePose:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def essential_matrix(*args, **kwargs):
        ...
    @essential_matrix.setter
    def essential_matrix(*args, **kwargs):
        ...
    @property
    def position(*args, **kwargs):
        ...
    @position.setter
    def position(*args, **kwargs):
        ...
    @property
    def rotation(*args, **kwargs):
        ...
    @rotation.setter
    def rotation(*args, **kwargs):
        ...
class RigidTransformation:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def rotation(*args, **kwargs):
        ...
    @rotation.setter
    def rotation(*args, **kwargs):
        ...
    @property
    def translation(*args, **kwargs):
        ...
    @translation.setter
    def translation(*args, **kwargs):
        ...
class RobustRotationEstimator(RotationEstimator):
    @staticmethod
    def AddRelativeRotationConstraint(*args, **kwargs):
        ...
    @staticmethod
    def EstimateRotations(*args, **kwargs):
        ...
    @staticmethod
    def SetFixedGlobalRotations(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class RobustRotationEstimatorOptions:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def irls_loss_parameter_sigma(*args, **kwargs):
        ...
    @irls_loss_parameter_sigma.setter
    def irls_loss_parameter_sigma(*args, **kwargs):
        ...
    @property
    def irls_step_convergence_threshold(*args, **kwargs):
        ...
    @irls_step_convergence_threshold.setter
    def irls_step_convergence_threshold(*args, **kwargs):
        ...
    @property
    def l1_step_convergence_threshold(*args, **kwargs):
        ...
    @l1_step_convergence_threshold.setter
    def l1_step_convergence_threshold(*args, **kwargs):
        ...
    @property
    def max_num_irls_iterations(*args, **kwargs):
        ...
    @max_num_irls_iterations.setter
    def max_num_irls_iterations(*args, **kwargs):
        ...
    @property
    def max_num_l1_iterations(*args, **kwargs):
        ...
    @max_num_l1_iterations.setter
    def max_num_l1_iterations(*args, **kwargs):
        ...
class RotationEstimator:
    pass
class Sim3AlignmentOptions:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @staticmethod
    def clear_initial_sim3_params(*args, **kwargs):
        """
        Clear initial SIM3 parameters
        """
    @staticmethod
    def clear_point_weights(*args, **kwargs):
        """
        Clear point weights
        """
    @staticmethod
    def clear_target_normals(*args, **kwargs):
        """
        Clear target normals
        """
    @staticmethod
    def set_initial_sim3_params(*args, **kwargs):
        """
        Set initial SIM3 parameters
        """
    @staticmethod
    def set_point_weights(*args, **kwargs):
        """
        Set point weights
        """
    @staticmethod
    def set_target_normals(*args, **kwargs):
        """
        Set target normals for point-to-plane alignment
        """
    @property
    def alignment_type(*args, **kwargs):
        ...
    @alignment_type.setter
    def alignment_type(*args, **kwargs):
        ...
    @property
    def huber_threshold(*args, **kwargs):
        ...
    @huber_threshold.setter
    def huber_threshold(*args, **kwargs):
        ...
    @property
    def max_iterations(*args, **kwargs):
        ...
    @max_iterations.setter
    def max_iterations(*args, **kwargs):
        ...
    @property
    def outlier_threshold(*args, **kwargs):
        ...
    @outlier_threshold.setter
    def outlier_threshold(*args, **kwargs):
        ...
    @property
    def perform_optimization(*args, **kwargs):
        ...
    @perform_optimization.setter
    def perform_optimization(*args, **kwargs):
        ...
    @property
    def point_weight(*args, **kwargs):
        ...
    @point_weight.setter
    def point_weight(*args, **kwargs):
        ...
    @property
    def verbose(*args, **kwargs):
        ...
    @verbose.setter
    def verbose(*args, **kwargs):
        ...
class Sim3AlignmentSummary:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def alignment_error(*args, **kwargs):
        ...
    @alignment_error.setter
    def alignment_error(*args, **kwargs):
        ...
    @property
    def final_cost(*args, **kwargs):
        ...
    @final_cost.setter
    def final_cost(*args, **kwargs):
        ...
    @property
    def num_iterations(*args, **kwargs):
        ...
    @num_iterations.setter
    def num_iterations(*args, **kwargs):
        ...
    @property
    def sim3_params(*args, **kwargs):
        ...
    @sim3_params.setter
    def sim3_params(*args, **kwargs):
        ...
    @property
    def success(*args, **kwargs):
        ...
    @success.setter
    def success(*args, **kwargs):
        ...
class Sim3AlignmentType:
    """
    Members:
    
      POINT_TO_POINT
    
      ROBUST_POINT_TO_POINT
    
      POINT_TO_PLANE
    """
    POINT_TO_PLANE: typing.ClassVar[Sim3AlignmentType]  # value = <Sim3AlignmentType.POINT_TO_PLANE: 2>
    POINT_TO_POINT: typing.ClassVar[Sim3AlignmentType]  # value = <Sim3AlignmentType.POINT_TO_POINT: 0>
    ROBUST_POINT_TO_POINT: typing.ClassVar[Sim3AlignmentType]  # value = <Sim3AlignmentType.ROBUST_POINT_TO_POINT: 1>
    __members__: typing.ClassVar[dict[str, Sim3AlignmentType]]  # value = {'POINT_TO_POINT': <Sim3AlignmentType.POINT_TO_POINT: 0>, 'ROBUST_POINT_TO_POINT': <Sim3AlignmentType.ROBUST_POINT_TO_POINT: 1>, 'POINT_TO_PLANE': <Sim3AlignmentType.POINT_TO_PLANE: 2>}
    @staticmethod
    def __eq__(*args, **kwargs):
        ...
    @staticmethod
    def __getstate__(*args, **kwargs):
        ...
    @staticmethod
    def __hash__(*args, **kwargs):
        ...
    @staticmethod
    def __index__(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @staticmethod
    def __int__(*args, **kwargs):
        ...
    @staticmethod
    def __ne__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        ...
    @staticmethod
    def __setstate__(*args, **kwargs):
        ...
    @staticmethod
    def __str__(*args, **kwargs):
        ...
    @property
    def name(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
class SimilarityTransformation:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def rotation(*args, **kwargs):
        ...
    @rotation.setter
    def rotation(*args, **kwargs):
        ...
    @property
    def scale(*args, **kwargs):
        ...
    @scale.setter
    def scale(*args, **kwargs):
        ...
    @property
    def translation(*args, **kwargs):
        ...
    @translation.setter
    def translation(*args, **kwargs):
        ...
class SparseLinearAlgebraLibraryType:
    """
    Members:
    
      SUITE_SPARSE
    
      EIGEN_SPARSE
    
      ACCELERATE_SPARSE
    
      CUDA_SPARSE
    """
    ACCELERATE_SPARSE: typing.ClassVar[SparseLinearAlgebraLibraryType]  # value = <SparseLinearAlgebraLibraryType.ACCELERATE_SPARSE: 2>
    CUDA_SPARSE: typing.ClassVar[SparseLinearAlgebraLibraryType]  # value = <SparseLinearAlgebraLibraryType.CUDA_SPARSE: 3>
    EIGEN_SPARSE: typing.ClassVar[SparseLinearAlgebraLibraryType]  # value = <SparseLinearAlgebraLibraryType.EIGEN_SPARSE: 1>
    SUITE_SPARSE: typing.ClassVar[SparseLinearAlgebraLibraryType]  # value = <SparseLinearAlgebraLibraryType.SUITE_SPARSE: 0>
    __members__: typing.ClassVar[dict[str, SparseLinearAlgebraLibraryType]]  # value = {'SUITE_SPARSE': <SparseLinearAlgebraLibraryType.SUITE_SPARSE: 0>, 'EIGEN_SPARSE': <SparseLinearAlgebraLibraryType.EIGEN_SPARSE: 1>, 'ACCELERATE_SPARSE': <SparseLinearAlgebraLibraryType.ACCELERATE_SPARSE: 2>, 'CUDA_SPARSE': <SparseLinearAlgebraLibraryType.CUDA_SPARSE: 3>}
    @staticmethod
    def __eq__(*args, **kwargs):
        ...
    @staticmethod
    def __getstate__(*args, **kwargs):
        ...
    @staticmethod
    def __hash__(*args, **kwargs):
        ...
    @staticmethod
    def __index__(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @staticmethod
    def __int__(*args, **kwargs):
        ...
    @staticmethod
    def __ne__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        ...
    @staticmethod
    def __setstate__(*args, **kwargs):
        ...
    @staticmethod
    def __str__(*args, **kwargs):
        ...
    @property
    def name(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
class Track:
    @staticmethod
    def AddView(*args, **kwargs):
        ...
    @staticmethod
    def Color(*args, **kwargs):
        ...
    @staticmethod
    def InverseDepth(*args, **kwargs):
        ...
    @staticmethod
    def IsEstimated(*args, **kwargs):
        ...
    @staticmethod
    def NumViews(*args, **kwargs):
        ...
    @staticmethod
    def Point(*args, **kwargs):
        ...
    @staticmethod
    def ReferenceBearingVector(*args, **kwargs):
        ...
    @staticmethod
    def ReferenceDescriptor(*args, **kwargs):
        ...
    @staticmethod
    def ReferenceViewId(*args, **kwargs):
        ...
    @staticmethod
    def RemoveView(*args, **kwargs):
        ...
    @staticmethod
    def SetColor(*args, **kwargs):
        ...
    @staticmethod
    def SetInverseDepth(*args, **kwargs):
        ...
    @staticmethod
    def SetIsEstimated(*args, **kwargs):
        ...
    @staticmethod
    def SetPoint(*args, **kwargs):
        ...
    @staticmethod
    def SetReferenceBearingVector(*args, **kwargs):
        ...
    @staticmethod
    def SetReferenceDescriptor(*args, **kwargs):
        ...
    @staticmethod
    def ViewIds(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class TrackBuilder:
    @staticmethod
    def AddFeatureCorrespondence(*args, **kwargs):
        ...
    @staticmethod
    def BuildTracks(*args, **kwargs):
        ...
    @staticmethod
    def BuildTracksIncremental(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class TrackEstimator:
    @staticmethod
    def EstimateAllTracks(*args, **kwargs):
        ...
    @staticmethod
    def EstimateTracks(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class TrackEstimatorOptions:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def bundle_adjustment(*args, **kwargs):
        ...
    @bundle_adjustment.setter
    def bundle_adjustment(*args, **kwargs):
        ...
    @property
    def max_acceptable_reprojection_error_pixels(*args, **kwargs):
        ...
    @max_acceptable_reprojection_error_pixels.setter
    def max_acceptable_reprojection_error_pixels(*args, **kwargs):
        ...
    @property
    def min_triangulation_angle_degrees(*args, **kwargs):
        ...
    @min_triangulation_angle_degrees.setter
    def min_triangulation_angle_degrees(*args, **kwargs):
        ...
    @property
    def multithreaded_step_size(*args, **kwargs):
        ...
    @multithreaded_step_size.setter
    def multithreaded_step_size(*args, **kwargs):
        ...
    @property
    def num_threads(*args, **kwargs):
        ...
    @num_threads.setter
    def num_threads(*args, **kwargs):
        ...
    @property
    def triangulation_method(*args, **kwargs):
        ...
    @triangulation_method.setter
    def triangulation_method(*args, **kwargs):
        ...
class TrackEstimatorSummary:
    @property
    def estimated_tracks(*args, **kwargs):
        ...
    @estimated_tracks.setter
    def estimated_tracks(*args, **kwargs):
        ...
    @property
    def input_num_estimated_tracks(*args, **kwargs):
        ...
    @input_num_estimated_tracks.setter
    def input_num_estimated_tracks(*args, **kwargs):
        ...
    @property
    def num_triangulation_attempts(*args, **kwargs):
        ...
    @num_triangulation_attempts.setter
    def num_triangulation_attempts(*args, **kwargs):
        ...
class TrackParametrizationType:
    """
    Members:
    
      XYZW
    
      XYZW_MANIFOLD
    
      INVERSE_DEPTH
    """
    INVERSE_DEPTH: typing.ClassVar[TrackParametrizationType]  # value = <TrackParametrizationType.INVERSE_DEPTH: 2>
    XYZW: typing.ClassVar[TrackParametrizationType]  # value = <TrackParametrizationType.XYZW: 0>
    XYZW_MANIFOLD: typing.ClassVar[TrackParametrizationType]  # value = <TrackParametrizationType.XYZW_MANIFOLD: 1>
    __members__: typing.ClassVar[dict[str, TrackParametrizationType]]  # value = {'XYZW': <TrackParametrizationType.XYZW: 0>, 'XYZW_MANIFOLD': <TrackParametrizationType.XYZW_MANIFOLD: 1>, 'INVERSE_DEPTH': <TrackParametrizationType.INVERSE_DEPTH: 2>}
    @staticmethod
    def __eq__(*args, **kwargs):
        ...
    @staticmethod
    def __getstate__(*args, **kwargs):
        ...
    @staticmethod
    def __hash__(*args, **kwargs):
        ...
    @staticmethod
    def __index__(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @staticmethod
    def __int__(*args, **kwargs):
        ...
    @staticmethod
    def __ne__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        ...
    @staticmethod
    def __setstate__(*args, **kwargs):
        ...
    @staticmethod
    def __str__(*args, **kwargs):
        ...
    @property
    def name(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
class TriangulationMethodType:
    """
    Members:
    
      MIDPOINT
    
      SVD
    
      L2_MINIMIZATION
    """
    L2_MINIMIZATION: typing.ClassVar[TriangulationMethodType]  # value = <TriangulationMethodType.L2_MINIMIZATION: 2>
    MIDPOINT: typing.ClassVar[TriangulationMethodType]  # value = <TriangulationMethodType.MIDPOINT: 0>
    SVD: typing.ClassVar[TriangulationMethodType]  # value = <TriangulationMethodType.SVD: 1>
    __members__: typing.ClassVar[dict[str, TriangulationMethodType]]  # value = {'MIDPOINT': <TriangulationMethodType.MIDPOINT: 0>, 'SVD': <TriangulationMethodType.SVD: 1>, 'L2_MINIMIZATION': <TriangulationMethodType.L2_MINIMIZATION: 2>}
    @staticmethod
    def __eq__(*args, **kwargs):
        ...
    @staticmethod
    def __getstate__(*args, **kwargs):
        ...
    @staticmethod
    def __hash__(*args, **kwargs):
        ...
    @staticmethod
    def __index__(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @staticmethod
    def __int__(*args, **kwargs):
        ...
    @staticmethod
    def __ne__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        ...
    @staticmethod
    def __setstate__(*args, **kwargs):
        ...
    @staticmethod
    def __str__(*args, **kwargs):
        ...
    @property
    def name(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
class TwoViewBundleAdjustmentOptions:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def ba_options(*args, **kwargs):
        ...
    @ba_options.setter
    def ba_options(*args, **kwargs):
        ...
    @property
    def constant_camera1_intrinsics(*args, **kwargs):
        ...
    @constant_camera1_intrinsics.setter
    def constant_camera1_intrinsics(*args, **kwargs):
        ...
    @property
    def constant_camera2_intrinsics(*args, **kwargs):
        ...
    @constant_camera2_intrinsics.setter
    def constant_camera2_intrinsics(*args, **kwargs):
        ...
class TwoViewInfo:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def focal_length_1(*args, **kwargs):
        ...
    @focal_length_1.setter
    def focal_length_1(*args, **kwargs):
        ...
    @property
    def focal_length_2(*args, **kwargs):
        ...
    @focal_length_2.setter
    def focal_length_2(*args, **kwargs):
        ...
    @property
    def num_homography_inliers(*args, **kwargs):
        ...
    @num_homography_inliers.setter
    def num_homography_inliers(*args, **kwargs):
        ...
    @property
    def num_verified_matches(*args, **kwargs):
        ...
    @num_verified_matches.setter
    def num_verified_matches(*args, **kwargs):
        ...
    @property
    def position_2(*args, **kwargs):
        ...
    @position_2.setter
    def position_2(*args, **kwargs):
        ...
    @property
    def rotation_2(*args, **kwargs):
        ...
    @rotation_2.setter
    def rotation_2(*args, **kwargs):
        ...
    @property
    def scale_estimate(*args, **kwargs):
        ...
    @scale_estimate.setter
    def scale_estimate(*args, **kwargs):
        ...
    @property
    def visibility_score(*args, **kwargs):
        ...
    @visibility_score.setter
    def visibility_score(*args, **kwargs):
        ...
class UncalibratedAbsolutePose:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def position(*args, **kwargs):
        ...
    @position.setter
    def position(*args, **kwargs):
        ...
    @property
    def rotation(*args, **kwargs):
        ...
    @rotation.setter
    def rotation(*args, **kwargs):
        ...
class UncalibratedRelativePose:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def focal_length1(*args, **kwargs):
        ...
    @focal_length1.setter
    def focal_length1(*args, **kwargs):
        ...
    @property
    def focal_length2(*args, **kwargs):
        ...
    @focal_length2.setter
    def focal_length2(*args, **kwargs):
        ...
    @property
    def fundamental_matrix(*args, **kwargs):
        ...
    @fundamental_matrix.setter
    def fundamental_matrix(*args, **kwargs):
        ...
    @property
    def position(*args, **kwargs):
        ...
    @position.setter
    def position(*args, **kwargs):
        ...
    @property
    def rotation(*args, **kwargs):
        ...
    @rotation.setter
    def rotation(*args, **kwargs):
        ...
class View:
    @staticmethod
    def AddFeature(*args, **kwargs):
        ...
    @staticmethod
    def Camera(*args, **kwargs):
        """
        Camera class object
        """
    @staticmethod
    def CameraIntrinsicsPrior(*args, **kwargs):
        ...
    @staticmethod
    def GetFeature(*args, **kwargs):
        ...
    @staticmethod
    def GetGravityPrior(*args, **kwargs):
        ...
    @staticmethod
    def GetGravityPriorSqrtInformation(*args, **kwargs):
        ...
    @staticmethod
    def GetOrientationPrior(*args, **kwargs):
        ...
    @staticmethod
    def GetOrientationPriorSqrtInformation(*args, **kwargs):
        ...
    @staticmethod
    def GetPositionPrior(*args, **kwargs):
        ...
    @staticmethod
    def GetPositionPriorSqrtInformation(*args, **kwargs):
        ...
    @staticmethod
    def GetTimestamp(*args, **kwargs):
        ...
    @staticmethod
    def GetTrack(*args, **kwargs):
        ...
    @staticmethod
    def HasGravityPrior(*args, **kwargs):
        ...
    @staticmethod
    def HasOrientationPrior(*args, **kwargs):
        ...
    @staticmethod
    def HasPositionPrior(*args, **kwargs):
        ...
    @staticmethod
    def IsEstimated(*args, **kwargs):
        ...
    @staticmethod
    def MutableCamera(*args, **kwargs):
        ...
    @staticmethod
    def MutableCameraIntrinsicsPrior(*args, **kwargs):
        ...
    @staticmethod
    def Name(*args, **kwargs):
        ...
    @staticmethod
    def NumFeatures(*args, **kwargs):
        ...
    @staticmethod
    def RemoveFeature(*args, **kwargs):
        ...
    @staticmethod
    def SetCameraIntrinsicsPrior(*args, **kwargs):
        ...
    @staticmethod
    def SetGravityPrior(*args, **kwargs):
        ...
    @staticmethod
    def SetIsEstimated(*args, **kwargs):
        ...
    @staticmethod
    def SetOrientationPrior(*args, **kwargs):
        ...
    @staticmethod
    def SetPositionPrior(*args, **kwargs):
        ...
    @staticmethod
    def TrackIds(*args, **kwargs):
        ...
    @staticmethod
    def UpdateFeature(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class ViewGraph:
    @staticmethod
    def AddEdge(*args, **kwargs):
        ...
    @staticmethod
    def GetAllEdges(*args, **kwargs):
        ...
    @staticmethod
    def GetEdge(*args, **kwargs):
        ...
    @staticmethod
    def GetNeighborIdsForView(*args, **kwargs):
        ...
    @staticmethod
    def HasEdge(*args, **kwargs):
        ...
    @staticmethod
    def HasView(*args, **kwargs):
        ...
    @staticmethod
    def NumEdges(*args, **kwargs):
        ...
    @staticmethod
    def NumViews(*args, **kwargs):
        ...
    @staticmethod
    def ReadFromDisk(*args, **kwargs):
        ...
    @staticmethod
    def RemoveEdge(*args, **kwargs):
        ...
    @staticmethod
    def RemoveView(*args, **kwargs):
        ...
    @staticmethod
    def WriteToDisk(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
class VisibilityClusteringType:
    """
    Members:
    
      SINGLE_LINKAGE
    
      CANONICAL_VIEWS
    """
    CANONICAL_VIEWS: typing.ClassVar[VisibilityClusteringType]  # value = <VisibilityClusteringType.CANONICAL_VIEWS: 0>
    SINGLE_LINKAGE: typing.ClassVar[VisibilityClusteringType]  # value = <VisibilityClusteringType.SINGLE_LINKAGE: 1>
    __members__: typing.ClassVar[dict[str, VisibilityClusteringType]]  # value = {'SINGLE_LINKAGE': <VisibilityClusteringType.SINGLE_LINKAGE: 1>, 'CANONICAL_VIEWS': <VisibilityClusteringType.CANONICAL_VIEWS: 0>}
    @staticmethod
    def __eq__(*args, **kwargs):
        ...
    @staticmethod
    def __getstate__(*args, **kwargs):
        ...
    @staticmethod
    def __hash__(*args, **kwargs):
        ...
    @staticmethod
    def __index__(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @staticmethod
    def __int__(*args, **kwargs):
        ...
    @staticmethod
    def __ne__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        ...
    @staticmethod
    def __setstate__(*args, **kwargs):
        ...
    @staticmethod
    def __str__(*args, **kwargs):
        ...
    @property
    def name(*args, **kwargs):
        ...
    @property
    def value(*args, **kwargs):
        ...
class VisibilityPyramid:
    @staticmethod
    def AddPoint(*args, **kwargs):
        ...
    @staticmethod
    def ComputeScore(*args, **kwargs):
        ...
    @staticmethod
    def __init__(*args, **kwargs):
        ...
def AddFeatureCorrespondencesToTrackBuilder(*args, **kwargs):
    ...
def AddFullFeatureCorrespondencesToTrackBuilder(*args, **kwargs):
    ...
def AddObservations(*args, **kwargs):
    ...
def AddTracks(*args, **kwargs):
    ...
def AlignPointCloudsUmeyama(*args, **kwargs):
    ...
def AlignPointCloudsUmeyamaWithWeights(*args, **kwargs):
    ...
def AlignReconstructions(*args, **kwargs):
    ...
def AlignReconstructionsRobust(*args, **kwargs):
    ...
def AlignRotations(*args, **kwargs):
    ...
def BundleAdjustPartialReconstruction(*args, **kwargs):
    ...
def BundleAdjustPartialViewsConstant(*args, **kwargs):
    ...
def BundleAdjustReconstruction(*args, **kwargs):
    ...
def BundleAdjustTrack(*args, **kwargs):
    ...
def BundleAdjustTrackWithCov(*args, **kwargs):
    ...
def BundleAdjustTracks(*args, **kwargs):
    ...
def BundleAdjustTracksWithCov(*args, **kwargs):
    ...
def BundleAdjustTwoViewsAngular(*args, **kwargs):
    ...
def BundleAdjustView(*args, **kwargs):
    ...
def BundleAdjustViewWithCov(*args, **kwargs):
    ...
def BundleAdjustViews(*args, **kwargs):
    ...
def BundleAdjustViewsWithCov(*args, **kwargs):
    ...
def CalibrationMatrixToIntrinsics(*args, **kwargs):
    ...
def ColorizeReconstruction(*args, **kwargs):
    ...
def ComposeFundamentalMatrix(*args, **kwargs):
    ...
def ComposeProjectionMatrix(*args, **kwargs):
    ...
def ComputeTripletBaselineRatios(*args, **kwargs):
    ...
def CreateEstimatedSubreconstruction(*args, **kwargs):
    ...
def DecomposeEssentialMatrix(*args, **kwargs):
    ...
def DecomposeProjectionMatrix(*args, **kwargs):
    ...
def DlsPnp(*args, **kwargs):
    ...
def EssentialMatrixFromFundamentalMatrix(*args, **kwargs):
    ...
def EssentialMatrixFromTwoProjectionMatrices(*args, **kwargs):
    ...
def EstimateAbsolutePoseWithKnownOrientation(*args, **kwargs):
    ...
def EstimateCalibratedAbsolutePose(*args, **kwargs):
    ...
def EstimateDominantPlaneFromPoints(*args, **kwargs):
    ...
def EstimateEssentialMatrix(*args, **kwargs):
    ...
def EstimateFundamentalMatrix(*args, **kwargs):
    ...
def EstimateHomography(*args, **kwargs):
    ...
def EstimateRadialHomographyMatrix(*args, **kwargs):
    ...
def EstimateRelativePose(*args, **kwargs):
    ...
def EstimateRelativePoseWithKnownOrientation(*args, **kwargs):
    ...
def EstimateRigidTransformation2D3D(*args, **kwargs):
    ...
def EstimateRigidTransformation2D3DNormalized(*args, **kwargs):
    ...
def EstimateTriangulation(*args, **kwargs):
    ...
def EstimateTwoViewInfo(*args, **kwargs):
    ...
def EstimateUncalibratedAbsolutePose(*args, **kwargs):
    ...
def EstimateUncalibratedRelativePose(*args, **kwargs):
    ...
def ExtractMaximallyParallelRigidSubgraph(*args, **kwargs):
    ...
def FilterViewGraphCyclesByRotation(*args, **kwargs):
    ...
def FilterViewPairsFromOrientation(*args, **kwargs):
    ...
def FilterViewPairsFromRelativeTranslation(*args, **kwargs):
    ...
def FindCommonTracksByFeatureInReconstructions(*args, **kwargs):
    ...
def FindCommonTracksInViews(*args, **kwargs):
    ...
def FindCommonViewsByName(*args, **kwargs):
    ...
def FivePointFocalLengthRadialDistortion(*args, **kwargs):
    ...
def FivePointRelativePose(*args, **kwargs):
    ...
def FocalLengthsFromFundamentalMatrix(*args, **kwargs):
    ...
def FourPointHomography(*args, **kwargs):
    ...
def FourPointPoseAndFocalLength(*args, **kwargs):
    ...
def FourPointRelativePosePartialRotation(*args, **kwargs):
    ...
def FourPointsPoseFocalLengthRadialDistortion(*args, **kwargs):
    ...
def FundamentalMatrixFromProjectionMatrices(*args, **kwargs):
    ...
def GdlsSimilarityTransform(*args, **kwargs):
    ...
def GetBestPoseFromEssentialMatrix(*args, **kwargs):
    ...
def GetEstimatedTracksFromReconstruction(*args, **kwargs):
    ...
def GetEstimatedViewsFromReconstruction(*args, **kwargs):
    ...
def IntrinsicsToCalibrationMatrix(*args, **kwargs):
    ...
def IsTriangulatedPointInFrontOfCameras(*args, **kwargs):
    ...
def LocalizeViewToReconstruction(*args, **kwargs):
    ...
def MLPnP(*args, **kwargs):
    ...
def NormalizedEightPointFundamentalMatrix(*args, **kwargs):
    ...
def NumEstimatedTracks(*args, **kwargs):
    ...
def NumEstimatedViews(*args, **kwargs):
    ...
def OptimizeAbsolutePoseOnNormFeatures(*args, **kwargs):
    ...
def OptimizeAlignmentSim3(*args, **kwargs):
    ...
def OptimizeRelativePositionWithKnownRotation(*args, **kwargs):
    ...
def PlanarUncalibratedOrthographicPose(*args, **kwargs):
    ...
def PoseFromThreePoints(*args, **kwargs):
    ...
def PositionFromTwoRays(*args, **kwargs):
    ...
def ProjectionMatricesFromFundamentalMatrix(*args, **kwargs):
    ...
def RelativePoseFromTwoPointsWithKnownRotation(*args, **kwargs):
    ...
def RelativeRotationsFromViewGraph(*args, **kwargs):
    ...
def RemoveDisconnectedViewPairs(*args, **kwargs):
    ...
def SelectGoodTracksForBundleAdjustment(*args, **kwargs):
    ...
def SetCameraIntrinsicsFromPriors(*args, **kwargs):
    ...
def SetOutlierTracksToUnestimated(*args, **kwargs):
    ...
def SetReconstructionFromEstimatedPoses(*args, **kwargs):
    ...
def SetUnderconstrainedTracksToUnestimated(*args, **kwargs):
    ...
def SetUnderconstrainedViewsToUnestimated(*args, **kwargs):
    ...
def SevenPointFundamentalMatrix(*args, **kwargs):
    ...
def SharedFocalLengthsFromFundamentalMatrix(*args, **kwargs):
    ...
def Sim3FromRotationTranslationScale(*args, **kwargs):
    ...
def Sim3ToHomogeneousMatrix(*args, **kwargs):
    ...
def Sim3ToRotationTranslationScale(*args, **kwargs):
    ...
def SimTransformPartialRotation(*args, **kwargs):
    ...
def SufficientTriangulationAngle(*args, **kwargs):
    ...
def SwapCameras(*args, **kwargs):
    ...
def ThreePointRelativePosePartialRotation(*args, **kwargs):
    ...
def TransformReconstruction(*args, **kwargs):
    ...
def TransformReconstruction4(*args, **kwargs):
    ...
def Triangulate(*args, **kwargs):
    ...
def TriangulateDLT(*args, **kwargs):
    ...
def TriangulateMidpoint(*args, **kwargs):
    ...
def TriangulateNView(*args, **kwargs):
    ...
def TriangulateNViewSVD(*args, **kwargs):
    ...
def TwoPointPosePartialRotation(*args, **kwargs):
    ...
def UpdateFeaturesInView(*args, **kwargs):
    ...
ACCELERATE_SPARSE: SparseLinearAlgebraLibraryType  # value = <SparseLinearAlgebraLibraryType.ACCELERATE_SPARSE: 2>
ALL: OptimizeIntrinsicsType  # value = <OptimizeIntrinsicsType.ALL: 63>
ARCTAN: LossFunctionType  # value = <LossFunctionType.ARCTAN: 4>
ASPECT_RATIO: OptimizeIntrinsicsType  # value = <OptimizeIntrinsicsType.ASPECT_RATIO: 2>
CANONICAL_VIEWS: VisibilityClusteringType  # value = <VisibilityClusteringType.CANONICAL_VIEWS: 0>
CAUCHY: LossFunctionType  # value = <LossFunctionType.CAUCHY: 3>
CGNR: LinearSolverType  # value = <LinearSolverType.CGNR: 6>
CLUSTER_JACOBI: PreconditionerType  # value = <PreconditionerType.CLUSTER_JACOBI: 4>
CLUSTER_TRIDIAGONAL: PreconditionerType  # value = <PreconditionerType.CLUSTER_TRIDIAGONAL: 5>
CUDA: DenseLinearAlgebraLibraryType  # value = <DenseLinearAlgebraLibraryType.CUDA: 2>
CUDA_SPARSE: SparseLinearAlgebraLibraryType  # value = <SparseLinearAlgebraLibraryType.CUDA_SPARSE: 3>
DENSE_NORMAL_CHOLESKY: LinearSolverType  # value = <LinearSolverType.DENSE_NORMAL_CHOLESKY: 0>
DENSE_QR: LinearSolverType  # value = <LinearSolverType.DENSE_QR: 1>
DENSE_SCHUR: LinearSolverType  # value = <LinearSolverType.DENSE_SCHUR: 3>
DISTORTION: OptimizeIntrinsicsType  # value = <OptimizeIntrinsicsType.DISTORTION: 48>
DIVISION_UNDISTORTION: CameraIntrinsicsModelType  # value = <CameraIntrinsicsModelType.DIVISION_UNDISTORTION: 4>
DLS: PnPType  # value = <PnPType.DLS: 2>
DOUBLE_SPHERE: CameraIntrinsicsModelType  # value = <CameraIntrinsicsModelType.DOUBLE_SPHERE: 5>
EIGEN: DenseLinearAlgebraLibraryType  # value = <DenseLinearAlgebraLibraryType.EIGEN: 0>
EIGEN_SPARSE: SparseLinearAlgebraLibraryType  # value = <SparseLinearAlgebraLibraryType.EIGEN_SPARSE: 1>
EXHAUSTIVE: RansacType  # value = <RansacType.EXHAUSTIVE: 3>
EXTENDED_UNIFIED: CameraIntrinsicsModelType  # value = <CameraIntrinsicsModelType.EXTENDED_UNIFIED: 6>
FISHEYE: CameraIntrinsicsModelType  # value = <CameraIntrinsicsModelType.FISHEYE: 2>
FOCAL_LENGTH: OptimizeIntrinsicsType  # value = <OptimizeIntrinsicsType.FOCAL_LENGTH: 1>
FOCAL_LENGTH_DISTORTION: OptimizeIntrinsicsType  # value = <OptimizeIntrinsicsType.FOCAL_LENGTH_DISTORTION: 49>
FOCAL_LENGTH_RADIAL_DISTORTION: OptimizeIntrinsicsType  # value = <OptimizeIntrinsicsType.FOCAL_LENGTH_RADIAL_DISTORTION: 17>
FOV: CameraIntrinsicsModelType  # value = <CameraIntrinsicsModelType.FOV: 3>
GLOBAL: ReconstructionEstimatorType  # value = <ReconstructionEstimatorType.GLOBAL: 0>
HUBER: LossFunctionType  # value = <LossFunctionType.HUBER: 1>
HYBRID: GlobalRotationEstimatorType  # value = <GlobalRotationEstimatorType.HYBRID: 4>
IDENTITY: PreconditionerType  # value = <PreconditionerType.IDENTITY: 0>
INCREMENTAL: ReconstructionEstimatorType  # value = <ReconstructionEstimatorType.INCREMENTAL: 1>
INVALID: CameraIntrinsicsModelType  # value = <CameraIntrinsicsModelType.INVALID: -1>
INVERSE_DEPTH: TrackParametrizationType  # value = <TrackParametrizationType.INVERSE_DEPTH: 2>
ITERATIVE_SCHUR: LinearSolverType  # value = <LinearSolverType.ITERATIVE_SCHUR: 5>
JACOBI: PreconditionerType  # value = <PreconditionerType.JACOBI: 1>
KNEIP: PnPType  # value = <PnPType.KNEIP: 0>
L2_MINIMIZATION: TriangulationMethodType  # value = <TriangulationMethodType.L2_MINIMIZATION: 2>
LAGRANGE_DUAL: GlobalRotationEstimatorType  # value = <GlobalRotationEstimatorType.LAGRANGE_DUAL: 3>
LAPACK: DenseLinearAlgebraLibraryType  # value = <DenseLinearAlgebraLibraryType.LAPACK: 1>
LEAST_UNSQUARED_DEVIATION: GlobalPositionEstimatorType  # value = <GlobalPositionEstimatorType.LEAST_UNSQUARED_DEVIATION: 2>
LIGT: GlobalPositionEstimatorType  # value = <GlobalPositionEstimatorType.LIGT: 3>
LINEAR: GlobalRotationEstimatorType  # value = <GlobalRotationEstimatorType.LINEAR: 2>
LINEAR_TRIPLET: GlobalPositionEstimatorType  # value = <GlobalPositionEstimatorType.LINEAR_TRIPLET: 1>
LMED: RansacType  # value = <RansacType.LMED: 2>
MIDPOINT: TriangulationMethodType  # value = <TriangulationMethodType.MIDPOINT: 0>
NONE: OptimizeIntrinsicsType  # value = <OptimizeIntrinsicsType.NONE: 0>
NONLINEAR: GlobalRotationEstimatorType  # value = <GlobalRotationEstimatorType.NONLINEAR: 1>
ORTHOGRAPHIC: CameraIntrinsicsModelType  # value = <CameraIntrinsicsModelType.ORTHOGRAPHIC: 7>
PINHOLE: CameraIntrinsicsModelType  # value = <CameraIntrinsicsModelType.PINHOLE: 0>
PINHOLE_RADIAL_TANGENTIAL: CameraIntrinsicsModelType  # value = <CameraIntrinsicsModelType.PINHOLE_RADIAL_TANGENTIAL: 1>
POINT_TO_PLANE: Sim3AlignmentType  # value = <Sim3AlignmentType.POINT_TO_PLANE: 2>
POINT_TO_POINT: Sim3AlignmentType  # value = <Sim3AlignmentType.POINT_TO_POINT: 0>
PRINCIPAL_POINTS: OptimizeIntrinsicsType  # value = <OptimizeIntrinsicsType.PRINCIPAL_POINTS: 8>
PROSAC: RansacType  # value = <RansacType.PROSAC: 1>
RADIAL_DISTORTION: OptimizeIntrinsicsType  # value = <OptimizeIntrinsicsType.RADIAL_DISTORTION: 16>
RANSAC: RansacType  # value = <RansacType.RANSAC: 0>
ROBUST_L1L2: GlobalRotationEstimatorType  # value = <GlobalRotationEstimatorType.ROBUST_L1L2: 0>
ROBUST_POINT_TO_POINT: Sim3AlignmentType  # value = <Sim3AlignmentType.ROBUST_POINT_TO_POINT: 1>
SCHUR_JACOBI: PreconditionerType  # value = <PreconditionerType.SCHUR_JACOBI: 2>
SINGLE_LINKAGE: VisibilityClusteringType  # value = <VisibilityClusteringType.SINGLE_LINKAGE: 1>
SKEW: OptimizeIntrinsicsType  # value = <OptimizeIntrinsicsType.SKEW: 4>
SOFTLONE: LossFunctionType  # value = <LossFunctionType.SOFTLONE: 2>
SPARSE_NORMAL_CHOLESKY: LinearSolverType  # value = <LinearSolverType.SPARSE_NORMAL_CHOLESKY: 2>
SPARSE_SCHUR: LinearSolverType  # value = <LinearSolverType.SPARSE_SCHUR: 4>
SQPnP: PnPType  # value = <PnPType.SQPnP: 1>
SUITE_SPARSE: SparseLinearAlgebraLibraryType  # value = <SparseLinearAlgebraLibraryType.SUITE_SPARSE: 0>
SVD: TriangulationMethodType  # value = <TriangulationMethodType.SVD: 1>
TANGENTIAL_DISTORTION: OptimizeIntrinsicsType  # value = <OptimizeIntrinsicsType.TANGENTIAL_DISTORTION: 32>
TRIVIAL: LossFunctionType  # value = <LossFunctionType.TRIVIAL: 0>
TUKEY: LossFunctionType  # value = <LossFunctionType.TUKEY: 5>
XYZW: TrackParametrizationType  # value = <TrackParametrizationType.XYZW: 0>
XYZW_MANIFOLD: TrackParametrizationType  # value = <TrackParametrizationType.XYZW_MANIFOLD: 1>
kInvalidTrackId: int = 4294967295
kInvalidViewId: int = 4294967295
