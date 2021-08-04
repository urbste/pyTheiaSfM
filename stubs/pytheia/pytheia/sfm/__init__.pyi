import pytheia.pytheia.sfm
import typing
import numpy
import pytheia.pytheia.matching
_Shape = typing.Tuple[int, ...]

__all__ = [
    "ALL",
    "ARCTAN",
    "ASPECT_RATIO",
    "AlignPointCloudsUmeyama",
    "AlignPointCloudsUmeyamaWithWeights",
    "AlignReconstructions",
    "AlignReconstructionsRobust",
    "AlignRotations",
    "BundleAdjustPartialReconstruction",
    "BundleAdjustReconstruction",
    "BundleAdjustTrack",
    "BundleAdjustTrackWithCov",
    "BundleAdjustTracksWithCov",
    "BundleAdjustTwoViews",
    "BundleAdjustTwoViewsAngular",
    "BundleAdjustView",
    "BundleAdjustViewWithCov",
    "BundleAdjustViewsWithCov",
    "BundleAdjuster",
    "BundleAdjustmentOptions",
    "BundleAdjustmentSummary",
    "CAUCHY",
    "CalibratedAbsolutePose",
    "CalibrationMatrixToIntrinsics",
    "Camera",
    "CameraAndFeatureCorrespondence2D3D",
    "CameraIntrinsicsModel",
    "CameraIntrinsicsModelType",
    "CameraIntrinsicsPrior",
    "ColorizeReconstruction",
    "ComposeFundamentalMatrix",
    "ComposeProjectionMatrix",
    "ComputeTripletBaselineRatios",
    "DISTORTION",
    "DIVISION_UNDISTORTION",
    "DOUBLE_SPHERE",
    "DecomposeEssentialMatrix",
    "DecomposeProjectionMatrix",
    "DivisionUndistortionCameraModel",
    "DlsPnp",
    "EXHAUSTIVE",
    "EXTENDED_UNIFIED",
    "EssentialMatrixFromFundamentalMatrix",
    "EssentialMatrixFromTwoProjectionMatrices",
    "EstimateAbsolutePoseWithKnownOrientation",
    "EstimateCalibratedAbsolutePose",
    "EstimateDominantPlaneFromPoints",
    "EstimateEssentialMatrix",
    "EstimateFundamentalMatrix",
    "EstimateHomography",
    "EstimateRadialHomographyMatrix",
    "EstimateRelativePose",
    "EstimateRelativePoseWithKnownOrientation",
    "EstimateRigidTransformation2D3D",
    "EstimateRigidTransformation2D3DNormalized",
    "EstimateTriangulation",
    "EstimateTwoViewInfo",
    "EstimateTwoViewInfoOptions",
    "EstimateUncalibratedAbsolutePose",
    "EstimateUncalibratedRelativePose",
    "ExtractMaximallyParallelRigidSubgraph",
    "FISHEYE",
    "FOCAL_LENGTH",
    "FOV",
    "FOVCameraModel",
    "Feature",
    "FeatureCorrespondence2D3D",
    "FilterViewGraphCyclesByRotation",
    "FilterViewPairsFromOrientation",
    "FilterViewPairsFromRelativeTranslation",
    "FilterViewPairsFromRelativeTranslationOptions",
    "FindCommonTracksInViews",
    "FindCommonViewsByName",
    "FisheyeCameraModel",
    "FivePointFocalLengthRadialDistortion",
    "FivePointRelativePose",
    "FocalLengthsFromFundamentalMatrix",
    "FourPointHomography",
    "FourPointPoseAndFocalLength",
    "FourPointRelativePosePartialRotation",
    "FourPointsPoseFocalLengthRadialDistortion",
    "FundamentalMatrixFromProjectionMatrices",
    "GLOBAL",
    "GPSConverter",
    "GdlsSimilarityTransform",
    "GetBestPoseFromEssentialMatrix",
    "GlobalPositionEstimatorType",
    "GlobalReconstructionEstimator",
    "GlobalRotationEstimatorType",
    "HUBER",
    "HYBRID",
    "HybridReconstructionEstimator",
    "INCREMENTAL",
    "INVALID",
    "IncrementalReconstructionEstimator",
    "IntrinsicsToCalibrationMatrix",
    "IsTriangulatedPointInFrontOfCameras",
    "LEAST_UNSQUARED_DEVIATION",
    "LINEAR",
    "LINEAR_TRIPLET",
    "LMED",
    "LeastUnsquaredDeviationPositionEstimator",
    "LinearPositionEstimator",
    "LinearRotationEstimator",
    "LocalizeViewToReconstruction",
    "LocalizeViewToReconstructionOptions",
    "LossFunctionType",
    "NONE",
    "NONLINEAR",
    "NonlinearPositionEstimator",
    "NonlinearRotationEstimator",
    "NormalizedEightPointFundamentalMatrix",
    "OptimizeIntrinsicsType",
    "OptimizeRelativePositionWithKnownRotation",
    "PINHOLE",
    "PINHOLE_RADIAL_TANGENTIAL",
    "PRINCIPAL_POINTS",
    "PROSAC",
    "PinholeCameraModel",
    "PinholeRadialTangentialCameraModel",
    "Plane",
    "PoseFromThreePoints",
    "PositionEstimator",
    "PositionFromTwoRays",
    "PriorScalar",
    "PriorVector2d",
    "PriorVector3d",
    "PriorVector4d",
    "ProjectionMatricesFromFundamentalMatrix",
    "RADIAL_DISTORTION",
    "RANSAC",
    "ROBUST_L1L2",
    "RadialDistortionFeatureCorrespondence",
    "RansacType",
    "Reconstruction",
    "ReconstructionBuilder",
    "ReconstructionBuilderOptions",
    "ReconstructionEstimator",
    "ReconstructionEstimatorOptions",
    "ReconstructionEstimatorSummary",
    "ReconstructionEstimatorType",
    "RelativePose",
    "RelativePoseFromTwoPointsWithKnownRotation",
    "RigidTransformation",
    "RobustRotationEstimator",
    "RotationEstimator",
    "SKEW",
    "SOFTLONE",
    "SelectGoodTracksForBundleAdjustment",
    "SelectGoodTracksForBundleAdjustmentAll",
    "SetCameraIntrinsicsFromPriors",
    "SetOutlierTracksToUnestimated",
    "SetOutlierTracksToUnestimatedAll",
    "SevenPointFundamentalMatrix",
    "SharedFocalLengthsFromFundamentalMatrix",
    "SimTransformPartialRotation",
    "SimilarityTransformation",
    "SufficientTriangulationAngle",
    "SwapCameras",
    "TANGENTIAL_DISTORTION",
    "TRIVIAL",
    "TUKEY",
    "ThreePointRelativePosePartialRotation",
    "Track",
    "TrackBuilder",
    "TrackEstimator",
    "TrackEstimatorOptions",
    "TrackEstimatorSummary",
    "TransformReconstruction",
    "Triangulate",
    "TriangulateDLT",
    "TriangulateMidpoint",
    "TriangulateNView",
    "TriangulateNViewSVD",
    "TwoPointPosePartialRotation",
    "TwoViewBundleAdjustmentOptions",
    "TwoViewInfo",
    "TwoViewMatchGeometricVerification",
    "TwoViewMatchGeometricVerificationOptions",
    "UncalibratedAbsolutePose",
    "UncalibratedRelativePose",
    "View",
    "ViewGraph",
    "VisibilityPyramid",
    "kInvalidTrackId",
    "kInvalidViewId"
]


class BundleAdjuster():
    def AddTrack(self, arg0: int, arg1: bool) -> None: ...
    def AddView(self, arg0: int) -> None: ...
    def Optimize(self) -> BundleAdjustmentSummary: ...
    pass
class BundleAdjustmentOptions():
    def __init__(self) -> None: ...
    @property
    def constant_camera_orientation(self) -> bool:
        """
        :type: bool
        """
    @constant_camera_orientation.setter
    def constant_camera_orientation(self, arg0: bool) -> None:
        pass
    @property
    def constant_camera_position(self) -> bool:
        """
        :type: bool
        """
    @constant_camera_position.setter
    def constant_camera_position(self, arg0: bool) -> None:
        pass
    @property
    def function_tolerance(self) -> float:
        """
        :type: float
        """
    @function_tolerance.setter
    def function_tolerance(self, arg0: float) -> None:
        pass
    @property
    def gradient_tolerance(self) -> float:
        """
        :type: float
        """
    @gradient_tolerance.setter
    def gradient_tolerance(self, arg0: float) -> None:
        pass
    @property
    def intrinsics_to_optimize(self) -> theia::OptimizeIntrinsicsType:
        """
        :type: theia::OptimizeIntrinsicsType
        """
    @intrinsics_to_optimize.setter
    def intrinsics_to_optimize(self, arg0: theia::OptimizeIntrinsicsType) -> None:
        pass
    @property
    def loss_function_type(self) -> theia::LossFunctionType:
        """
        :type: theia::LossFunctionType
        """
    @loss_function_type.setter
    def loss_function_type(self, arg0: theia::LossFunctionType) -> None:
        pass
    @property
    def max_num_iterations(self) -> int:
        """
        :type: int
        """
    @max_num_iterations.setter
    def max_num_iterations(self, arg0: int) -> None:
        pass
    @property
    def max_solver_time_in_seconds(self) -> float:
        """
        :type: float
        """
    @max_solver_time_in_seconds.setter
    def max_solver_time_in_seconds(self, arg0: float) -> None:
        pass
    @property
    def max_trust_region_radius(self) -> float:
        """
        :type: float
        """
    @max_trust_region_radius.setter
    def max_trust_region_radius(self, arg0: float) -> None:
        pass
    @property
    def num_threads(self) -> int:
        """
        :type: int
        """
    @num_threads.setter
    def num_threads(self, arg0: int) -> None:
        pass
    @property
    def parameter_tolerance(self) -> float:
        """
        :type: float
        """
    @parameter_tolerance.setter
    def parameter_tolerance(self, arg0: float) -> None:
        pass
    @property
    def robust_loss_width(self) -> float:
        """
        :type: float
        """
    @robust_loss_width.setter
    def robust_loss_width(self, arg0: float) -> None:
        pass
    @property
    def use_inner_iterations(self) -> bool:
        """
        :type: bool
        """
    @use_inner_iterations.setter
    def use_inner_iterations(self, arg0: bool) -> None:
        pass
    @property
    def verbose(self) -> bool:
        """
        :type: bool
        """
    @verbose.setter
    def verbose(self, arg0: bool) -> None:
        pass
    pass
class BundleAdjustmentSummary():
    def __init__(self) -> None: ...
    @property
    def final_cost(self) -> float:
        """
        :type: float
        """
    @final_cost.setter
    def final_cost(self, arg0: float) -> None:
        pass
    @property
    def initial_cost(self) -> float:
        """
        :type: float
        """
    @initial_cost.setter
    def initial_cost(self, arg0: float) -> None:
        pass
    @property
    def setup_time_in_seconds(self) -> float:
        """
        :type: float
        """
    @setup_time_in_seconds.setter
    def setup_time_in_seconds(self, arg0: float) -> None:
        pass
    @property
    def solve_time_in_seconds(self) -> float:
        """
        :type: float
        """
    @solve_time_in_seconds.setter
    def solve_time_in_seconds(self, arg0: float) -> None:
        pass
    @property
    def success(self) -> bool:
        """
        :type: bool
        """
    @success.setter
    def success(self, arg0: bool) -> None:
        pass
    pass
class CalibratedAbsolutePose():
    def __init__(self) -> None: ...
    @property
    def position(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    @position.setter
    def position(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None:
        pass
    @property
    def rotation(self) -> numpy.ndarray[numpy.float64, _Shape[3, 3]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 3]]
        """
    @rotation.setter
    def rotation(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]]) -> None:
        pass
    pass
class Camera():
    def CameraIntrinsics(self) -> CameraIntrinsicsModel: ...
    @staticmethod
    def CameraIntrinsicsPriorFromIntrinsics(*args, **kwargs) -> typing.Any: ...
    def DeepCopy(self, arg0: Camera) -> None: ...
    def GetCalibrationMatrix(self) -> numpy.ndarray[numpy.float64, _Shape[3, 3]]: ...
    @staticmethod
    def GetCameraIntrinsicsModelType(*args, **kwargs) -> typing.Any: ...
    def GetOrientationAsAngleAxis(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]: ...
    def GetOrientationAsRotationMatrix(self) -> numpy.ndarray[numpy.float64, _Shape[3, 3]]: ...
    def GetProjectionMatrix(self) -> numpy.ndarray[numpy.float64, _Shape[3, 4]]: ...
    def InitializeFromProjectionMatrix(self, arg0: int, arg1: int, arg2: numpy.ndarray[numpy.float64, _Shape[3, 4]]) -> bool: ...
    def PixelToNormalizedCoordinates(self, arg0: numpy.ndarray[numpy.float64, _Shape[2, 1]]) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]: ...
    def PixelToUnitDepthRay(self, arg0: numpy.ndarray[numpy.float64, _Shape[2, 1]]) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]: ...
    def PrintCameraIntrinsics(self) -> None: ...
    def ProjectPoint(self, arg0: numpy.ndarray[numpy.float64, _Shape[4, 1]]) -> typing.Tuple[float, numpy.ndarray[numpy.float64, _Shape[2, 1]]]: ...
    @staticmethod
    def SetCameraIntrinsicsModelType(*args, **kwargs) -> typing.Any: ...
    @staticmethod
    def SetFromCameraIntrinsicsPriors(*args, **kwargs) -> typing.Any: ...
    def SetImageSize(self, arg0: int, arg1: int) -> None: ...
    def SetOrientationFromAngleAxis(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None: ...
    def SetOrientationFromRotationMatrix(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]]) -> None: ...
    def SetPrincipalPoint(self, arg0: float, arg1: float) -> None: ...
    @staticmethod
    @typing.overload
    def __init__(*args, **kwargs) -> typing.Any: ...
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, arg0: Camera) -> None: ...
    @property
    def FocalLength(self) -> float:
        """
        :type: float
        """
    @FocalLength.setter
    def FocalLength(self, arg1: float) -> None:
        pass
    @property
    def ImageHeight(self) -> int:
        """
        :type: int
        """
    @property
    def ImageWidth(self) -> int:
        """
        :type: int
        """
    @property
    def Position(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    @Position.setter
    def Position(self, arg1: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None:
        pass
    @property
    def PrincipalPointX(self) -> float:
        """
        :type: float
        """
    @property
    def PrincipalPointY(self) -> float:
        """
        :type: float
        """
    pass
class CameraAndFeatureCorrespondence2D3D():
    def __init__(self) -> None: ...
    @property
    def camera(self) -> Camera:
        """
        :type: Camera
        """
    @camera.setter
    def camera(self, arg0: Camera) -> None:
        pass
    @property
    def observation(self) -> Feature:
        """
        :type: Feature
        """
    @observation.setter
    def observation(self, arg0: Feature) -> None:
        pass
    @property
    def point3d(self) -> numpy.ndarray[numpy.float64, _Shape[4, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[4, 1]]
        """
    @point3d.setter
    def point3d(self, arg0: numpy.ndarray[numpy.float64, _Shape[4, 1]]) -> None:
        pass
    pass
class CameraIntrinsicsModel():
    def CameraToImageCoordinates(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> numpy.ndarray[numpy.float64, _Shape[2, 1]]: ...
    def GetParameter(self, arg0: int) -> float: ...
    def ImageToCameraCoordinates(self, arg0: numpy.ndarray[numpy.float64, _Shape[2, 1]]) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]: ...
    def SetParameter(self, arg0: int, arg1: float) -> None: ...
    def SetPrincipalPoint(self, arg0: float, arg1: float) -> None: ...
    @property
    def FocalLength(self) -> float:
        """
        :type: float
        """
    @FocalLength.setter
    def FocalLength(self, arg1: float) -> None:
        pass
    @property
    def PrincipalPointX(self) -> float:
        """
        :type: float
        """
    @property
    def PrincipalPointY(self) -> float:
        """
        :type: float
        """
    pass
class CameraIntrinsicsModelType():
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
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    DIVISION_UNDISTORTION: pytheia.pytheia.sfm.CameraIntrinsicsModelType # value = <CameraIntrinsicsModelType.DIVISION_UNDISTORTION: 4>
    DOUBLE_SPHERE: pytheia.pytheia.sfm.CameraIntrinsicsModelType # value = <CameraIntrinsicsModelType.DOUBLE_SPHERE: 5>
    EXTENDED_UNIFIED: pytheia.pytheia.sfm.CameraIntrinsicsModelType # value = <CameraIntrinsicsModelType.EXTENDED_UNIFIED: 6>
    FISHEYE: pytheia.pytheia.sfm.CameraIntrinsicsModelType # value = <CameraIntrinsicsModelType.FISHEYE: 2>
    FOV: pytheia.pytheia.sfm.CameraIntrinsicsModelType # value = <CameraIntrinsicsModelType.FOV: 3>
    INVALID: pytheia.pytheia.sfm.CameraIntrinsicsModelType # value = <CameraIntrinsicsModelType.INVALID: -1>
    PINHOLE: pytheia.pytheia.sfm.CameraIntrinsicsModelType # value = <CameraIntrinsicsModelType.PINHOLE: 0>
    PINHOLE_RADIAL_TANGENTIAL: pytheia.pytheia.sfm.CameraIntrinsicsModelType # value = <CameraIntrinsicsModelType.PINHOLE_RADIAL_TANGENTIAL: 1>
    __members__: dict # value = {'INVALID': <CameraIntrinsicsModelType.INVALID: -1>, 'PINHOLE': <CameraIntrinsicsModelType.PINHOLE: 0>, 'PINHOLE_RADIAL_TANGENTIAL': <CameraIntrinsicsModelType.PINHOLE_RADIAL_TANGENTIAL: 1>, 'FISHEYE': <CameraIntrinsicsModelType.FISHEYE: 2>, 'FOV': <CameraIntrinsicsModelType.FOV: 3>, 'DIVISION_UNDISTORTION': <CameraIntrinsicsModelType.DIVISION_UNDISTORTION: 4>, 'DOUBLE_SPHERE': <CameraIntrinsicsModelType.DOUBLE_SPHERE: 5>, 'EXTENDED_UNIFIED': <CameraIntrinsicsModelType.EXTENDED_UNIFIED: 6>}
    pass
class CameraIntrinsicsPrior():
    def __init__(self) -> None: ...
    @property
    def altitude(self) -> PriorScalar:
        """
        :type: PriorScalar
        """
    @altitude.setter
    def altitude(self, arg0: PriorScalar) -> None:
        pass
    @property
    def aspect_ratio(self) -> PriorScalar:
        """
        :type: PriorScalar
        """
    @aspect_ratio.setter
    def aspect_ratio(self, arg0: PriorScalar) -> None:
        pass
    @property
    def camera_intrinsics_model_type(self) -> str:
        """
        :type: str
        """
    @camera_intrinsics_model_type.setter
    def camera_intrinsics_model_type(self, arg0: str) -> None:
        pass
    @property
    def focal_length(self) -> PriorScalar:
        """
        :type: PriorScalar
        """
    @focal_length.setter
    def focal_length(self, arg0: PriorScalar) -> None:
        pass
    @property
    def image_height(self) -> int:
        """
        :type: int
        """
    @image_height.setter
    def image_height(self, arg0: int) -> None:
        pass
    @property
    def image_width(self) -> int:
        """
        :type: int
        """
    @image_width.setter
    def image_width(self, arg0: int) -> None:
        pass
    @property
    def latitude(self) -> PriorScalar:
        """
        :type: PriorScalar
        """
    @latitude.setter
    def latitude(self, arg0: PriorScalar) -> None:
        pass
    @property
    def longitude(self) -> PriorScalar:
        """
        :type: PriorScalar
        """
    @longitude.setter
    def longitude(self, arg0: PriorScalar) -> None:
        pass
    @property
    def orientation(self) -> PriorVector3d:
        """
        :type: PriorVector3d
        """
    @orientation.setter
    def orientation(self, arg0: PriorVector3d) -> None:
        pass
    @property
    def position(self) -> PriorVector3d:
        """
        :type: PriorVector3d
        """
    @position.setter
    def position(self, arg0: PriorVector3d) -> None:
        pass
    @property
    def principal_point(self) -> PriorVector2d:
        """
        :type: PriorVector2d
        """
    @principal_point.setter
    def principal_point(self, arg0: PriorVector2d) -> None:
        pass
    @property
    def radial_distortion(self) -> PriorVector4d:
        """
        :type: PriorVector4d
        """
    @radial_distortion.setter
    def radial_distortion(self, arg0: PriorVector4d) -> None:
        pass
    @property
    def skew(self) -> PriorScalar:
        """
        :type: PriorScalar
        """
    @skew.setter
    def skew(self, arg0: PriorScalar) -> None:
        pass
    @property
    def tangential_distortion(self) -> PriorVector2d:
        """
        :type: PriorVector2d
        """
    @tangential_distortion.setter
    def tangential_distortion(self, arg0: PriorVector2d) -> None:
        pass
    pass
class DivisionUndistortionCameraModel(CameraIntrinsicsModel):
    @staticmethod
    def CameraIntrinsicsPriorFromIntrinsics(*args, **kwargs) -> typing.Any: ...
    def GetCalibrationMatrix(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]]) -> None: ...
    @staticmethod
    def GetSubsetFromOptimizeIntrinsicsType(*args, **kwargs) -> typing.Any: ...
    def NumParameters(self) -> int: ...
    def PrintIntrinsics(self) -> None: ...
    @staticmethod
    def SetFromCameraIntrinsicsPriors(*args, **kwargs) -> typing.Any: ...
    def SetRadialDistortion(self, arg0: float) -> None: ...
    @staticmethod
    def Type(*args, **kwargs) -> typing.Any: ...
    def __init__(self) -> None: ...
    @property
    def AspectRatio(self) -> float:
        """
        :type: float
        """
    @AspectRatio.setter
    def AspectRatio(self, arg1: float) -> None:
        pass
    @property
    def RadialDistortion1(self) -> float:
        """
        :type: float
        """
    @property
    def kIntrinsicsSize(self) -> int:
        """
        :type: int
        """
    pass
class EstimateTwoViewInfoOptions():
    def __init__(self) -> None: ...
    @property
    def expected_ransac_confidence(self) -> float:
        """
        :type: float
        """
    @expected_ransac_confidence.setter
    def expected_ransac_confidence(self, arg0: float) -> None:
        pass
    @property
    def max_ransac_iterations(self) -> int:
        """
        :type: int
        """
    @max_ransac_iterations.setter
    def max_ransac_iterations(self, arg0: int) -> None:
        pass
    @property
    def max_sampson_error_pixels(self) -> float:
        """
        :type: float
        """
    @max_sampson_error_pixels.setter
    def max_sampson_error_pixels(self, arg0: float) -> None:
        pass
    @property
    def min_ransac_iterations(self) -> int:
        """
        :type: int
        """
    @min_ransac_iterations.setter
    def min_ransac_iterations(self, arg0: int) -> None:
        pass
    @property
    def ransac_type(self) -> RansacType:
        """
        :type: RansacType
        """
    @ransac_type.setter
    def ransac_type(self, arg0: RansacType) -> None:
        pass
    @property
    def use_mle(self) -> bool:
        """
        :type: bool
        """
    @use_mle.setter
    def use_mle(self, arg0: bool) -> None:
        pass
    pass
class FOVCameraModel(CameraIntrinsicsModel):
    @staticmethod
    def CameraIntrinsicsPriorFromIntrinsics(*args, **kwargs) -> typing.Any: ...
    def GetCalibrationMatrix(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]]) -> None: ...
    @staticmethod
    def GetSubsetFromOptimizeIntrinsicsType(*args, **kwargs) -> typing.Any: ...
    def NumParameters(self) -> int: ...
    def PrintIntrinsics(self) -> None: ...
    @staticmethod
    def SetFromCameraIntrinsicsPriors(*args, **kwargs) -> typing.Any: ...
    def SetRadialDistortion(self, arg0: float) -> None: ...
    @staticmethod
    def Type(*args, **kwargs) -> typing.Any: ...
    def __init__(self) -> None: ...
    @property
    def AspectRatio(self) -> float:
        """
        :type: float
        """
    @AspectRatio.setter
    def AspectRatio(self, arg1: float) -> None:
        pass
    @property
    def RadialDistortion1(self) -> float:
        """
        :type: float
        """
    @property
    def kIntrinsicsSize(self) -> int:
        """
        :type: int
        """
    pass
class Feature():
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, arg0: float, arg1: float) -> None: ...
    @typing.overload
    def __init__(self, arg0: numpy.ndarray[numpy.float64, _Shape[2, 1]]) -> None: ...
    @typing.overload
    def __init__(self, arg0: numpy.ndarray[numpy.float64, _Shape[2, 1]], arg1: numpy.ndarray[numpy.float64, _Shape[2, 2]]) -> None: ...
    def x(self) -> float: ...
    def y(self) -> float: ...
    @property
    def covariance(self) -> numpy.ndarray[numpy.float64, _Shape[2, 2]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[2, 2]]
        """
    @covariance.setter
    def covariance(self, arg0: numpy.ndarray[numpy.float64, _Shape[2, 2]]) -> None:
        pass
    @property
    def point(self) -> numpy.ndarray[numpy.float64, _Shape[2, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[2, 1]]
        """
    @point.setter
    def point(self, arg0: numpy.ndarray[numpy.float64, _Shape[2, 1]]) -> None:
        pass
    pass
class FeatureCorrespondence2D3D():
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, arg0: numpy.ndarray[numpy.float64, _Shape[2, 1]], arg1: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None: ...
    @property
    def feature(self) -> numpy.ndarray[numpy.float64, _Shape[2, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[2, 1]]
        """
    @feature.setter
    def feature(self, arg0: numpy.ndarray[numpy.float64, _Shape[2, 1]]) -> None:
        pass
    @property
    def world_point(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    @world_point.setter
    def world_point(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None:
        pass
    pass
class FilterViewPairsFromRelativeTranslationOptions():
    def __init__(self) -> None: ...
    @property
    def num_iterations(self) -> int:
        """
        :type: int
        """
    @num_iterations.setter
    def num_iterations(self, arg0: int) -> None:
        pass
    @property
    def num_threads(self) -> int:
        """
        :type: int
        """
    @num_threads.setter
    def num_threads(self, arg0: int) -> None:
        pass
    @property
    def translation_projection_tolerance(self) -> float:
        """
        :type: float
        """
    @translation_projection_tolerance.setter
    def translation_projection_tolerance(self, arg0: float) -> None:
        pass
    pass
class FisheyeCameraModel(CameraIntrinsicsModel):
    @staticmethod
    def CameraIntrinsicsPriorFromIntrinsics(*args, **kwargs) -> typing.Any: ...
    def GetCalibrationMatrix(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]]) -> None: ...
    @staticmethod
    def GetSubsetFromOptimizeIntrinsicsType(*args, **kwargs) -> typing.Any: ...
    def NumParameters(self) -> int: ...
    def PrintIntrinsics(self) -> None: ...
    @staticmethod
    def SetFromCameraIntrinsicsPriors(*args, **kwargs) -> typing.Any: ...
    def SetRadialDistortion(self, arg0: float, arg1: float, arg2: float, arg3: float) -> None: ...
    @staticmethod
    def Type(*args, **kwargs) -> typing.Any: ...
    def __init__(self) -> None: ...
    @property
    def AspectRatio(self) -> float:
        """
        :type: float
        """
    @AspectRatio.setter
    def AspectRatio(self, arg1: float) -> None:
        pass
    @property
    def RadialDistortion1(self) -> float:
        """
        :type: float
        """
    @property
    def RadialDistortion2(self) -> float:
        """
        :type: float
        """
    @property
    def RadialDistortion3(self) -> float:
        """
        :type: float
        """
    @property
    def RadialDistortion4(self) -> float:
        """
        :type: float
        """
    @property
    def Skew(self) -> float:
        """
        :type: float
        """
    @Skew.setter
    def Skew(self, arg1: float) -> None:
        pass
    @property
    def kIntrinsicsSize(self) -> int:
        """
        :type: int
        """
    pass
class GPSConverter():
    @staticmethod
    def ECEFToLLA(arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]: ...
    @staticmethod
    def LLAToECEF(arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]: ...
    def __init__(self) -> None: ...
    pass
class GlobalPositionEstimatorType():
    """
    Members:

      NONLINEAR

      LINEAR_TRIPLET

      LEAST_UNSQUARED_DEVIATION
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    LEAST_UNSQUARED_DEVIATION: pytheia.pytheia.sfm.GlobalPositionEstimatorType # value = <GlobalPositionEstimatorType.LEAST_UNSQUARED_DEVIATION: 2>
    LINEAR_TRIPLET: pytheia.pytheia.sfm.GlobalPositionEstimatorType # value = <GlobalPositionEstimatorType.LINEAR_TRIPLET: 1>
    NONLINEAR: pytheia.pytheia.sfm.GlobalPositionEstimatorType # value = <GlobalPositionEstimatorType.NONLINEAR: 0>
    __members__: dict # value = {'NONLINEAR': <GlobalPositionEstimatorType.NONLINEAR: 0>, 'LINEAR_TRIPLET': <GlobalPositionEstimatorType.LINEAR_TRIPLET: 1>, 'LEAST_UNSQUARED_DEVIATION': <GlobalPositionEstimatorType.LEAST_UNSQUARED_DEVIATION: 2>}
    pass
class ReconstructionEstimator():
    @staticmethod
    def Create(*args, **kwargs) -> typing.Any: ...
    pass
class GlobalRotationEstimatorType():
    """
    Members:

      ROBUST_L1L2

      NONLINEAR

      LINEAR
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    LINEAR: pytheia.pytheia.sfm.GlobalRotationEstimatorType # value = <GlobalRotationEstimatorType.LINEAR: 2>
    NONLINEAR: pytheia.pytheia.sfm.GlobalRotationEstimatorType # value = <GlobalRotationEstimatorType.NONLINEAR: 1>
    ROBUST_L1L2: pytheia.pytheia.sfm.GlobalRotationEstimatorType # value = <GlobalRotationEstimatorType.ROBUST_L1L2: 0>
    __members__: dict # value = {'ROBUST_L1L2': <GlobalRotationEstimatorType.ROBUST_L1L2: 0>, 'NONLINEAR': <GlobalRotationEstimatorType.NONLINEAR: 1>, 'LINEAR': <GlobalRotationEstimatorType.LINEAR: 2>}
    pass
class HybridReconstructionEstimator(ReconstructionEstimator):
    @staticmethod
    def Estimate(*args, **kwargs) -> typing.Any: ...
    @staticmethod
    def __init__(*args, **kwargs) -> typing.Any: ...
    pass
class IncrementalReconstructionEstimator(ReconstructionEstimator):
    @staticmethod
    def Estimate(*args, **kwargs) -> typing.Any: ...
    @staticmethod
    def __init__(*args, **kwargs) -> typing.Any: ...
    pass
class PositionEstimator():
    pass
class LinearPositionEstimator(PositionEstimator):
    def EstimatePositions(self, arg0: typing.Dict[typing.Tuple[int, int], TwoViewInfo], arg1: typing.Dict[int, numpy.ndarray[numpy.float64, _Shape[3, 1]]]) -> typing.Dict[int, numpy.ndarray[numpy.float64, _Shape[3, 1]]]: ...
    @staticmethod
    def __init__(*args, **kwargs) -> typing.Any: ...
    pass
class RotationEstimator():
    pass
class LocalizeViewToReconstructionOptions():
    def __init__(self) -> None: ...
    @property
    def assume_known_orientation(self) -> bool:
        """
        :type: bool
        """
    @assume_known_orientation.setter
    def assume_known_orientation(self, arg0: bool) -> None:
        pass
    @property
    def ba_options(self) -> theia::BundleAdjustmentOptions:
        """
        :type: theia::BundleAdjustmentOptions
        """
    @ba_options.setter
    def ba_options(self, arg0: theia::BundleAdjustmentOptions) -> None:
        pass
    @property
    def bundle_adjust_view(self) -> bool:
        """
        :type: bool
        """
    @bundle_adjust_view.setter
    def bundle_adjust_view(self, arg0: bool) -> None:
        pass
    @property
    def min_num_inliers(self) -> int:
        """
        :type: int
        """
    @min_num_inliers.setter
    def min_num_inliers(self, arg0: int) -> None:
        pass
    @property
    def ransac_params(self) -> theia::RansacParameters:
        """
        :type: theia::RansacParameters
        """
    @ransac_params.setter
    def ransac_params(self, arg0: theia::RansacParameters) -> None:
        pass
    @property
    def reprojection_error_threshold_pixels(self) -> float:
        """
        :type: float
        """
    @reprojection_error_threshold_pixels.setter
    def reprojection_error_threshold_pixels(self, arg0: float) -> None:
        pass
    pass
class LossFunctionType():
    """
    Members:

      TRIVIAL

      HUBER

      SOFTLONE

      CAUCHY

      ARCTAN

      TUKEY
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    ARCTAN: pytheia.pytheia.sfm.LossFunctionType # value = <LossFunctionType.ARCTAN: 4>
    CAUCHY: pytheia.pytheia.sfm.LossFunctionType # value = <LossFunctionType.CAUCHY: 3>
    HUBER: pytheia.pytheia.sfm.LossFunctionType # value = <LossFunctionType.HUBER: 1>
    SOFTLONE: pytheia.pytheia.sfm.LossFunctionType # value = <LossFunctionType.SOFTLONE: 2>
    TRIVIAL: pytheia.pytheia.sfm.LossFunctionType # value = <LossFunctionType.TRIVIAL: 0>
    TUKEY: pytheia.pytheia.sfm.LossFunctionType # value = <LossFunctionType.TUKEY: 5>
    __members__: dict # value = {'TRIVIAL': <LossFunctionType.TRIVIAL: 0>, 'HUBER': <LossFunctionType.HUBER: 1>, 'SOFTLONE': <LossFunctionType.SOFTLONE: 2>, 'CAUCHY': <LossFunctionType.CAUCHY: 3>, 'ARCTAN': <LossFunctionType.ARCTAN: 4>, 'TUKEY': <LossFunctionType.TUKEY: 5>}
    pass
class NonlinearPositionEstimator(PositionEstimator):
    def EstimatePositions(self, arg0: typing.Dict[typing.Tuple[int, int], TwoViewInfo], arg1: typing.Dict[int, numpy.ndarray[numpy.float64, _Shape[3, 1]]]) -> typing.Dict[int, numpy.ndarray[numpy.float64, _Shape[3, 1]]]: ...
    @staticmethod
    def __init__(*args, **kwargs) -> typing.Any: ...
    pass
class NonlinearRotationEstimator(RotationEstimator):
    def EstimateRotations(self, arg0: typing.Dict[typing.Tuple[int, int], TwoViewInfo]) -> typing.Dict[int, numpy.ndarray[numpy.float64, _Shape[3, 1]]]: ...
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, arg0: float) -> None: ...
    pass
class OptimizeIntrinsicsType():
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

      ALL
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    ALL: pytheia.pytheia.sfm.OptimizeIntrinsicsType # value = <OptimizeIntrinsicsType.ALL: 63>
    ASPECT_RATIO: pytheia.pytheia.sfm.OptimizeIntrinsicsType # value = <OptimizeIntrinsicsType.ASPECT_RATIO: 2>
    DISTORTION: pytheia.pytheia.sfm.OptimizeIntrinsicsType # value = <OptimizeIntrinsicsType.DISTORTION: 48>
    FOCAL_LENGTH: pytheia.pytheia.sfm.OptimizeIntrinsicsType # value = <OptimizeIntrinsicsType.FOCAL_LENGTH: 1>
    NONE: pytheia.pytheia.sfm.OptimizeIntrinsicsType # value = <OptimizeIntrinsicsType.NONE: 0>
    PRINCIPAL_POINTS: pytheia.pytheia.sfm.OptimizeIntrinsicsType # value = <OptimizeIntrinsicsType.PRINCIPAL_POINTS: 8>
    RADIAL_DISTORTION: pytheia.pytheia.sfm.OptimizeIntrinsicsType # value = <OptimizeIntrinsicsType.RADIAL_DISTORTION: 16>
    SKEW: pytheia.pytheia.sfm.OptimizeIntrinsicsType # value = <OptimizeIntrinsicsType.SKEW: 4>
    TANGENTIAL_DISTORTION: pytheia.pytheia.sfm.OptimizeIntrinsicsType # value = <OptimizeIntrinsicsType.TANGENTIAL_DISTORTION: 32>
    __members__: dict # value = {'NONE': <OptimizeIntrinsicsType.NONE: 0>, 'FOCAL_LENGTH': <OptimizeIntrinsicsType.FOCAL_LENGTH: 1>, 'ASPECT_RATIO': <OptimizeIntrinsicsType.ASPECT_RATIO: 2>, 'SKEW': <OptimizeIntrinsicsType.SKEW: 4>, 'PRINCIPAL_POINTS': <OptimizeIntrinsicsType.PRINCIPAL_POINTS: 8>, 'RADIAL_DISTORTION': <OptimizeIntrinsicsType.RADIAL_DISTORTION: 16>, 'TANGENTIAL_DISTORTION': <OptimizeIntrinsicsType.TANGENTIAL_DISTORTION: 32>, 'DISTORTION': <OptimizeIntrinsicsType.DISTORTION: 48>, 'ALL': <OptimizeIntrinsicsType.ALL: 63>}
    pass
class PinholeCameraModel(CameraIntrinsicsModel):
    @staticmethod
    def CameraIntrinsicsPriorFromIntrinsics(*args, **kwargs) -> typing.Any: ...
    def GetCalibrationMatrix(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]]) -> None: ...
    @staticmethod
    def GetSubsetFromOptimizeIntrinsicsType(*args, **kwargs) -> typing.Any: ...
    def NumParameters(self) -> int: ...
    def PrintIntrinsics(self) -> None: ...
    @staticmethod
    def SetFromCameraIntrinsicsPriors(*args, **kwargs) -> typing.Any: ...
    def SetRadialDistortion(self, arg0: float, arg1: float) -> None: ...
    @staticmethod
    def Type(*args, **kwargs) -> typing.Any: ...
    def __init__(self) -> None: ...
    @property
    def AspectRatio(self) -> float:
        """
        :type: float
        """
    @AspectRatio.setter
    def AspectRatio(self, arg1: float) -> None:
        pass
    @property
    def RadialDistortion1(self) -> float:
        """
        :type: float
        """
    @property
    def RadialDistortion2(self) -> float:
        """
        :type: float
        """
    @property
    def Skew(self) -> float:
        """
        :type: float
        """
    @Skew.setter
    def Skew(self, arg1: float) -> None:
        pass
    @property
    def kIntrinsicsSize(self) -> int:
        """
        :type: int
        """
    pass
class PinholeRadialTangentialCameraModel(CameraIntrinsicsModel):
    @staticmethod
    def CameraIntrinsicsPriorFromIntrinsics(*args, **kwargs) -> typing.Any: ...
    def GetCalibrationMatrix(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]]) -> None: ...
    @staticmethod
    def GetSubsetFromOptimizeIntrinsicsType(*args, **kwargs) -> typing.Any: ...
    def NumParameters(self) -> int: ...
    def PrintIntrinsics(self) -> None: ...
    @staticmethod
    def SetFromCameraIntrinsicsPriors(*args, **kwargs) -> typing.Any: ...
    def SetRadialDistortion(self, arg0: float, arg1: float, arg2: float) -> None: ...
    def SetTangentialDistortion(self, arg0: float, arg1: float) -> None: ...
    @staticmethod
    def Type(*args, **kwargs) -> typing.Any: ...
    def __init__(self) -> None: ...
    @property
    def AspectRatio(self) -> float:
        """
        :type: float
        """
    @AspectRatio.setter
    def AspectRatio(self, arg1: float) -> None:
        pass
    @property
    def RadialDistortion1(self) -> float:
        """
        :type: float
        """
    @property
    def RadialDistortion2(self) -> float:
        """
        :type: float
        """
    @property
    def RadialDistortion3(self) -> float:
        """
        :type: float
        """
    @property
    def Skew(self) -> float:
        """
        :type: float
        """
    @Skew.setter
    def Skew(self, arg1: float) -> None:
        pass
    @property
    def TangentialDistortion1(self) -> float:
        """
        :type: float
        """
    @property
    def TangentialDistortion2(self) -> float:
        """
        :type: float
        """
    @property
    def kIntrinsicsSize(self) -> int:
        """
        :type: int
        """
    pass
class Plane():
    def __init__(self) -> None: ...
    @property
    def point(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    @point.setter
    def point(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None:
        pass
    @property
    def unit_normal(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    @unit_normal.setter
    def unit_normal(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None:
        pass
    pass
class LeastUnsquaredDeviationPositionEstimator(PositionEstimator):
    def EstimatePositions(self, arg0: typing.Dict[typing.Tuple[int, int], TwoViewInfo], arg1: typing.Dict[int, numpy.ndarray[numpy.float64, _Shape[3, 1]]]) -> typing.Dict[int, numpy.ndarray[numpy.float64, _Shape[3, 1]]]: ...
    @staticmethod
    def __init__(*args, **kwargs) -> typing.Any: ...
    pass
class PriorScalar():
    def __init__(self) -> None: ...
    @property
    def is_set(self) -> bool:
        """
        :type: bool
        """
    @is_set.setter
    def is_set(self, arg0: bool) -> None:
        pass
    @property
    def value(self) -> numpy.ndarray[numpy.float64, _Shape[1, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[1, 1]]
        """
    @value.setter
    def value(self, arg1: numpy.ndarray[numpy.float64, _Shape[1, 1]]) -> None:
        pass
    pass
class PriorVector2d():
    def __init__(self) -> None: ...
    @property
    def is_set(self) -> bool:
        """
        :type: bool
        """
    @is_set.setter
    def is_set(self, arg0: bool) -> None:
        pass
    @property
    def value(self) -> numpy.ndarray[numpy.float64, _Shape[2, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[2, 1]]
        """
    @value.setter
    def value(self, arg1: numpy.ndarray[numpy.float64, _Shape[2, 1]]) -> None:
        pass
    pass
class PriorVector3d():
    def __init__(self) -> None: ...
    @property
    def is_set(self) -> bool:
        """
        :type: bool
        """
    @is_set.setter
    def is_set(self, arg0: bool) -> None:
        pass
    @property
    def value(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    @value.setter
    def value(self, arg1: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None:
        pass
    pass
class PriorVector4d():
    def __init__(self) -> None: ...
    @property
    def is_set(self) -> bool:
        """
        :type: bool
        """
    @is_set.setter
    def is_set(self, arg0: bool) -> None:
        pass
    @property
    def value(self) -> numpy.ndarray[numpy.float64, _Shape[4, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[4, 1]]
        """
    @value.setter
    def value(self, arg1: numpy.ndarray[numpy.float64, _Shape[4, 1]]) -> None:
        pass
    pass
class RadialDistortionFeatureCorrespondence():
    def __init__(self) -> None: ...
    @property
    def feature_left(self) -> numpy.ndarray[numpy.float64, _Shape[2, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[2, 1]]
        """
    @feature_left.setter
    def feature_left(self, arg0: numpy.ndarray[numpy.float64, _Shape[2, 1]]) -> None:
        pass
    @property
    def feature_right(self) -> numpy.ndarray[numpy.float64, _Shape[2, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[2, 1]]
        """
    @feature_right.setter
    def feature_right(self, arg0: numpy.ndarray[numpy.float64, _Shape[2, 1]]) -> None:
        pass
    @property
    def focal_length_estimate_left(self) -> float:
        """
        :type: float
        """
    @focal_length_estimate_left.setter
    def focal_length_estimate_left(self, arg0: float) -> None:
        pass
    @property
    def focal_length_estimate_right(self) -> float:
        """
        :type: float
        """
    @focal_length_estimate_right.setter
    def focal_length_estimate_right(self, arg0: float) -> None:
        pass
    @property
    def max_radial_distortion(self) -> float:
        """
        :type: float
        """
    @max_radial_distortion.setter
    def max_radial_distortion(self, arg0: float) -> None:
        pass
    @property
    def min_radial_distortion(self) -> float:
        """
        :type: float
        """
    @min_radial_distortion.setter
    def min_radial_distortion(self, arg0: float) -> None:
        pass
    @property
    def normalized_feature_left(self) -> numpy.ndarray[numpy.float64, _Shape[2, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[2, 1]]
        """
    @normalized_feature_left.setter
    def normalized_feature_left(self, arg0: numpy.ndarray[numpy.float64, _Shape[2, 1]]) -> None:
        pass
    @property
    def normalized_feature_right(self) -> numpy.ndarray[numpy.float64, _Shape[2, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[2, 1]]
        """
    @normalized_feature_right.setter
    def normalized_feature_right(self, arg0: numpy.ndarray[numpy.float64, _Shape[2, 1]]) -> None:
        pass
    pass
class RansacType():
    """
    Members:

      RANSAC

      PROSAC

      LMED

      EXHAUSTIVE
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    EXHAUSTIVE: pytheia.pytheia.sfm.RansacType # value = <RansacType.EXHAUSTIVE: 3>
    LMED: pytheia.pytheia.sfm.RansacType # value = <RansacType.LMED: 2>
    PROSAC: pytheia.pytheia.sfm.RansacType # value = <RansacType.PROSAC: 1>
    RANSAC: pytheia.pytheia.sfm.RansacType # value = <RansacType.RANSAC: 0>
    __members__: dict # value = {'RANSAC': <RansacType.RANSAC: 0>, 'PROSAC': <RansacType.PROSAC: 1>, 'LMED': <RansacType.LMED: 2>, 'EXHAUSTIVE': <RansacType.EXHAUSTIVE: 3>}
    pass
class Reconstruction():
    def AddObservation(self, arg0: int, arg1: int, arg2: Feature) -> bool: ...
    @typing.overload
    def AddTrack(self) -> int: ...
    @typing.overload
    def AddTrack(self, arg0: typing.List[typing.Tuple[int, Feature]]) -> int: ...
    @typing.overload
    def AddView(self, arg0: str, arg1: float) -> int: ...
    @typing.overload
    def AddView(self, arg0: str, arg1: int, arg2: float) -> int: ...
    def CameraIntrinsicsGroupIdFromViewId(self, arg0: int) -> int: ...
    def CameraIntrinsicsGroupIds(self) -> typing.Set[int]: ...
    def GetViewsInCameraIntrinsicGroup(self, arg0: int) -> typing.Set[int]: ...
    def MutableTrack(self, arg0: int) -> Track: ...
    def MutableView(self, arg0: int) -> View: ...
    def Normalize(self) -> None: ...
    def NumViews(self) -> int: ...
    def RemoveTrack(self, arg0: int) -> bool: ...
    def RemoveView(self, arg0: int) -> bool: ...
    def Track(self, arg0: int) -> Track: ...
    def View(self, arg0: int) -> View: ...
    def ViewIdFromName(self, arg0: str) -> int: ...
    def __init__(self) -> None: ...
    @property
    def NumCameraIntrinsicGroups(self) -> int:
        """
        :type: int
        """
    @property
    def NumTracks(self) -> int:
        """
        :type: int
        """
    @property
    def TrackIds(self) -> typing.List[int]:
        """
        :type: typing.List[int]
        """
    @property
    def ViewIds(self) -> typing.List[int]:
        """
        :type: typing.List[int]
        """
    pass
class ReconstructionBuilder():
    @typing.overload
    def AddImage(self, arg0: str) -> bool: ...
    @typing.overload
    def AddImage(self, arg0: str, arg1: int) -> bool: ...
    @typing.overload
    def AddImageWithCameraIntrinsicsPrior(self, arg0: str, arg1: CameraIntrinsicsPrior) -> bool: ...
    @typing.overload
    def AddImageWithCameraIntrinsicsPrior(self, arg0: str, arg1: CameraIntrinsicsPrior, arg2: int) -> bool: ...
    def AddMaskForFeaturesExtraction(self, arg0: str, arg1: str) -> bool: ...
    def ExtractAndMatchFeatures(self) -> bool: ...
    def __init__(self, arg0: ReconstructionBuilderOptions, arg1: pytheia.pytheia.matching.FeaturesAndMatchesDatabase) -> None: ...
    pass
class ReconstructionBuilderOptions():
    def __init__(self) -> None: ...
    @property
    def descriptor_type(self) -> theia::DescriptorExtractorType:
        """
        :type: theia::DescriptorExtractorType
        """
    @descriptor_type.setter
    def descriptor_type(self, arg0: theia::DescriptorExtractorType) -> None:
        pass
    @property
    def feature_density(self) -> theia::FeatureDensity:
        """
        :type: theia::FeatureDensity
        """
    @feature_density.setter
    def feature_density(self, arg0: theia::FeatureDensity) -> None:
        pass
    @property
    def features_and_matches_database_directory(self) -> str:
        """
        :type: str
        """
    @features_and_matches_database_directory.setter
    def features_and_matches_database_directory(self, arg0: str) -> None:
        pass
    @property
    def matching_options(self) -> pytheia.pytheia.matching.FeatureMatcherOptions:
        """
        :type: pytheia.pytheia.matching.FeatureMatcherOptions
        """
    @matching_options.setter
    def matching_options(self, arg0: pytheia.pytheia.matching.FeatureMatcherOptions) -> None:
        pass
    @property
    def matching_strategy(self) -> pytheia.pytheia.matching.MatchingStrategy:
        """
        :type: pytheia.pytheia.matching.MatchingStrategy
        """
    @matching_strategy.setter
    def matching_strategy(self, arg0: pytheia.pytheia.matching.MatchingStrategy) -> None:
        pass
    @property
    def max_num_features_for_fisher_vector_training(self) -> int:
        """
        :type: int
        """
    @max_num_features_for_fisher_vector_training.setter
    def max_num_features_for_fisher_vector_training(self, arg0: int) -> None:
        pass
    @property
    def max_track_length(self) -> int:
        """
        :type: int
        """
    @max_track_length.setter
    def max_track_length(self, arg0: int) -> None:
        pass
    @property
    def min_num_inlier_matches(self) -> int:
        """
        :type: int
        """
    @min_num_inlier_matches.setter
    def min_num_inlier_matches(self, arg0: int) -> None:
        pass
    @property
    def min_track_length(self) -> int:
        """
        :type: int
        """
    @min_track_length.setter
    def min_track_length(self, arg0: int) -> None:
        pass
    @property
    def num_gmm_clusters_for_fisher_vector(self) -> int:
        """
        :type: int
        """
    @num_gmm_clusters_for_fisher_vector.setter
    def num_gmm_clusters_for_fisher_vector(self, arg0: int) -> None:
        pass
    @property
    def num_nearest_neighbors_for_global_descriptor_matching(self) -> int:
        """
        :type: int
        """
    @num_nearest_neighbors_for_global_descriptor_matching.setter
    def num_nearest_neighbors_for_global_descriptor_matching(self, arg0: int) -> None:
        pass
    @property
    def only_calibrated_views(self) -> bool:
        """
        :type: bool
        """
    @only_calibrated_views.setter
    def only_calibrated_views(self, arg0: bool) -> None:
        pass
    @property
    def reconstruct_largest_connected_component(self) -> bool:
        """
        :type: bool
        """
    @reconstruct_largest_connected_component.setter
    def reconstruct_largest_connected_component(self, arg0: bool) -> None:
        pass
    @property
    def reconstruction_estimator_options(self) -> theia::ReconstructionEstimatorOptions:
        """
        :type: theia::ReconstructionEstimatorOptions
        """
    @reconstruction_estimator_options.setter
    def reconstruction_estimator_options(self, arg0: theia::ReconstructionEstimatorOptions) -> None:
        pass
    @property
    def select_image_pairs_with_global_image_descriptor_matching(self) -> bool:
        """
        :type: bool
        """
    @select_image_pairs_with_global_image_descriptor_matching.setter
    def select_image_pairs_with_global_image_descriptor_matching(self, arg0: bool) -> None:
        pass
    pass
class GlobalReconstructionEstimator(ReconstructionEstimator):
    @staticmethod
    def Estimate(*args, **kwargs) -> typing.Any: ...
    @staticmethod
    def __init__(*args, **kwargs) -> typing.Any: ...
    pass
class ReconstructionEstimatorOptions():
    def __init__(self) -> None: ...
    @property
    def absolute_pose_reprojection_error_threshold(self) -> float:
        """
        :type: float
        """
    @absolute_pose_reprojection_error_threshold.setter
    def absolute_pose_reprojection_error_threshold(self, arg0: float) -> None:
        pass
    @property
    def bundle_adjust_tracks(self) -> bool:
        """
        :type: bool
        """
    @bundle_adjust_tracks.setter
    def bundle_adjust_tracks(self, arg0: bool) -> None:
        pass
    @property
    def bundle_adjustment_loss_function_type(self) -> theia::LossFunctionType:
        """
        :type: theia::LossFunctionType
        """
    @bundle_adjustment_loss_function_type.setter
    def bundle_adjustment_loss_function_type(self, arg0: theia::LossFunctionType) -> None:
        pass
    @property
    def bundle_adjustment_robust_loss_width(self) -> float:
        """
        :type: float
        """
    @bundle_adjustment_robust_loss_width.setter
    def bundle_adjustment_robust_loss_width(self, arg0: float) -> None:
        pass
    @property
    def extract_maximal_rigid_subgraph(self) -> bool:
        """
        :type: bool
        """
    @extract_maximal_rigid_subgraph.setter
    def extract_maximal_rigid_subgraph(self, arg0: bool) -> None:
        pass
    @property
    def filter_relative_translations_with_1dsfm(self) -> bool:
        """
        :type: bool
        """
    @filter_relative_translations_with_1dsfm.setter
    def filter_relative_translations_with_1dsfm(self, arg0: bool) -> None:
        pass
    @property
    def full_bundle_adjustment_growth_percent(self) -> float:
        """
        :type: float
        """
    @full_bundle_adjustment_growth_percent.setter
    def full_bundle_adjustment_growth_percent(self, arg0: float) -> None:
        pass
    @property
    def global_position_estimator_type(self) -> GlobalPositionEstimatorType:
        """
        :type: GlobalPositionEstimatorType
        """
    @global_position_estimator_type.setter
    def global_position_estimator_type(self, arg0: GlobalPositionEstimatorType) -> None:
        pass
    @property
    def global_rotation_estimator_type(self) -> GlobalRotationEstimatorType:
        """
        :type: GlobalRotationEstimatorType
        """
    @global_rotation_estimator_type.setter
    def global_rotation_estimator_type(self, arg0: GlobalRotationEstimatorType) -> None:
        pass
    @property
    def intrinsics_to_optimize(self) -> theia::OptimizeIntrinsicsType:
        """
        :type: theia::OptimizeIntrinsicsType
        """
    @intrinsics_to_optimize.setter
    def intrinsics_to_optimize(self, arg0: theia::OptimizeIntrinsicsType) -> None:
        pass
    @property
    def least_unsquared_deviation_position_estimator_options(self) -> theia::LeastUnsquaredDeviationPositionEstimator::Options:
        """
        :type: theia::LeastUnsquaredDeviationPositionEstimator::Options
        """
    @least_unsquared_deviation_position_estimator_options.setter
    def least_unsquared_deviation_position_estimator_options(self, arg0: theia::LeastUnsquaredDeviationPositionEstimator::Options) -> None:
        pass
    @property
    def linear_triplet_position_estimator_options(self) -> theia::LinearPositionEstimator::Options:
        """
        :type: theia::LinearPositionEstimator::Options
        """
    @linear_triplet_position_estimator_options.setter
    def linear_triplet_position_estimator_options(self, arg0: theia::LinearPositionEstimator::Options) -> None:
        pass
    @property
    def max_reprojection_error_in_pixels(self) -> float:
        """
        :type: float
        """
    @max_reprojection_error_in_pixels.setter
    def max_reprojection_error_in_pixels(self, arg0: float) -> None:
        pass
    @property
    def min_cameras_for_iterative_solver(self) -> int:
        """
        :type: int
        """
    @min_cameras_for_iterative_solver.setter
    def min_cameras_for_iterative_solver(self, arg0: int) -> None:
        pass
    @property
    def min_num_absolute_pose_inliers(self) -> int:
        """
        :type: int
        """
    @min_num_absolute_pose_inliers.setter
    def min_num_absolute_pose_inliers(self, arg0: int) -> None:
        pass
    @property
    def min_num_optimized_tracks_per_view(self) -> int:
        """
        :type: int
        """
    @min_num_optimized_tracks_per_view.setter
    def min_num_optimized_tracks_per_view(self, arg0: int) -> None:
        pass
    @property
    def min_num_two_view_inliers(self) -> int:
        """
        :type: int
        """
    @min_num_two_view_inliers.setter
    def min_num_two_view_inliers(self, arg0: int) -> None:
        pass
    @property
    def min_triangulation_angle_degrees(self) -> float:
        """
        :type: float
        """
    @min_triangulation_angle_degrees.setter
    def min_triangulation_angle_degrees(self, arg0: float) -> None:
        pass
    @property
    def multiple_view_localization_ratio(self) -> float:
        """
        :type: float
        """
    @multiple_view_localization_ratio.setter
    def multiple_view_localization_ratio(self, arg0: float) -> None:
        pass
    @property
    def nonlinear_position_estimator_options(self) -> theia::NonlinearPositionEstimator::Options:
        """
        :type: theia::NonlinearPositionEstimator::Options
        """
    @nonlinear_position_estimator_options.setter
    def nonlinear_position_estimator_options(self, arg0: theia::NonlinearPositionEstimator::Options) -> None:
        pass
    @property
    def num_retriangulation_iterations(self) -> int:
        """
        :type: int
        """
    @num_retriangulation_iterations.setter
    def num_retriangulation_iterations(self, arg0: int) -> None:
        pass
    @property
    def num_threads(self) -> int:
        """
        :type: int
        """
    @num_threads.setter
    def num_threads(self, arg0: int) -> None:
        pass
    @property
    def partial_bundle_adjustment_num_views(self) -> int:
        """
        :type: int
        """
    @partial_bundle_adjustment_num_views.setter
    def partial_bundle_adjustment_num_views(self, arg0: int) -> None:
        pass
    @property
    def ransac_confidence(self) -> float:
        """
        :type: float
        """
    @ransac_confidence.setter
    def ransac_confidence(self, arg0: float) -> None:
        pass
    @property
    def ransac_max_iterations(self) -> int:
        """
        :type: int
        """
    @ransac_max_iterations.setter
    def ransac_max_iterations(self, arg0: int) -> None:
        pass
    @property
    def ransac_min_iterations(self) -> int:
        """
        :type: int
        """
    @ransac_min_iterations.setter
    def ransac_min_iterations(self, arg0: int) -> None:
        pass
    @property
    def ransac_use_mle(self) -> bool:
        """
        :type: bool
        """
    @ransac_use_mle.setter
    def ransac_use_mle(self, arg0: bool) -> None:
        pass
    @property
    def reconstruction_estimator_type(self) -> ReconstructionEstimatorType:
        """
        :type: ReconstructionEstimatorType
        """
    @reconstruction_estimator_type.setter
    def reconstruction_estimator_type(self, arg0: ReconstructionEstimatorType) -> None:
        pass
    @property
    def refine_camera_positions_and_points_after_position_estimation(self) -> bool:
        """
        :type: bool
        """
    @refine_camera_positions_and_points_after_position_estimation.setter
    def refine_camera_positions_and_points_after_position_estimation(self, arg0: bool) -> None:
        pass
    @property
    def refine_relative_translations_after_rotation_estimation(self) -> bool:
        """
        :type: bool
        """
    @refine_relative_translations_after_rotation_estimation.setter
    def refine_relative_translations_after_rotation_estimation(self, arg0: bool) -> None:
        pass
    @property
    def relative_position_estimation_max_sampson_error_pixels(self) -> float:
        """
        :type: float
        """
    @relative_position_estimation_max_sampson_error_pixels.setter
    def relative_position_estimation_max_sampson_error_pixels(self, arg0: float) -> None:
        pass
    @property
    def rotation_estimation_robust_loss_scale(self) -> float:
        """
        :type: float
        """
    @rotation_estimation_robust_loss_scale.setter
    def rotation_estimation_robust_loss_scale(self, arg0: float) -> None:
        pass
    @property
    def rotation_filtering_max_difference_degrees(self) -> float:
        """
        :type: float
        """
    @rotation_filtering_max_difference_degrees.setter
    def rotation_filtering_max_difference_degrees(self, arg0: float) -> None:
        pass
    @property
    def subsample_tracks_for_bundle_adjustment(self) -> bool:
        """
        :type: bool
        """
    @subsample_tracks_for_bundle_adjustment.setter
    def subsample_tracks_for_bundle_adjustment(self, arg0: bool) -> None:
        pass
    @property
    def track_selection_image_grid_cell_size_pixels(self) -> int:
        """
        :type: int
        """
    @track_selection_image_grid_cell_size_pixels.setter
    def track_selection_image_grid_cell_size_pixels(self, arg0: int) -> None:
        pass
    @property
    def track_subset_selection_long_track_length_threshold(self) -> int:
        """
        :type: int
        """
    @track_subset_selection_long_track_length_threshold.setter
    def track_subset_selection_long_track_length_threshold(self, arg0: int) -> None:
        pass
    @property
    def translation_filtering_num_iterations(self) -> int:
        """
        :type: int
        """
    @translation_filtering_num_iterations.setter
    def translation_filtering_num_iterations(self, arg0: int) -> None:
        pass
    @property
    def translation_filtering_projection_tolerance(self) -> float:
        """
        :type: float
        """
    @translation_filtering_projection_tolerance.setter
    def translation_filtering_projection_tolerance(self, arg0: float) -> None:
        pass
    pass
class ReconstructionEstimatorSummary():
    @property
    def bundle_adjustment_time(self) -> float:
        """
        :type: float
        """
    @bundle_adjustment_time.setter
    def bundle_adjustment_time(self, arg0: float) -> None:
        pass
    @property
    def camera_intrinsics_calibration_time(self) -> float:
        """
        :type: float
        """
    @camera_intrinsics_calibration_time.setter
    def camera_intrinsics_calibration_time(self, arg0: float) -> None:
        pass
    @property
    def estimated_tracks(self) -> typing.Set[int]:
        """
        :type: typing.Set[int]
        """
    @estimated_tracks.setter
    def estimated_tracks(self, arg0: typing.Set[int]) -> None:
        pass
    @property
    def estimated_views(self) -> typing.Set[int]:
        """
        :type: typing.Set[int]
        """
    @estimated_views.setter
    def estimated_views(self, arg0: typing.Set[int]) -> None:
        pass
    @property
    def message(self) -> str:
        """
        :type: str
        """
    @message.setter
    def message(self, arg0: str) -> None:
        pass
    @property
    def pose_estimation_time(self) -> float:
        """
        :type: float
        """
    @pose_estimation_time.setter
    def pose_estimation_time(self, arg0: float) -> None:
        pass
    @property
    def success(self) -> bool:
        """
        :type: bool
        """
    @success.setter
    def success(self, arg0: bool) -> None:
        pass
    @property
    def total_time(self) -> float:
        """
        :type: float
        """
    @total_time.setter
    def total_time(self, arg0: float) -> None:
        pass
    @property
    def triangulation_time(self) -> float:
        """
        :type: float
        """
    @triangulation_time.setter
    def triangulation_time(self, arg0: float) -> None:
        pass
    pass
class ReconstructionEstimatorType():
    """
    Members:

      GLOBAL

      INCREMENTAL

      HYBRID
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    GLOBAL: pytheia.pytheia.sfm.ReconstructionEstimatorType # value = <ReconstructionEstimatorType.GLOBAL: 0>
    HYBRID: pytheia.pytheia.sfm.ReconstructionEstimatorType # value = <ReconstructionEstimatorType.HYBRID: 2>
    INCREMENTAL: pytheia.pytheia.sfm.ReconstructionEstimatorType # value = <ReconstructionEstimatorType.INCREMENTAL: 1>
    __members__: dict # value = {'GLOBAL': <ReconstructionEstimatorType.GLOBAL: 0>, 'INCREMENTAL': <ReconstructionEstimatorType.INCREMENTAL: 1>, 'HYBRID': <ReconstructionEstimatorType.HYBRID: 2>}
    pass
class RelativePose():
    def __init__(self) -> None: ...
    @property
    def essential_matrix(self) -> numpy.ndarray[numpy.float64, _Shape[3, 3]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 3]]
        """
    @essential_matrix.setter
    def essential_matrix(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]]) -> None:
        pass
    @property
    def position(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    @position.setter
    def position(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None:
        pass
    @property
    def rotation(self) -> numpy.ndarray[numpy.float64, _Shape[3, 3]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 3]]
        """
    @rotation.setter
    def rotation(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]]) -> None:
        pass
    pass
class RigidTransformation():
    def __init__(self) -> None: ...
    @property
    def rotation(self) -> numpy.ndarray[numpy.float64, _Shape[3, 3]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 3]]
        """
    @rotation.setter
    def rotation(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]]) -> None:
        pass
    @property
    def translation(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    @translation.setter
    def translation(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None:
        pass
    pass
class RobustRotationEstimator(RotationEstimator):
    def AddRelativeRotationConstraint(self, arg0: typing.Tuple[int, int], arg1: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None: ...
    def EstimateRotations(self, arg0: typing.Dict[typing.Tuple[int, int], TwoViewInfo]) -> typing.Dict[int, numpy.ndarray[numpy.float64, _Shape[3, 1]]]: ...
    @staticmethod
    def __init__(*args, **kwargs) -> typing.Any: ...
    pass
class LinearRotationEstimator(RotationEstimator):
    def AddRelativeRotationConstraint(self, arg0: typing.Tuple[int, int], arg1: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None: ...
    def EstimateRotations(self, arg0: typing.Dict[typing.Tuple[int, int], TwoViewInfo]) -> typing.Dict[int, numpy.ndarray[numpy.float64, _Shape[3, 1]]]: ...
    def __init__(self) -> None: ...
    pass
class SimilarityTransformation():
    def __init__(self) -> None: ...
    @property
    def rotation(self) -> numpy.ndarray[numpy.float64, _Shape[3, 3]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 3]]
        """
    @rotation.setter
    def rotation(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]]) -> None:
        pass
    @property
    def scale(self) -> float:
        """
        :type: float
        """
    @scale.setter
    def scale(self, arg0: float) -> None:
        pass
    @property
    def translation(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    @translation.setter
    def translation(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None:
        pass
    pass
class Track():
    def AddView(self, arg0: int) -> None: ...
    def NumViews(self) -> int: ...
    def ReferenceBearingVector(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]: ...
    def ReferenceDescriptor(self) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]: ...
    def ReferenceViewId(self) -> int: ...
    def RemoveView(self, arg0: int) -> bool: ...
    def SetReferenceBearingVector(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None: ...
    def SetReferenceDescriptor(self, arg0: numpy.ndarray[numpy.float32, _Shape[m, 1]]) -> None: ...
    def __init__(self) -> None: ...
    @property
    def Color(self) -> numpy.ndarray[numpy.uint8, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.uint8, _Shape[3, 1]]
        """
    @Color.setter
    def Color(self, arg1: numpy.ndarray[numpy.uint8, _Shape[3, 1]]) -> None:
        pass
    @property
    def InverseDepth(self) -> float:
        """
        :type: float
        """
    @InverseDepth.setter
    def InverseDepth(self, arg1: float) -> None:
        pass
    @property
    def IsEstimated(self) -> bool:
        """
        :type: bool
        """
    @IsEstimated.setter
    def IsEstimated(self, arg1: bool) -> None:
        pass
    @property
    def Point(self) -> numpy.ndarray[numpy.float64, _Shape[4, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[4, 1]]
        """
    @Point.setter
    def Point(self, arg1: numpy.ndarray[numpy.float64, _Shape[4, 1]]) -> None:
        pass
    @property
    def ViewIds(self) -> typing.Set[int]:
        """
        :type: typing.Set[int]
        """
    pass
class TrackBuilder():
    def AddFeatureCorrespondence(self, arg0: int, arg1: Feature, arg2: int, arg3: Feature) -> None: ...
    @staticmethod
    def BuildTracks(*args, **kwargs) -> typing.Any: ...
    def __init__(self, arg0: int, arg1: int) -> None: ...
    pass
class TrackEstimator():
    def EstimateAllTracks(self) -> TrackEstimatorSummary: ...
    def EstimateTracks(self, arg0: typing.Set[int]) -> TrackEstimatorSummary: ...
    pass
class TrackEstimatorOptions():
    @property
    def bundle_adjustment(self) -> bool:
        """
        :type: bool
        """
    @bundle_adjustment.setter
    def bundle_adjustment(self, arg0: bool) -> None:
        pass
    @property
    def max_acceptable_reprojection_error_pixels(self) -> float:
        """
        :type: float
        """
    @max_acceptable_reprojection_error_pixels.setter
    def max_acceptable_reprojection_error_pixels(self, arg0: float) -> None:
        pass
    @property
    def min_triangulation_angle_degrees(self) -> float:
        """
        :type: float
        """
    @min_triangulation_angle_degrees.setter
    def min_triangulation_angle_degrees(self, arg0: float) -> None:
        pass
    @property
    def multithreaded_step_size(self) -> int:
        """
        :type: int
        """
    @multithreaded_step_size.setter
    def multithreaded_step_size(self, arg0: int) -> None:
        pass
    @property
    def num_threads(self) -> int:
        """
        :type: int
        """
    @num_threads.setter
    def num_threads(self, arg0: int) -> None:
        pass
    pass
class TrackEstimatorSummary():
    @property
    def estimated_tracks(self) -> typing.Set[int]:
        """
        :type: typing.Set[int]
        """
    @estimated_tracks.setter
    def estimated_tracks(self, arg0: typing.Set[int]) -> None:
        pass
    @property
    def input_num_estimated_tracks(self) -> int:
        """
        :type: int
        """
    @input_num_estimated_tracks.setter
    def input_num_estimated_tracks(self, arg0: int) -> None:
        pass
    @property
    def num_triangulation_attempts(self) -> int:
        """
        :type: int
        """
    @num_triangulation_attempts.setter
    def num_triangulation_attempts(self, arg0: int) -> None:
        pass
    pass
class TwoViewBundleAdjustmentOptions():
    def __init__(self) -> None: ...
    @property
    def ba_options(self) -> BundleAdjustmentOptions:
        """
        :type: BundleAdjustmentOptions
        """
    @ba_options.setter
    def ba_options(self, arg0: BundleAdjustmentOptions) -> None:
        pass
    @property
    def constant_camera1_intrinsics(self) -> bool:
        """
        :type: bool
        """
    @constant_camera1_intrinsics.setter
    def constant_camera1_intrinsics(self, arg0: bool) -> None:
        pass
    @property
    def constant_camera2_intrinsics(self) -> bool:
        """
        :type: bool
        """
    @constant_camera2_intrinsics.setter
    def constant_camera2_intrinsics(self, arg0: bool) -> None:
        pass
    pass
class TwoViewInfo():
    def __init__(self) -> None: ...
    @property
    def focal_length_1(self) -> float:
        """
        :type: float
        """
    @focal_length_1.setter
    def focal_length_1(self, arg0: float) -> None:
        pass
    @property
    def focal_length_2(self) -> float:
        """
        :type: float
        """
    @focal_length_2.setter
    def focal_length_2(self, arg0: float) -> None:
        pass
    @property
    def num_homography_inliers(self) -> int:
        """
        :type: int
        """
    @num_homography_inliers.setter
    def num_homography_inliers(self, arg0: int) -> None:
        pass
    @property
    def num_verified_matches(self) -> int:
        """
        :type: int
        """
    @num_verified_matches.setter
    def num_verified_matches(self, arg0: int) -> None:
        pass
    @property
    def position_2(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    @position_2.setter
    def position_2(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None:
        pass
    @property
    def rotation_2(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    @rotation_2.setter
    def rotation_2(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None:
        pass
    @property
    def visibility_score(self) -> int:
        """
        :type: int
        """
    @visibility_score.setter
    def visibility_score(self, arg0: int) -> None:
        pass
    pass
class TwoViewMatchGeometricVerification():
    @staticmethod
    def VerifyMatches(*args, **kwargs) -> typing.Any: ...
    def __init__(self, arg0: TwoViewMatchGeometricVerificationOptions, arg1: CameraIntrinsicsPrior, arg2: CameraIntrinsicsPrior, arg3: pytheia.pytheia.matching.KeypointsAndDescriptors, arg4: pytheia.pytheia.matching.KeypointsAndDescriptors, arg5: typing.List[pytheia.pytheia.matching.IndexedFeatureMatch]) -> None: ...
    pass
class TwoViewMatchGeometricVerificationOptions():
    def __init__(self) -> None: ...
    @property
    def bundle_adjustment(self) -> bool:
        """
        :type: bool
        """
    @bundle_adjustment.setter
    def bundle_adjustment(self, arg0: bool) -> None:
        pass
    @property
    def estimate_twoview_info_options(self) -> EstimateTwoViewInfoOptions:
        """
        :type: EstimateTwoViewInfoOptions
        """
    @estimate_twoview_info_options.setter
    def estimate_twoview_info_options(self, arg0: EstimateTwoViewInfoOptions) -> None:
        pass
    @property
    def final_max_reprojection_error(self) -> float:
        """
        :type: float
        """
    @final_max_reprojection_error.setter
    def final_max_reprojection_error(self, arg0: float) -> None:
        pass
    @property
    def guided_matching(self) -> bool:
        """
        :type: bool
        """
    @guided_matching.setter
    def guided_matching(self, arg0: bool) -> None:
        pass
    @property
    def guided_matching_lowes_ratio(self) -> float:
        """
        :type: float
        """
    @guided_matching_lowes_ratio.setter
    def guided_matching_lowes_ratio(self, arg0: float) -> None:
        pass
    @property
    def guided_matching_max_distance_pixels(self) -> float:
        """
        :type: float
        """
    @guided_matching_max_distance_pixels.setter
    def guided_matching_max_distance_pixels(self, arg0: float) -> None:
        pass
    @property
    def min_num_inlier_matches(self) -> int:
        """
        :type: int
        """
    @min_num_inlier_matches.setter
    def min_num_inlier_matches(self, arg0: int) -> None:
        pass
    @property
    def min_triangulation_angle_degrees(self) -> float:
        """
        :type: float
        """
    @min_triangulation_angle_degrees.setter
    def min_triangulation_angle_degrees(self, arg0: float) -> None:
        pass
    @property
    def triangulation_max_reprojection_error(self) -> float:
        """
        :type: float
        """
    @triangulation_max_reprojection_error.setter
    def triangulation_max_reprojection_error(self, arg0: float) -> None:
        pass
    pass
class UncalibratedAbsolutePose():
    def __init__(self) -> None: ...
    @property
    def position(self) -> float:
        """
        :type: float
        """
    @position.setter
    def position(self, arg0: float) -> None:
        pass
    @property
    def rotation(self) -> numpy.ndarray[numpy.float64, _Shape[3, 3]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 3]]
        """
    @rotation.setter
    def rotation(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]]) -> None:
        pass
    pass
class UncalibratedRelativePose():
    def __init__(self) -> None: ...
    @property
    def focal_length1(self) -> float:
        """
        :type: float
        """
    @focal_length1.setter
    def focal_length1(self, arg0: float) -> None:
        pass
    @property
    def focal_length2(self) -> float:
        """
        :type: float
        """
    @focal_length2.setter
    def focal_length2(self, arg0: float) -> None:
        pass
    @property
    def fundamental_matrix(self) -> numpy.ndarray[numpy.float64, _Shape[3, 3]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 3]]
        """
    @fundamental_matrix.setter
    def fundamental_matrix(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]]) -> None:
        pass
    @property
    def position(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    @position.setter
    def position(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None:
        pass
    @property
    def rotation(self) -> numpy.ndarray[numpy.float64, _Shape[3, 3]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 3]]
        """
    @rotation.setter
    def rotation(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]]) -> None:
        pass
    pass
class View():
    def AddFeature(self, arg0: int, arg1: Feature) -> None: ...
    def Camera(self) -> Camera: 
        """
        Camera class object
        """
    def CameraIntrinsicsPrior(self) -> CameraIntrinsicsPrior: ...
    def GetFeature(self, arg0: int) -> Feature: ...
    def GetTrack(self, arg0: Feature) -> int: ...
    def MutableCamera(self) -> Camera: ...
    def MutableCameraIntrinsicsPrior(self) -> CameraIntrinsicsPrior: ...
    def NumFeatures(self) -> int: ...
    def RemoveFeature(self, arg0: int) -> bool: ...
    def SetCameraIntrinsicsPrior(self, arg0: CameraIntrinsicsPrior) -> None: ...
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, arg0: str) -> None: ...
    @property
    def IsEstimated(self) -> bool:
        """
        :type: bool
        """
    @IsEstimated.setter
    def IsEstimated(self, arg1: bool) -> None:
        pass
    @property
    def Name(self) -> str:
        """
        :type: str
        """
    @property
    def TrackIds(self) -> typing.List[int]:
        """
        :type: typing.List[int]
        """
    pass
class ViewGraph():
    def AddEdge(self, arg0: int, arg1: int, arg2: TwoViewInfo) -> None: ...
    def GetAllEdges(self) -> typing.Dict[typing.Tuple[int, int], TwoViewInfo]: ...
    def GetEdge(self, arg0: int, arg1: int) -> TwoViewInfo: ...
    def GetNeighborIdsForView(self, arg0: int) -> typing.Set[int]: ...
    def HasEdge(self, arg0: int, arg1: int) -> bool: ...
    def HasView(self, arg0: int) -> bool: ...
    def ReadFromDisk(self, arg0: str) -> bool: ...
    def RemoveEdge(self, arg0: int, arg1: int) -> bool: ...
    def RemoveView(self, arg0: int) -> bool: ...
    def WriteToDisk(self, arg0: str) -> bool: ...
    def __init__(self) -> None: ...
    @property
    def NumEdges(self) -> int:
        """
        :type: int
        """
    @property
    def NumViews(self) -> int:
        """
        :type: int
        """
    pass
class VisibilityPyramid():
    def AddPoint(self, arg0: numpy.ndarray[numpy.float64, _Shape[2, 1]]) -> None: ...
    def ComputeScore(self) -> int: ...
    def __init__(self, arg0: int, arg1: int, arg2: int) -> None: ...
    pass
def AlignPointCloudsUmeyama(arg0: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], arg1: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]) -> typing.Tuple[numpy.ndarray[numpy.float64, _Shape[3, 3]], numpy.ndarray[numpy.float64, _Shape[3, 1]], float]:
    pass
def AlignPointCloudsUmeyamaWithWeights(arg0: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], arg1: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], arg2: typing.List[float]) -> typing.Tuple[numpy.ndarray[numpy.float64, _Shape[3, 3]], numpy.ndarray[numpy.float64, _Shape[3, 1]], float]:
    pass
def AlignReconstructions(*args, **kwargs) -> typing.Any:
    pass
def AlignReconstructionsRobust(*args, **kwargs) -> typing.Any:
    pass
def AlignRotations(arg0: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]) -> typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]:
    pass
def BundleAdjustPartialReconstruction(arg0: BundleAdjustmentOptions, arg1: typing.Set[int], arg2: typing.Set[int]) -> typing.Tuple[BundleAdjustmentSummary, Reconstruction]:
    pass
def BundleAdjustReconstruction(arg0: BundleAdjustmentOptions) -> typing.Tuple[BundleAdjustmentSummary, Reconstruction]:
    pass
def BundleAdjustTrack(arg0: Reconstruction, arg1: BundleAdjustmentOptions, arg2: int) -> typing.Tuple[BundleAdjustmentSummary, Reconstruction]:
    pass
def BundleAdjustTrackWithCov(arg0: Reconstruction, arg1: BundleAdjustmentOptions, arg2: int) -> typing.Tuple[BundleAdjustmentSummary, Reconstruction, numpy.ndarray[numpy.float64, _Shape[3, 3]], float]:
    pass
def BundleAdjustTracksWithCov(arg0: Reconstruction, arg1: BundleAdjustmentOptions, arg2: typing.List[int]) -> typing.Tuple[BundleAdjustmentSummary, Reconstruction, typing.Dict[int, numpy.ndarray[numpy.float64, _Shape[3, 3]]], float]:
    pass
def BundleAdjustTwoViews(arg0: TwoViewBundleAdjustmentOptions, arg1: typing.List[pytheia.pytheia.matching.FeatureCorrespondence]) -> typing.Tuple[BundleAdjustmentSummary, Camera, Camera, typing.List[numpy.ndarray[numpy.float64, _Shape[4, 1]]]]:
    pass
def BundleAdjustTwoViewsAngular(arg0: BundleAdjustmentOptions, arg1: typing.List[pytheia.pytheia.matching.FeatureCorrespondence]) -> typing.Tuple[BundleAdjustmentSummary, TwoViewInfo]:
    pass
def BundleAdjustView(arg0: Reconstruction, arg1: BundleAdjustmentOptions, arg2: int) -> typing.Tuple[BundleAdjustmentSummary, Reconstruction]:
    pass
def BundleAdjustViewWithCov(arg0: Reconstruction, arg1: BundleAdjustmentOptions, arg2: int) -> typing.Tuple[BundleAdjustmentSummary, Reconstruction, numpy.ndarray[numpy.float64, _Shape[6, 6]], float]:
    pass
def BundleAdjustViewsWithCov(arg0: Reconstruction, arg1: BundleAdjustmentOptions, arg2: typing.List[int]) -> typing.Tuple[BundleAdjustmentSummary, Reconstruction, typing.Dict[int, numpy.ndarray[numpy.float64, _Shape[6, 6]]], float]:
    pass
def CalibrationMatrixToIntrinsics(arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]]) -> typing.Tuple[float, float, float, float, float]:
    pass
def ColorizeReconstruction(*args, **kwargs) -> typing.Any:
    pass
def ComposeFundamentalMatrix(arg0: float, arg1: float, arg2: numpy.ndarray[numpy.float64, _Shape[3, 3]], arg3: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> numpy.ndarray[numpy.float64, _Shape[3, 3]]:
    pass
def ComposeProjectionMatrix(arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]], arg1: numpy.ndarray[numpy.float64, _Shape[3, 1]], arg2: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> typing.Tuple[bool, numpy.ndarray[numpy.float64, _Shape[3, 4]]]:
    pass
def ComputeTripletBaselineRatios(*args, **kwargs) -> typing.Any:
    pass
def DecomposeEssentialMatrix(arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]]) -> typing.Tuple[numpy.ndarray[numpy.float64, _Shape[3, 3]], numpy.ndarray[numpy.float64, _Shape[3, 3]], numpy.ndarray[numpy.float64, _Shape[3, 1]]]:
    pass
def DecomposeProjectionMatrix(arg0: numpy.ndarray[numpy.float64, _Shape[3, 4]]) -> typing.Tuple[bool, numpy.ndarray[numpy.float64, _Shape[3, 3]], numpy.ndarray[numpy.float64, _Shape[3, 1]], numpy.ndarray[numpy.float64, _Shape[3, 1]]]:
    pass
def DlsPnp(arg0: typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]], arg1: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]) -> typing.Tuple[typing.List[numpy.ndarray[numpy.float64, _Shape[4, 1]]], typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]]:
    pass
def EssentialMatrixFromFundamentalMatrix(arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]], arg1: float, arg2: float) -> numpy.ndarray[numpy.float64, _Shape[3, 3]]:
    pass
def EssentialMatrixFromTwoProjectionMatrices(arg0: numpy.ndarray[numpy.float64, _Shape[3, 4]], arg1: numpy.ndarray[numpy.float64, _Shape[3, 4]]) -> numpy.ndarray[numpy.float64, _Shape[3, 3]]:
    pass
def EstimateAbsolutePoseWithKnownOrientation(*args, **kwargs) -> typing.Any:
    pass
def EstimateCalibratedAbsolutePose(*args, **kwargs) -> typing.Any:
    pass
def EstimateDominantPlaneFromPoints(*args, **kwargs) -> typing.Any:
    pass
def EstimateEssentialMatrix(*args, **kwargs) -> typing.Any:
    pass
def EstimateFundamentalMatrix(*args, **kwargs) -> typing.Any:
    pass
def EstimateHomography(*args, **kwargs) -> typing.Any:
    pass
def EstimateRadialHomographyMatrix(*args, **kwargs) -> typing.Any:
    pass
def EstimateRelativePose(*args, **kwargs) -> typing.Any:
    pass
def EstimateRelativePoseWithKnownOrientation(*args, **kwargs) -> typing.Any:
    pass
def EstimateRigidTransformation2D3D(*args, **kwargs) -> typing.Any:
    pass
def EstimateRigidTransformation2D3DNormalized(*args, **kwargs) -> typing.Any:
    pass
def EstimateTriangulation(*args, **kwargs) -> typing.Any:
    pass
def EstimateTwoViewInfo(arg0: EstimateTwoViewInfoOptions, arg1: CameraIntrinsicsPrior, arg2: CameraIntrinsicsPrior, arg3: typing.List[pytheia.pytheia.matching.FeatureCorrespondence]) -> typing.Tuple[bool, theia::TwoViewInfo, typing.List[int]]:
    pass
def EstimateUncalibratedAbsolutePose(*args, **kwargs) -> typing.Any:
    pass
def EstimateUncalibratedRelativePose(*args, **kwargs) -> typing.Any:
    pass
def ExtractMaximallyParallelRigidSubgraph(*args, **kwargs) -> typing.Any:
    pass
def FilterViewGraphCyclesByRotation(*args, **kwargs) -> typing.Any:
    pass
def FilterViewPairsFromOrientation(*args, **kwargs) -> typing.Any:
    pass
def FilterViewPairsFromRelativeTranslation(*args, **kwargs) -> typing.Any:
    pass
def FindCommonTracksInViews(*args, **kwargs) -> typing.Any:
    pass
def FindCommonViewsByName(*args, **kwargs) -> typing.Any:
    pass
def FivePointFocalLengthRadialDistortion(arg0: typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]], arg1: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], arg2: int) -> typing.Tuple[bool, typing.List[numpy.ndarray[numpy.float64, _Shape[3, 4]]], typing.List[typing.List[float]]]:
    pass
def FivePointRelativePose(arg0: typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]], arg1: typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]]) -> typing.Tuple[bool, typing.List[numpy.ndarray[numpy.float64, _Shape[3, 3]]]]:
    pass
def FocalLengthsFromFundamentalMatrix(arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]]) -> typing.Tuple[bool, float, float]:
    pass
def FourPointHomography(arg0: typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]], arg1: typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]]) -> typing.Tuple[bool, numpy.ndarray[numpy.float64, _Shape[3, 3]]]:
    pass
def FourPointPoseAndFocalLength(arg0: typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]], arg1: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]) -> typing.Tuple[int, typing.List[numpy.ndarray[numpy.float64, _Shape[3, 4]]]]:
    pass
def FourPointRelativePosePartialRotation(arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]], arg1: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], arg2: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], arg3: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], arg4: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]) -> typing.Tuple[typing.List[numpy.ndarray[numpy.float64, _Shape[4, 1]]], typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]]:
    pass
def FourPointsPoseFocalLengthRadialDistortion(arg0: typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]], arg1: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]) -> typing.Tuple[bool, typing.List[numpy.ndarray[numpy.float64, _Shape[3, 3]]], typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], typing.List[float], typing.List[float]]:
    pass
def FundamentalMatrixFromProjectionMatrices(arg0: numpy.ndarray[numpy.float64, _Shape[3, 4]], arg1: numpy.ndarray[numpy.float64, _Shape[3, 4]]) -> numpy.ndarray[numpy.float64, _Shape[3, 3]]:
    pass
def GdlsSimilarityTransform(arg0: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], arg1: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], arg2: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]) -> typing.Tuple[typing.List[numpy.ndarray[numpy.float64, _Shape[4, 1]]], typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], typing.List[float]]:
    pass
def GetBestPoseFromEssentialMatrix(arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]], arg1: typing.List[pytheia.pytheia.matching.FeatureCorrespondence]) -> typing.Tuple[int, numpy.ndarray[numpy.float64, _Shape[3, 3]], numpy.ndarray[numpy.float64, _Shape[3, 1]]]:
    pass
def IntrinsicsToCalibrationMatrix(arg0: float, arg1: float, arg2: float, arg3: float, arg4: float) -> numpy.ndarray[numpy.float64, _Shape[3, 3]]:
    pass
def IsTriangulatedPointInFrontOfCameras(arg0: pytheia.pytheia.matching.FeatureCorrespondence, arg1: numpy.ndarray[numpy.float64, _Shape[3, 3]], arg2: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> bool:
    pass
def LocalizeViewToReconstruction(arg0: int, arg1: LocalizeViewToReconstructionOptions) -> typing.Tuple[bool, theia::Reconstruction, theia::RansacSummary]:
    pass
def NormalizedEightPointFundamentalMatrix(arg0: typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]], arg1: typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]]) -> typing.Tuple[bool, numpy.ndarray[numpy.float64, _Shape[3, 3]]]:
    pass
def OptimizeRelativePositionWithKnownRotation(arg0: typing.List[pytheia.pytheia.matching.FeatureCorrespondence], arg1: numpy.ndarray[numpy.float64, _Shape[3, 1]], arg2: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> typing.Tuple[bool, numpy.ndarray[numpy.float64, _Shape[3, 1]]]:
    pass
def PoseFromThreePoints(arg0: typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]], arg1: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]) -> typing.Tuple[bool, typing.List[numpy.ndarray[numpy.float64, _Shape[3, 3]]], typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]]:
    pass
def PositionFromTwoRays(arg0: numpy.ndarray[numpy.float64, _Shape[2, 1]], arg1: numpy.ndarray[numpy.float64, _Shape[3, 1]], arg2: numpy.ndarray[numpy.float64, _Shape[2, 1]], arg3: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> typing.Tuple[bool, numpy.ndarray[numpy.float64, _Shape[3, 1]]]:
    pass
def ProjectionMatricesFromFundamentalMatrix(arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]]) -> typing.Tuple[numpy.ndarray[numpy.float64, _Shape[3, 4]], numpy.ndarray[numpy.float64, _Shape[3, 4]]]:
    pass
def RelativePoseFromTwoPointsWithKnownRotation(arg0: typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]], arg1: typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]]) -> typing.Tuple[bool, numpy.ndarray[numpy.float64, _Shape[3, 1]]]:
    pass
def SelectGoodTracksForBundleAdjustment(*args, **kwargs) -> typing.Any:
    pass
def SelectGoodTracksForBundleAdjustmentAll(*args, **kwargs) -> typing.Any:
    pass
def SetCameraIntrinsicsFromPriors(*args, **kwargs) -> typing.Any:
    pass
def SetOutlierTracksToUnestimated(arg0: typing.Set[int], arg1: float, arg2: float) -> typing.Tuple[int, theia::Reconstruction]:
    pass
def SetOutlierTracksToUnestimatedAll(arg0: float, arg1: float) -> typing.Tuple[int, theia::Reconstruction]:
    pass
def SevenPointFundamentalMatrix(arg0: typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]], arg1: typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]]) -> typing.Tuple[bool, typing.List[numpy.ndarray[numpy.float64, _Shape[3, 3]]]]:
    pass
def SharedFocalLengthsFromFundamentalMatrix(arg0: numpy.ndarray[numpy.float64, _Shape[3, 3]]) -> typing.Tuple[bool, float]:
    pass
def SimTransformPartialRotation(arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]], arg1: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], arg2: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], arg3: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], arg4: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]) -> typing.Tuple[typing.List[numpy.ndarray[numpy.float64, _Shape[4, 1]]], typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], typing.List[float]]:
    pass
def SufficientTriangulationAngle(arg0: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], arg1: float) -> bool:
    pass
def SwapCameras(arg0: TwoViewInfo) -> None:
    pass
def ThreePointRelativePosePartialRotation(arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]], arg1: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], arg2: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]) -> typing.Tuple[typing.List[numpy.ndarray[numpy.float64, _Shape[4, 1]]], typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]]:
    pass
def TransformReconstruction(*args, **kwargs) -> typing.Any:
    pass
def Triangulate(arg0: numpy.ndarray[numpy.float64, _Shape[3, 4]], arg1: numpy.ndarray[numpy.float64, _Shape[3, 4]], arg2: numpy.ndarray[numpy.float64, _Shape[2, 1]], arg3: numpy.ndarray[numpy.float64, _Shape[2, 1]]) -> typing.Tuple[bool, numpy.ndarray[numpy.float64, _Shape[4, 1]]]:
    pass
def TriangulateDLT(arg0: numpy.ndarray[numpy.float64, _Shape[3, 4]], arg1: numpy.ndarray[numpy.float64, _Shape[3, 4]], arg2: numpy.ndarray[numpy.float64, _Shape[2, 1]], arg3: numpy.ndarray[numpy.float64, _Shape[2, 1]]) -> typing.Tuple[bool, numpy.ndarray[numpy.float64, _Shape[4, 1]]]:
    pass
def TriangulateMidpoint(arg0: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], arg1: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]) -> typing.Tuple[bool, numpy.ndarray[numpy.float64, _Shape[4, 1]]]:
    pass
def TriangulateNView(arg0: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 4]]], arg1: typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]]) -> typing.Tuple[bool, numpy.ndarray[numpy.float64, _Shape[4, 1]]]:
    pass
def TriangulateNViewSVD(arg0: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 4]]], arg1: typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]]) -> typing.Tuple[bool, numpy.ndarray[numpy.float64, _Shape[4, 1]]]:
    pass
def TwoPointPosePartialRotation(arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]], arg1: numpy.ndarray[numpy.float64, _Shape[3, 1]], arg2: numpy.ndarray[numpy.float64, _Shape[3, 1]], arg3: numpy.ndarray[numpy.float64, _Shape[3, 1]], arg4: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> typing.Tuple[int, typing.List[numpy.ndarray[numpy.float64, _Shape[4, 1]]], typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]]:
    pass
ALL: pytheia.pytheia.sfm.OptimizeIntrinsicsType # value = <OptimizeIntrinsicsType.ALL: 63>
ARCTAN: pytheia.pytheia.sfm.LossFunctionType # value = <LossFunctionType.ARCTAN: 4>
ASPECT_RATIO: pytheia.pytheia.sfm.OptimizeIntrinsicsType # value = <OptimizeIntrinsicsType.ASPECT_RATIO: 2>
CAUCHY: pytheia.pytheia.sfm.LossFunctionType # value = <LossFunctionType.CAUCHY: 3>
DISTORTION: pytheia.pytheia.sfm.OptimizeIntrinsicsType # value = <OptimizeIntrinsicsType.DISTORTION: 48>
DIVISION_UNDISTORTION: pytheia.pytheia.sfm.CameraIntrinsicsModelType # value = <CameraIntrinsicsModelType.DIVISION_UNDISTORTION: 4>
DOUBLE_SPHERE: pytheia.pytheia.sfm.CameraIntrinsicsModelType # value = <CameraIntrinsicsModelType.DOUBLE_SPHERE: 5>
EXHAUSTIVE: pytheia.pytheia.sfm.RansacType # value = <RansacType.EXHAUSTIVE: 3>
EXTENDED_UNIFIED: pytheia.pytheia.sfm.CameraIntrinsicsModelType # value = <CameraIntrinsicsModelType.EXTENDED_UNIFIED: 6>
FISHEYE: pytheia.pytheia.sfm.CameraIntrinsicsModelType # value = <CameraIntrinsicsModelType.FISHEYE: 2>
FOCAL_LENGTH: pytheia.pytheia.sfm.OptimizeIntrinsicsType # value = <OptimizeIntrinsicsType.FOCAL_LENGTH: 1>
FOV: pytheia.pytheia.sfm.CameraIntrinsicsModelType # value = <CameraIntrinsicsModelType.FOV: 3>
GLOBAL: pytheia.pytheia.sfm.ReconstructionEstimatorType # value = <ReconstructionEstimatorType.GLOBAL: 0>
HUBER: pytheia.pytheia.sfm.LossFunctionType # value = <LossFunctionType.HUBER: 1>
HYBRID: pytheia.pytheia.sfm.ReconstructionEstimatorType # value = <ReconstructionEstimatorType.HYBRID: 2>
INCREMENTAL: pytheia.pytheia.sfm.ReconstructionEstimatorType # value = <ReconstructionEstimatorType.INCREMENTAL: 1>
INVALID: pytheia.pytheia.sfm.CameraIntrinsicsModelType # value = <CameraIntrinsicsModelType.INVALID: -1>
LEAST_UNSQUARED_DEVIATION: pytheia.pytheia.sfm.GlobalPositionEstimatorType # value = <GlobalPositionEstimatorType.LEAST_UNSQUARED_DEVIATION: 2>
LINEAR: pytheia.pytheia.sfm.GlobalRotationEstimatorType # value = <GlobalRotationEstimatorType.LINEAR: 2>
LINEAR_TRIPLET: pytheia.pytheia.sfm.GlobalPositionEstimatorType # value = <GlobalPositionEstimatorType.LINEAR_TRIPLET: 1>
LMED: pytheia.pytheia.sfm.RansacType # value = <RansacType.LMED: 2>
NONE: pytheia.pytheia.sfm.OptimizeIntrinsicsType # value = <OptimizeIntrinsicsType.NONE: 0>
NONLINEAR: pytheia.pytheia.sfm.GlobalRotationEstimatorType # value = <GlobalRotationEstimatorType.NONLINEAR: 1>
PINHOLE: pytheia.pytheia.sfm.CameraIntrinsicsModelType # value = <CameraIntrinsicsModelType.PINHOLE: 0>
PINHOLE_RADIAL_TANGENTIAL: pytheia.pytheia.sfm.CameraIntrinsicsModelType # value = <CameraIntrinsicsModelType.PINHOLE_RADIAL_TANGENTIAL: 1>
PRINCIPAL_POINTS: pytheia.pytheia.sfm.OptimizeIntrinsicsType # value = <OptimizeIntrinsicsType.PRINCIPAL_POINTS: 8>
PROSAC: pytheia.pytheia.sfm.RansacType # value = <RansacType.PROSAC: 1>
RADIAL_DISTORTION: pytheia.pytheia.sfm.OptimizeIntrinsicsType # value = <OptimizeIntrinsicsType.RADIAL_DISTORTION: 16>
RANSAC: pytheia.pytheia.sfm.RansacType # value = <RansacType.RANSAC: 0>
ROBUST_L1L2: pytheia.pytheia.sfm.GlobalRotationEstimatorType # value = <GlobalRotationEstimatorType.ROBUST_L1L2: 0>
SKEW: pytheia.pytheia.sfm.OptimizeIntrinsicsType # value = <OptimizeIntrinsicsType.SKEW: 4>
SOFTLONE: pytheia.pytheia.sfm.LossFunctionType # value = <LossFunctionType.SOFTLONE: 2>
TANGENTIAL_DISTORTION: pytheia.pytheia.sfm.OptimizeIntrinsicsType # value = <OptimizeIntrinsicsType.TANGENTIAL_DISTORTION: 32>
TRIVIAL: pytheia.pytheia.sfm.LossFunctionType # value = <LossFunctionType.TRIVIAL: 0>
TUKEY: pytheia.pytheia.sfm.LossFunctionType # value = <LossFunctionType.TUKEY: 5>
kInvalidTrackId = 4294967295
kInvalidViewId = 4294967295
