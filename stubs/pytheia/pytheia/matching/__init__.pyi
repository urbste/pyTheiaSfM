import pytheia.pytheia.matching
import typing
import numpy
_Shape = typing.Tuple[int, ...]

__all__ = [
    "BruteForceFeatureMatcher",
    "CascadeHashingFeatureMatcher",
    "FeatureCorrespondence",
    "FeatureMatcher",
    "FeatureMatcherOptions",
    "FeaturesAndMatchesDatabase",
    "FisherVectorExtractor",
    "FisherVectorExtractorOptions",
    "GLOBAL",
    "GlobalDescriptorExtractor",
    "INCREMENTAL",
    "ImagePairMatch",
    "InMemoryFeaturesAndMatchesDatabase",
    "IndexedFeatureMatch",
    "KeypointsAndDescriptors",
    "MatchingStrategy"
]


class FeatureMatcher():
    def AddImage(self, arg0: str) -> None: ...
    def AddImages(self, arg0: typing.List[str]) -> None: ...
    def MatchImages(self) -> None: ...
    def SetImagePairsToMatch(self, arg0: typing.List[typing.Tuple[str, str]]) -> None: ...
    pass
class CascadeHashingFeatureMatcher(FeatureMatcher):
    def AddImage(self, arg0: str) -> None: ...
    def AddImages(self, arg0: typing.List[str]) -> None: ...
    def __init__(self, arg0: FeatureMatcherOptions, arg1: FeaturesAndMatchesDatabase) -> None: ...
    pass
class FeatureCorrespondence():
    @staticmethod
    @typing.overload
    def __init__(*args, **kwargs) -> typing.Any: ...
    @typing.overload
    def __init__(self) -> None: ...
    @property
    def feature1(self) -> theia::Feature:
        """
        :type: theia::Feature
        """
    @feature1.setter
    def feature1(self, arg0: theia::Feature) -> None:
        pass
    @property
    def feature2(self) -> theia::Feature:
        """
        :type: theia::Feature
        """
    @feature2.setter
    def feature2(self, arg0: theia::Feature) -> None:
        pass
    pass
class BruteForceFeatureMatcher(FeatureMatcher):
    def __init__(self, arg0: FeatureMatcherOptions, arg1: FeaturesAndMatchesDatabase) -> None: ...
    pass
class FeatureMatcherOptions():
    def __init__(self) -> None: ...
    @property
    def geometric_verification_options(self) -> theia::TwoViewMatchGeometricVerification::Options:
        """
        :type: theia::TwoViewMatchGeometricVerification::Options
        """
    @geometric_verification_options.setter
    def geometric_verification_options(self, arg0: theia::TwoViewMatchGeometricVerification::Options) -> None:
        pass
    @property
    def keep_only_symmetric_matches(self) -> bool:
        """
        :type: bool
        """
    @keep_only_symmetric_matches.setter
    def keep_only_symmetric_matches(self, arg0: bool) -> None:
        pass
    @property
    def lowes_ratio(self) -> float:
        """
        :type: float
        """
    @lowes_ratio.setter
    def lowes_ratio(self, arg0: float) -> None:
        pass
    @property
    def min_num_feature_matches(self) -> int:
        """
        :type: int
        """
    @min_num_feature_matches.setter
    def min_num_feature_matches(self, arg0: int) -> None:
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
    def perform_geometric_verification(self) -> bool:
        """
        :type: bool
        """
    @perform_geometric_verification.setter
    def perform_geometric_verification(self, arg0: bool) -> None:
        pass
    @property
    def use_lowes_ratio(self) -> bool:
        """
        :type: bool
        """
    @use_lowes_ratio.setter
    def use_lowes_ratio(self, arg0: bool) -> None:
        pass
    pass
class FeaturesAndMatchesDatabase():
    pass
class GlobalDescriptorExtractor():
    pass
class FisherVectorExtractorOptions():
    def __init__(self) -> None: ...
    @property
    def keep_only_symmetric_matches(self) -> int:
        """
        :type: int
        """
    @keep_only_symmetric_matches.setter
    def keep_only_symmetric_matches(self, arg0: int) -> None:
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
class FisherVectorExtractor(GlobalDescriptorExtractor):
    def AddFeaturesForTraining(self, arg0: typing.List[numpy.ndarray[numpy.float32, _Shape[m, 1]]]) -> None: ...
    def ExtractGlobalDescriptor(self, arg0: typing.List[numpy.ndarray[numpy.float32, _Shape[m, 1]]]) -> numpy.ndarray[numpy.float32, _Shape[m, 1]]: ...
    def Train(self) -> bool: ...
    def __init__(self, arg0: FisherVectorExtractorOptions) -> None: ...
    pass
class ImagePairMatch():
    def __init__(self) -> None: ...
    @property
    def correspondences(self) -> typing.List[theia::FeatureCorrespondence]:
        """
        :type: typing.List[theia::FeatureCorrespondence]
        """
    @correspondences.setter
    def correspondences(self, arg0: typing.List[theia::FeatureCorrespondence]) -> None:
        pass
    @property
    def image1(self) -> str:
        """
        :type: str
        """
    @image1.setter
    def image1(self, arg0: str) -> None:
        pass
    @property
    def image2(self) -> str:
        """
        :type: str
        """
    @image2.setter
    def image2(self, arg0: str) -> None:
        pass
    @property
    def twoview_info(self) -> theia::TwoViewInfo:
        """
        :type: theia::TwoViewInfo
        """
    @twoview_info.setter
    def twoview_info(self, arg0: theia::TwoViewInfo) -> None:
        pass
    pass
class InMemoryFeaturesAndMatchesDatabase(FeaturesAndMatchesDatabase):
    def ContainsCameraIntrinsicsPrior(self, arg0: str) -> bool: ...
    def ContainsFeatures(self, arg0: str) -> bool: ...
    @staticmethod
    def GetCameraIntrinsicsPrior(*args, **kwargs) -> typing.Any: ...
    @staticmethod
    def GetFeatures(*args, **kwargs) -> typing.Any: ...
    @staticmethod
    def GetImagePairMatch(*args, **kwargs) -> typing.Any: ...
    def ImageNamesOfCameraIntrinsicsPriors(self) -> typing.List[str]: ...
    def ImageNamesOfFeatures(self) -> typing.List[str]: ...
    def ImageNamesOfMatches(self) -> typing.List[typing.Tuple[str, str]]: ...
    def NumCameraIntrinsicsPrior(self) -> int: ...
    def NumImages(self) -> int: ...
    def NumMatches(self) -> int: ...
    @staticmethod
    def PutCameraIntrinsicsPrior(*args, **kwargs) -> typing.Any: ...
    @staticmethod
    def PutFeatures(*args, **kwargs) -> typing.Any: ...
    @staticmethod
    def PutImagePairMatch(*args, **kwargs) -> typing.Any: ...
    def __init__(self) -> None: ...
    pass
class IndexedFeatureMatch():
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, arg0: int, arg1: int, arg2: float) -> None: ...
    @property
    def distance(self) -> float:
        """
        :type: float
        """
    @distance.setter
    def distance(self, arg0: float) -> None:
        pass
    @property
    def feature1_ind(self) -> int:
        """
        :type: int
        """
    @feature1_ind.setter
    def feature1_ind(self, arg0: int) -> None:
        pass
    @property
    def feature2_ind(self) -> int:
        """
        :type: int
        """
    @feature2_ind.setter
    def feature2_ind(self, arg0: int) -> None:
        pass
    pass
class KeypointsAndDescriptors():
    def __init__(self) -> None: ...
    @property
    def descriptors(self) -> typing.List[numpy.ndarray[numpy.float32, _Shape[m, 1]]]:
        """
        :type: typing.List[numpy.ndarray[numpy.float32, _Shape[m, 1]]]
        """
    @descriptors.setter
    def descriptors(self, arg0: typing.List[numpy.ndarray[numpy.float32, _Shape[m, 1]]]) -> None:
        pass
    @property
    def image_name(self) -> str:
        """
        :type: str
        """
    @image_name.setter
    def image_name(self, arg0: str) -> None:
        pass
    @property
    def keypoints(self) -> typing.List[theia::Keypoint]:
        """
        :type: typing.List[theia::Keypoint]
        """
    @keypoints.setter
    def keypoints(self, arg0: typing.List[theia::Keypoint]) -> None:
        pass
    pass
class MatchingStrategy():
    """
    Members:

      GLOBAL

      INCREMENTAL
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
    GLOBAL: pytheia.pytheia.matching.MatchingStrategy # value = <MatchingStrategy.GLOBAL: 0>
    INCREMENTAL: pytheia.pytheia.matching.MatchingStrategy # value = <MatchingStrategy.INCREMENTAL: 1>
    __members__: dict # value = {'GLOBAL': <MatchingStrategy.GLOBAL: 0>, 'INCREMENTAL': <MatchingStrategy.INCREMENTAL: 1>}
    pass
GLOBAL: pytheia.pytheia.matching.MatchingStrategy # value = <MatchingStrategy.GLOBAL: 0>
INCREMENTAL: pytheia.pytheia.matching.MatchingStrategy # value = <MatchingStrategy.INCREMENTAL: 1>
