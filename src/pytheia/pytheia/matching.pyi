from __future__ import annotations
import typing
__all__: list[str] = ['FeatureCorrespondence', 'FeatureMatcherOptions', 'GLOBAL', 'GraphMatch', 'INCREMENTAL', 'ImagePairMatch', 'IndexedFeatureMatch', 'MatchingStrategy']
class FeatureCorrespondence:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def feature1(*args, **kwargs):
        ...
    @feature1.setter
    def feature1(*args, **kwargs):
        ...
    @property
    def feature2(*args, **kwargs):
        ...
    @feature2.setter
    def feature2(*args, **kwargs):
        ...
class FeatureMatcherOptions:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def geometric_verification_options(*args, **kwargs):
        ...
    @geometric_verification_options.setter
    def geometric_verification_options(*args, **kwargs):
        ...
    @property
    def keep_only_symmetric_matches(*args, **kwargs):
        ...
    @keep_only_symmetric_matches.setter
    def keep_only_symmetric_matches(*args, **kwargs):
        ...
    @property
    def lowes_ratio(*args, **kwargs):
        ...
    @lowes_ratio.setter
    def lowes_ratio(*args, **kwargs):
        ...
    @property
    def min_num_feature_matches(*args, **kwargs):
        ...
    @min_num_feature_matches.setter
    def min_num_feature_matches(*args, **kwargs):
        ...
    @property
    def num_threads(*args, **kwargs):
        ...
    @num_threads.setter
    def num_threads(*args, **kwargs):
        ...
    @property
    def perform_geometric_verification(*args, **kwargs):
        ...
    @perform_geometric_verification.setter
    def perform_geometric_verification(*args, **kwargs):
        ...
    @property
    def use_lowes_ratio(*args, **kwargs):
        ...
    @use_lowes_ratio.setter
    def use_lowes_ratio(*args, **kwargs):
        ...
class ImagePairMatch:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def correspondences(*args, **kwargs):
        ...
    @correspondences.setter
    def correspondences(*args, **kwargs):
        ...
    @property
    def image1(*args, **kwargs):
        ...
    @image1.setter
    def image1(*args, **kwargs):
        ...
    @property
    def image2(*args, **kwargs):
        ...
    @image2.setter
    def image2(*args, **kwargs):
        ...
    @property
    def twoview_info(*args, **kwargs):
        ...
    @twoview_info.setter
    def twoview_info(*args, **kwargs):
        ...
class IndexedFeatureMatch:
    @staticmethod
    def __init__(*args, **kwargs):
        ...
    @property
    def distance(*args, **kwargs):
        ...
    @distance.setter
    def distance(*args, **kwargs):
        ...
    @property
    def feature1_ind(*args, **kwargs):
        ...
    @feature1_ind.setter
    def feature1_ind(*args, **kwargs):
        ...
    @property
    def feature2_ind(*args, **kwargs):
        ...
    @feature2_ind.setter
    def feature2_ind(*args, **kwargs):
        ...
class MatchingStrategy:
    """
    Members:
    
      GLOBAL
    
      INCREMENTAL
    """
    GLOBAL: typing.ClassVar[MatchingStrategy]  # value = <MatchingStrategy.GLOBAL: 0>
    INCREMENTAL: typing.ClassVar[MatchingStrategy]  # value = <MatchingStrategy.GLOBAL: 0>
    __members__: typing.ClassVar[dict[str, MatchingStrategy]]  # value = {'GLOBAL': <MatchingStrategy.GLOBAL: 0>, 'INCREMENTAL': <MatchingStrategy.GLOBAL: 0>}
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
def GraphMatch(*args, **kwargs):
    ...
GLOBAL: MatchingStrategy  # value = <MatchingStrategy.GLOBAL: 0>
INCREMENTAL: MatchingStrategy  # value = <MatchingStrategy.GLOBAL: 0>
