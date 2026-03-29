from __future__ import annotations
import collections.abc
import numpy
import numpy.typing
import typing

__all__: list[str] = [
    'FeatureCorrespondence',
    'FeatureMatcherOptions',
    'FeaturesAndMatchesDatabase',
    'GLOBAL',
    'GraphMatch',
    'INCREMENTAL',
    'ImagePairMatch',
    'InMemoryFeaturesAndMatchesDatabase',
    'IndexedFeatureMatch',
    'MatchingStrategy',
]

class FeatureCorrespondence:
    feature1: ...
    feature2: ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: ..., arg1: ...) -> None:
        ...

class FeatureMatcherOptions:
    geometric_verification_options: ...
    keep_only_symmetric_matches: bool
    perform_geometric_verification: bool
    use_lowes_ratio: bool
    def __init__(self) -> None:
        ...
    @property
    def lowes_ratio(self) -> float:
        ...
    @lowes_ratio.setter
    def lowes_ratio(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def min_num_feature_matches(self) -> int:
        ...
    @min_num_feature_matches.setter
    def min_num_feature_matches(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def num_threads(self) -> int:
        ...
    @num_threads.setter
    def num_threads(self, arg0: typing.SupportsInt) -> None:
        ...

class FeaturesAndMatchesDatabase:
    pass

class ImagePairMatch:
    image1: str
    image2: str
    twoview_info: ...
    def __init__(self) -> None:
        ...
    @property
    def correspondences(self) -> list[...]:
        ...
    @correspondences.setter
    def correspondences(self, arg0: collections.abc.Sequence[...]) -> None:
        ...

class InMemoryFeaturesAndMatchesDatabase(FeaturesAndMatchesDatabase):
    def ContainsCameraIntrinsicsPrior(self, arg0: str) -> bool:
        ...
    def GetCameraIntrinsicsPrior(self, arg0: str) -> ...:
        ...
    def GetImagePairMatch(self, arg0: str, arg1: str) -> ...:
        ...
    def ImageNamesOfCameraIntrinsicsPriors(self) -> list[str]:
        ...
    def ImageNamesOfMatches(self) -> list[tuple[str, str]]:
        ...
    def NumCameraIntrinsicsPrior(self) -> int:
        ...
    def NumMatches(self) -> int:
        ...
    def PutCameraIntrinsicsPrior(self, arg0: str, arg1: ...) -> None:
        ...
    def PutImagePairMatch(self, arg0: str, arg1: str, arg2: ...) -> None:
        ...
    def __init__(self) -> None:
        ...

class IndexedFeatureMatch:
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: typing.SupportsInt, arg1: typing.SupportsInt, arg2: typing.SupportsFloat) -> None:
        ...
    @property
    def distance(self) -> float:
        ...
    @distance.setter
    def distance(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def feature1_ind(self) -> int:
        ...
    @feature1_ind.setter
    def feature1_ind(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def feature2_ind(self) -> int:
        ...
    @feature2_ind.setter
    def feature2_ind(self, arg0: typing.SupportsInt) -> None:
        ...

class MatchingStrategy:
    """
    Members:
      GLOBAL
      INCREMENTAL
    """
    GLOBAL: typing.ClassVar[MatchingStrategy]
    INCREMENTAL: typing.ClassVar[MatchingStrategy]
    __members__: typing.ClassVar[dict[str, MatchingStrategy]]
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...

def GraphMatch(arg0: collections.abc.Sequence[str], arg1: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"]], arg2: typing.SupportsInt) -> list[tuple[str, str]]:
    ...

GLOBAL: MatchingStrategy
INCREMENTAL: MatchingStrategy
