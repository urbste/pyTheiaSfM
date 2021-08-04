import pytheia.pytheia.io
import typing
import numpy
_Shape = typing.Tuple[int, ...]

__all__ = [
    "BundlerCamera",
    "BundlerFileReader",
    "FeatureInfo",
    "ImportNVMFile",
    "ListImgEntry",
    "PopulateImageSizesAndPrincipalPoints",
    "Read1DSFM",
    "ReadBundlerFiles",
    "ReadKeypointsAndDescriptors",
    "ReadReconstruction",
    "ReadSiftKeyBinaryFile",
    "ReadSiftKeyTextFile",
    "ReadStrechaDataset",
    "WriteBundlerFiles",
    "WriteColmapFiles",
    "WriteKeypointsAndDescriptors",
    "WriteNVMFile",
    "WritePlyFile",
    "WriteReconstruction",
    "WriteReconstructionJson",
    "WriteSiftKeyBinaryFile"
]


class BundlerCamera():
    def __init__(self) -> None: ...
    @property
    def focal_length(self) -> float:
        """
        :type: float
        """
    @focal_length.setter
    def focal_length(self, arg0: float) -> None:
        pass
    @property
    def radial_coeff_1(self) -> float:
        """
        :type: float
        """
    @radial_coeff_1.setter
    def radial_coeff_1(self, arg0: float) -> None:
        pass
    @property
    def radial_coeff_2(self) -> float:
        """
        :type: float
        """
    @radial_coeff_2.setter
    def radial_coeff_2(self, arg0: float) -> None:
        pass
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
class BundlerFileReader():
    def ParseBundleFile(self) -> bool: ...
    def ParseListsFile(self) -> bool: ...
    def __init__(self, arg0: str, arg1: str) -> None: ...
    def cameras(self) -> typing.List[theia::BundlerCamera]: ...
    def img_entries(self) -> typing.List[theia::ListImgEntry]: ...
    def points(self) -> typing.List[theia::BundlerPoint]: ...
    pass
class FeatureInfo():
    def __init__(self) -> None: ...
    @property
    def camera_index(self) -> int:
        """
        :type: int
        """
    @camera_index.setter
    def camera_index(self, arg0: int) -> None:
        pass
    @property
    def kpt_x(self) -> int:
        """
        :type: int
        """
    @kpt_x.setter
    def kpt_x(self, arg0: int) -> None:
        pass
    @property
    def kpt_y(self) -> int:
        """
        :type: int
        """
    @kpt_y.setter
    def kpt_y(self, arg0: int) -> None:
        pass
    @property
    def sift_index(self) -> int:
        """
        :type: int
        """
    @sift_index.setter
    def sift_index(self, arg0: int) -> None:
        pass
    pass
class ListImgEntry():
    def __init__(self) -> None: ...
    @property
    def filename(self) -> str:
        """
        :type: str
        """
    @filename.setter
    def filename(self, arg0: str) -> None:
        pass
    @property
    def focal_length(self) -> float:
        """
        :type: float
        """
    @focal_length.setter
    def focal_length(self, arg0: float) -> None:
        pass
    @property
    def second_entry(self) -> float:
        """
        :type: float
        """
    @second_entry.setter
    def second_entry(self, arg0: float) -> None:
        pass
    pass
def ImportNVMFile(arg0: str) -> typing.Tuple[bool, theia::Reconstruction]:
    pass
def PopulateImageSizesAndPrincipalPoints(arg0: str) -> typing.Tuple[bool, theia::Reconstruction]:
    pass
def Read1DSFM(arg0: str) -> typing.Tuple[bool, theia::Reconstruction, theia::ViewGraph]:
    pass
def ReadBundlerFiles(arg0: str, arg1: str) -> typing.Tuple[bool, theia::Reconstruction]:
    pass
def ReadKeypointsAndDescriptors(arg0: str) -> typing.Tuple[bool, typing.List[theia::Keypoint], typing.List[numpy.ndarray[numpy.float32, _Shape[m, 1]]]]:
    pass
def ReadReconstruction(arg0: str) -> typing.Tuple[bool, theia::Reconstruction]:
    pass
def ReadSiftKeyBinaryFile(arg0: str) -> typing.Tuple[bool, typing.List[numpy.ndarray[numpy.float32, _Shape[m, 1]]], typing.List[theia::Keypoint]]:
    pass
def ReadSiftKeyTextFile(arg0: str) -> typing.Tuple[bool, typing.List[numpy.ndarray[numpy.float32, _Shape[m, 1]]], typing.List[theia::Keypoint]]:
    pass
def ReadStrechaDataset(arg0: str) -> typing.Tuple[bool, theia::Reconstruction]:
    pass
def WriteBundlerFiles(*args, **kwargs) -> typing.Any:
    pass
def WriteColmapFiles(*args, **kwargs) -> typing.Any:
    pass
def WriteKeypointsAndDescriptors(arg0: str, arg1: typing.List[theia::Keypoint], arg2: typing.List[numpy.ndarray[numpy.float32, _Shape[m, 1]]]) -> bool:
    pass
def WriteNVMFile(*args, **kwargs) -> typing.Any:
    pass
def WritePlyFile(*args, **kwargs) -> typing.Any:
    pass
def WriteReconstruction(*args, **kwargs) -> typing.Any:
    pass
def WriteReconstructionJson(*args, **kwargs) -> typing.Any:
    pass
def WriteSiftKeyBinaryFile(arg0: str, arg1: typing.List[numpy.ndarray[numpy.float32, _Shape[m, 1]]], arg2: typing.List[theia::Keypoint]) -> bool:
    pass
