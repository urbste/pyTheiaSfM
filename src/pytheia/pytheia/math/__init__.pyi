from __future__ import annotations
import collections.abc
import numpy
import numpy.typing
import typing
from . import Sophus
__all__: list[str] = ['AlignOrientations', 'AlignRotations', 'ApplyRelativeRotation', 'FindQuadraticPolynomialRoots', 'MultiplyRotations', 'RelativeRotationFromTwoRotations', 'RelativeTranslationFromTwoPositions', 'SE3FromRotationTranslation', 'SE3d', 'Sim3FromRotationTranslationScale', 'Sim3d', 'Sophus']
class SE3d:
    @staticmethod
    @typing.overload
    def __init__(*args, **kwargs) -> None:
        """
        Constructor from SO3 and translation
        """
    @staticmethod
    @typing.overload
    def __init__(*args, **kwargs) -> None:
        """
        Constructor from quaternion and translation
        """
    @staticmethod
    def exp(tangent: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 1]"]) -> SE3d:
        """
        Exponential map
        """
    @staticmethod
    def rot_x(x: typing.SupportsFloat) -> SE3d:
        """
        Rotation around X axis
        """
    @staticmethod
    def rot_y(y: typing.SupportsFloat) -> SE3d:
        """
        Rotation around Y axis
        """
    @staticmethod
    def rot_z(z: typing.SupportsFloat) -> SE3d:
        """
        Rotation around Z axis
        """
    @staticmethod
    def set_quaternion(*args, **kwargs) -> None:
        """
        Set quaternion
        """
    @staticmethod
    @typing.overload
    def trans(xyz: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> SE3d:
        """
        Translation
        """
    @staticmethod
    @typing.overload
    def trans(x: typing.SupportsFloat, y: typing.SupportsFloat, z: typing.SupportsFloat) -> SE3d:
        """
        Translation
        """
    @staticmethod
    def trans_x(x: typing.SupportsFloat) -> SE3d:
        """
        Translation along X axis
        """
    @staticmethod
    def trans_y(y: typing.SupportsFloat) -> SE3d:
        """
        Translation along Y axis
        """
    @staticmethod
    def trans_z(z: typing.SupportsFloat) -> SE3d:
        """
        Translation along Z axis
        """
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor
        """
    @typing.overload
    def __init__(self, rotation_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 3]"], translation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> None:
        """
        Constructor from rotation matrix and translation
        """
    @typing.overload
    def __init__(self, rotation_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 3]"], translation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> None:
        """
        Constructor from rotation matrix and translation
        """
    @typing.overload
    def __init__(self, quaternion: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"], translation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> None:
        """
        Constructor from quaternion [w, x, y, z] and translation
        """
    @typing.overload
    def __mul__(self, arg0: SE3d) -> SE3d:
        ...
    @typing.overload
    def __mul__(self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]"]:
        ...
    @typing.overload
    def __mul__(self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[4, 1]"]:
        ...
    def __repr__(self) -> str:
        ...
    def adjoint(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[6, 6]"]:
        """
        Get adjoint matrix
        """
    def inverse(self) -> SE3d:
        """
        Get inverse transformation
        """
    def log(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[6, 1]"]:
        """
        Get Lie algebra (tangent vector)
        """
    def matrix(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[4, 4]"]:
        """
        Get 4x4 transformation matrix
        """
    def matrix3x4(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 4]"]:
        """
        Get 3x4 matrix
        """
    def params(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 1]"]:
        """
        Get internal parameters
        """
    def rotation_matrix(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 3]"]:
        """
        Get 3x3 rotation matrix
        """
    def set_rotation_matrix(self, R: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 3]"]) -> None:
        """
        Set rotation matrix
        """
    def translation(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]"]:
        """
        Get translation vector
        """
class Sim3d:
    @staticmethod
    @typing.overload
    def __init__(*args, **kwargs) -> None:
        """
        Constructor from quaternion and translation
        """
    @staticmethod
    def exp(tangent: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"]) -> Sim3d:
        """
        Exponential map
        """
    @staticmethod
    def set_quaternion(*args, **kwargs) -> None:
        """
        Set quaternion
        """
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor
        """
    @typing.overload
    def __init__(self, rotation_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 3]"], translation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], scale: typing.SupportsFloat) -> None:
        """
        Constructor from rotation matrix, translation, and scale
        """
    @typing.overload
    def __init__(self, quaternion: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"], translation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], scale: typing.SupportsFloat) -> None:
        """
        Constructor from quaternion [w, x, y, z], translation, and scale
        """
    @typing.overload
    def __init__(self, tangent_vector: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"]) -> None:
        """
        Constructor from tangent vector using exponential map
        """
    @typing.overload
    def __mul__(self, arg0: Sim3d) -> Sim3d:
        ...
    @typing.overload
    def __mul__(self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]"]:
        ...
    @typing.overload
    def __mul__(self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[4, 1]"]:
        ...
    def __repr__(self) -> str:
        ...
    def adjoint(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 7]"]:
        """
        Get adjoint matrix
        """
    def inverse(self) -> Sim3d:
        """
        Get inverse transformation
        """
    def log(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 1]"]:
        """
        Get Lie algebra (tangent vector)
        """
    def matrix(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[4, 4]"]:
        """
        Get 4x4 transformation matrix
        """
    def matrix3x4(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 4]"]:
        """
        Get 3x4 matrix
        """
    def params(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 1]"]:
        """
        Get internal parameters
        """
    def quaternion(self) -> ...:
        """
        Get quaternion
        """
    def scale(self) -> float:
        """
        Get scale factor
        """
    def set_rotation_matrix(self, R: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 3]"]) -> None:
        """
        Set rotation matrix
        """
    def set_scale(self, scale: typing.SupportsFloat) -> None:
        """
        Set scale factor
        """
    def set_scaled_rotation_matrix(self, sR: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 3]"]) -> None:
        """
        Set scaled rotation matrix
        """
    def translation(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]"]:
        """
        Get translation vector
        """
def AlignOrientations(arg0: collections.abc.Mapping[typing.SupportsInt, typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]], arg1: collections.abc.Mapping[typing.SupportsInt, typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]]) -> dict[int, typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]"]]:
    """
    This functions takes as input a dictionary of view_ids to global orientations that should be aligned. Then it calls AlignRotations internally.
    """
def AlignRotations(arg0: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]], arg1: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]]) -> None:
    """
    Solves a nonlinear least squares problem so that: rotations * R = gt_rotations.
    """
def ApplyRelativeRotation(arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]"]:
    """
    returns R2 = R12 * R1
    """
def FindQuadraticPolynomialRoots(arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"], arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"], arg2: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"]) -> None:
    ...
def MultiplyRotations(arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]"]:
    """
    return R = R1 * R2
    """
def RelativeRotationFromTwoRotations(arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]"]:
    """
    returns R12 = R2 * R1^T
    """
def RelativeTranslationFromTwoPositions(arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], arg2: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]"]:
    """
    returns t12 = R1*(p2-p1)
    """
def SE3FromRotationTranslation(rotation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 3]"], translation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> SE3d:
    """
    Create SE3 from rotation matrix and translation vector
    """
def Sim3FromRotationTranslationScale(rotation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 3]"], translation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], scale: typing.SupportsFloat) -> Sim3d:
    """
    Create Sim3 from rotation matrix, translation vector, and scale factor
    """
