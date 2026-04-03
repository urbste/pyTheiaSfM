from __future__ import annotations
from . import Sophus
__all__: list[str] = ['AlignOrientations', 'AlignRotations', 'ApplyRelativeRotation', 'FindQuadraticPolynomialRoots', 'MultiplyRotations', 'RelativeRotationFromTwoRotations', 'RelativeTranslationFromTwoPositions', 'SE3FromRotationTranslation', 'SE3d', 'Sim3FromRotationTranslationScale', 'Sim3d', 'Sophus']
class SE3d:
    @staticmethod
    def __init__(*args, **kwargs):
        """
        Default constructor
        Constructor from SO3 and translation
        Constructor from quaternion and translation
        Constructor from rotation matrix and translation
        Constructor from rotation matrix and translation
        Constructor from quaternion [w, x, y, z] and translation
        """
    @staticmethod
    def __mul__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        ...
    @staticmethod
    def adjoint(*args, **kwargs):
        """
        Get adjoint matrix
        """
    @staticmethod
    def exp(*args, **kwargs):
        """
        Exponential map
        """
    @staticmethod
    def inverse(*args, **kwargs):
        """
        Get inverse transformation
        """
    @staticmethod
    def log(*args, **kwargs):
        """
        Get Lie algebra (tangent vector)
        """
    @staticmethod
    def matrix(*args, **kwargs):
        """
        Get 4x4 transformation matrix
        """
    @staticmethod
    def matrix3x4(*args, **kwargs):
        """
        Get 3x4 matrix
        """
    @staticmethod
    def params(*args, **kwargs):
        """
        Get internal parameters
        """
    @staticmethod
    def rot_x(*args, **kwargs):
        """
        Rotation around X axis
        """
    @staticmethod
    def rot_y(*args, **kwargs):
        """
        Rotation around Y axis
        """
    @staticmethod
    def rot_z(*args, **kwargs):
        """
        Rotation around Z axis
        """
    @staticmethod
    def rotation_matrix(*args, **kwargs):
        """
        Get 3x3 rotation matrix
        """
    @staticmethod
    def set_quaternion(*args, **kwargs):
        """
        Set quaternion
        """
    @staticmethod
    def set_rotation_matrix(*args, **kwargs):
        """
        Set rotation matrix
        """
    @staticmethod
    def trans(*args, **kwargs):
        """
        Translation
        Translation
        """
    @staticmethod
    def trans_x(*args, **kwargs):
        """
        Translation along X axis
        """
    @staticmethod
    def trans_y(*args, **kwargs):
        """
        Translation along Y axis
        """
    @staticmethod
    def trans_z(*args, **kwargs):
        """
        Translation along Z axis
        """
    @staticmethod
    def translation(*args, **kwargs):
        """
        Get translation vector
        """
class Sim3d:
    @staticmethod
    def __init__(*args, **kwargs):
        """
        Default constructor
        Constructor from quaternion and translation
        Constructor from rotation matrix, translation, and scale
        Constructor from quaternion [w, x, y, z], translation, and scale
        Constructor from tangent vector using exponential map
        """
    @staticmethod
    def __mul__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        ...
    @staticmethod
    def adjoint(*args, **kwargs):
        """
        Get adjoint matrix
        """
    @staticmethod
    def exp(*args, **kwargs):
        """
        Exponential map
        """
    @staticmethod
    def inverse(*args, **kwargs):
        """
        Get inverse transformation
        """
    @staticmethod
    def log(*args, **kwargs):
        """
        Get Lie algebra (tangent vector)
        """
    @staticmethod
    def matrix(*args, **kwargs):
        """
        Get 4x4 transformation matrix
        """
    @staticmethod
    def matrix3x4(*args, **kwargs):
        """
        Get 3x4 matrix
        """
    @staticmethod
    def params(*args, **kwargs):
        """
        Get internal parameters
        """
    @staticmethod
    def quaternion(*args, **kwargs):
        """
        Get quaternion
        """
    @staticmethod
    def scale(*args, **kwargs):
        """
        Get scale factor
        """
    @staticmethod
    def set_quaternion(*args, **kwargs):
        """
        Set quaternion
        """
    @staticmethod
    def set_rotation_matrix(*args, **kwargs):
        """
        Set rotation matrix
        """
    @staticmethod
    def set_scale(*args, **kwargs):
        """
        Set scale factor
        """
    @staticmethod
    def set_scaled_rotation_matrix(*args, **kwargs):
        """
        Set scaled rotation matrix
        """
    @staticmethod
    def translation(*args, **kwargs):
        """
        Get translation vector
        """
def AlignOrientations(*args, **kwargs):
    """
    This functions takes as input a dictionary of view_ids to global orientations that should be aligned. Then it calls AlignRotations internally.
    """
def AlignRotations(*args, **kwargs):
    """
    Solves a nonlinear least squares problem so that: rotations * R = gt_rotations.
    """
def ApplyRelativeRotation(*args, **kwargs):
    """
    returns R2 = R12 * R1
    """
def FindQuadraticPolynomialRoots(*args, **kwargs):
    ...
def MultiplyRotations(*args, **kwargs):
    """
    return R = R1 * R2
    """
def RelativeRotationFromTwoRotations(*args, **kwargs):
    """
    returns R12 = R2 * R1^T
    """
def RelativeTranslationFromTwoPositions(*args, **kwargs):
    """
    returns t12 = R1*(p2-p1)
    """
def SE3FromRotationTranslation(*args, **kwargs):
    """
    Create SE3 from rotation matrix and translation vector
    """
def Sim3FromRotationTranslationScale(*args, **kwargs):
    """
    Create Sim3 from rotation matrix, translation vector, and scale factor
    """
