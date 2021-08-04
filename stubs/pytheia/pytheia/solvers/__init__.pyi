import pytheia.pytheia.solvers
import typing

__all__ = [
    "RandomNumberGenerator",
    "RansacParameters",
    "RansacSummary"
]


class RandomNumberGenerator():
    def RandDouble(self, arg0: float, arg1: float) -> float: ...
    def RandFloat(self, arg0: float, arg1: float) -> float: ...
    def RandGaussian(self, arg0: float, arg1: float) -> float: ...
    def RandInt(self, arg0: int, arg1: int) -> int: ...
    def Seed(self, arg0: int) -> None: ...
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, arg0: int) -> None: ...
    pass
class RansacParameters():
    def __init__(self) -> None: ...
    @property
    def error_thresh(self) -> float:
        """
        :type: float
        """
    @error_thresh.setter
    def error_thresh(self, arg0: float) -> None:
        pass
    @property
    def failure_probability(self) -> float:
        """
        :type: float
        """
    @failure_probability.setter
    def failure_probability(self, arg0: float) -> None:
        pass
    @property
    def max_iterations(self) -> int:
        """
        :type: int
        """
    @max_iterations.setter
    def max_iterations(self, arg0: int) -> None:
        pass
    @property
    def min_inlier_ratio(self) -> float:
        """
        :type: float
        """
    @min_inlier_ratio.setter
    def min_inlier_ratio(self, arg0: float) -> None:
        pass
    @property
    def min_iterations(self) -> int:
        """
        :type: int
        """
    @min_iterations.setter
    def min_iterations(self, arg0: int) -> None:
        pass
    @property
    def use_Tdd_test(self) -> bool:
        """
        :type: bool
        """
    @use_Tdd_test.setter
    def use_Tdd_test(self, arg0: bool) -> None:
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
class RansacSummary():
    @property
    def confidence(self) -> float:
        """
        :type: float
        """
    @confidence.setter
    def confidence(self, arg0: float) -> None:
        pass
    @property
    def inliers(self) -> typing.List[int]:
        """
        :type: typing.List[int]
        """
    @inliers.setter
    def inliers(self, arg0: typing.List[int]) -> None:
        pass
    @property
    def num_input_data_points(self) -> int:
        """
        :type: int
        """
    @num_input_data_points.setter
    def num_input_data_points(self, arg0: int) -> None:
        pass
    @property
    def num_iterations(self) -> int:
        """
        :type: int
        """
    @num_iterations.setter
    def num_iterations(self, arg0: int) -> None:
        pass
    pass
