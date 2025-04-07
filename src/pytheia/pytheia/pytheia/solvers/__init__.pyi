"""Type stubs for the pytheia.solvers module."""
from typing import List, Optional, Set, Tuple, Union, Dict, Any
import numpy as np
from numpy.typing import NDArray

class RansacSummary:
    inliers: List[int]
    num_input_data_points: int
    num_iterations: int
    confidence: float
    num_lo_iterations: int

class RansacParameters:
    def __init__(self) -> None: ...
    error_thresh: float
    failure_probability: float
    min_inlier_ratio: float
    min_iterations: int
    max_iterations: int
    use_mle: bool
    use_lo: bool
    lo_start_iterations: int
    use_Tdd_test: bool

class RandomNumberGenerator:
    def __init__(self, seed: Optional[int] = None) -> None: ...
    def Seed(self, seed: int) -> None: ...
    def RandDouble(self, min_val: float = 0.0, max_val: float = 1.0) -> float: ...
    def RandFloat(self, min_val: float = 0.0, max_val: float = 1.0) -> float: ...
    def RandInt(self, min_val: int, max_val: int) -> int: ...
    def RandGaussian(self, mean: float = 0.0, std_dev: float = 1.0) -> float: ...