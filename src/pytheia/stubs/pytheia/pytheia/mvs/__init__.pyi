"""Type stubs for the pytheia.mvs module."""
from typing import Dict, List, Optional, Tuple, Union, Any
import numpy as np
from numpy.typing import NDArray

def ViewSelectionMVSNet(poses: List[NDArray], 
                        intrinsic_parameters: List[NDArray], 
                        reference_image_id: int, 
                        num_views: int) -> List[int]: ...