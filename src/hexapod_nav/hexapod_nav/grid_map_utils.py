"""Shared GridMap MultiArray layout helpers.

grid_map_ros expects dim labels ``column_index`` / ``row_index`` (not
``column`` / ``row``). Wrong labels make RViz spam:
  isRowMajor() failed because layout label is not set correctly.
"""

from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import numpy as np


def make_layer(data: np.ndarray) -> Float32MultiArray:
    """Pack a square (N, N) float array as column-major GridMap layer data."""
    n = int(data.shape[0])
    layer = Float32MultiArray()
    # Column-major storage: dim[0]=column_index, dim[1]=row_index
    layer.layout.dim = [
        MultiArrayDimension(label='column_index', size=n, stride=n * n),
        MultiArrayDimension(label='row_index', size=n, stride=n),
    ]
    flat = np.asarray(data, dtype=np.float32).flatten(order='F')
    layer.data = flat.tolist()
    return layer
