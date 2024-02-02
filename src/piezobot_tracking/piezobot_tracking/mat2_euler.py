#!/usr/bin/env python3


from transforms3d.euler import mat2euler
import numpy as np


matrix = np.array(
    [
        [0.0610471, -0.0296239, -0.997695],
        [-0.000426196, -0.99956, 0.0296532],
        [-0.998135, -0.00138503, -0.0610328],
    ]
)
print(mat2euler(matrix, axes="sxyz"))
