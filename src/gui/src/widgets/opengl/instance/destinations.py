from ..renderer import InstanceRenderer

import numpy as np


class DestinationsRenderer(InstanceRenderer):
    def __init__(self, positions, colors=(0.0, 0.7, 0.7, 1.0), rotations=None, scales=None):
        base_vertices = []
        center = (0.0, 0.0, 0.0)
        num_segs = 64
        for i in range(num_segs):
            theta0 = 2 * np.pi * i / num_segs
            theta1 = 2 * np.pi * (i + 1) / num_segs
            v0 = (np.cos(theta0), np.sin(theta0), 0.0)
            v1 = (np.cos(theta1), np.sin(theta1), 0.0)
            base_vertices += center + v0 + v1

        super().__init__(
            base_vertices=base_vertices,
            positions=positions,
            colors=colors,
            rotations=rotations,
            scales=scales,
        )
