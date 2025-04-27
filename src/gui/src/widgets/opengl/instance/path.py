from ..renderer import InstanceRenderer


class PathRenderer(InstanceRenderer):
    def __init__(self, positions, colors=(1.0, 1.0, 0.0, 1.0), rotations=None, scales=1.0):
        base_vertices = [
            0.0, 0.6667, 0.0,
            -0.5, -0.3333, 0.0,
            0.5, -0.3333, 0.0
        ]

        super().__init__(
            base_vertices=base_vertices,
            positions=positions,
            colors=colors,
            rotations=rotations,
            scales=scales,
        )
