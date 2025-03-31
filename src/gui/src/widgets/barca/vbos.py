from OpenGL.arrays import vbo

import numpy as np


def ellipse_vbo(rx=3.0, ry=2.0) -> vbo:
    """Create VBO for an ellipse centered at origin with specified radii"""
    segments = 100
    theta = np.linspace(0, 2 * np.pi, segments)

    # Parametric equations for ellipse
    x = rx * np.cos(theta)
    y = ry * np.sin(theta)

    # Create 3D vertices (x, y, 0)
    vertices = np.zeros((segments, 3), dtype=np.float32)
    vertices[:, 0] = x
    vertices[:, 1] = y

    return vbo.VBO(vertices)
