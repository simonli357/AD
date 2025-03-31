from OpenGL.arrays import vbo

import numpy as np


def ellipse_vbo() -> vbo:
    rx = 1.0 / 70.0
    ry = 1.0 / 25.0
    segments = 100
    theta = np.linspace(0, 2 * np.pi, segments)
    x = rx * np.cos(theta)
    y = ry * np.sin(theta)
    vertices = np.column_stack((x, y))
    vertices = np.append(vertices, [vertices[0]], axis=0)
    return vbo.VBO(vertices.astype(np.float32))
