from OpenGL.arrays import vbo
from ..enums import BarcaMapData

import numpy as np


def ellipse_vbo() -> vbo:
    rx = 1.0 / (BarcaMapData.MAP_WIDTH.value * 0.01)
    ry = 1.0 / (BarcaMapData.MAP_HEIGHT.value * 0.01)
    segments = 100
    theta = np.linspace(0, 2 * np.pi, segments)
    x = rx * np.cos(theta)
    y = ry * np.sin(theta)
    vertices = np.column_stack((x, y))
    vertices = np.append(vertices, [vertices[0]], axis=0)
    return vbo.VBO(vertices.astype(np.float32))
