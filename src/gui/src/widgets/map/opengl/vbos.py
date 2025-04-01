from OpenGL.arrays import vbo
from ...enums import MapData

import numpy as np


def grid_vbo(grid_size_x=32, grid_size_y=15):
    step = 1
    grid_vertices = []
    for x in range(-grid_size_x, grid_size_x + 4, step):
        grid_vertices.extend([x, -grid_size_y, 0, x, grid_size_y - 1, 0])
    for y in range(-grid_size_y, grid_size_y, step):
        grid_vertices.extend([-grid_size_x, y, 0, grid_size_x + 3, y, 0])

    grid_vbo = vbo.VBO(np.array(grid_vertices, dtype=np.float32))
    grid_vertex_count = len(grid_vertices) // 3
    return grid_vbo, grid_vertex_count


def track_vbo(width, height):
    # Define track quad vertices and texture coordinates
    # Format: [x, y, z, u, v]
    track_vertices = np.array([
        # Bottom-left
        0.0, 0.0, 0.0, 0.0, 0.0,
        # Bottom-right
        width, 0.0, 0.0, 1.0, 0.0,
        # Top-right
        width, height, 0.0, 1.0, 1.0,
        # Top-left
        0.0, height, 0.0, 0.0, 1.0
    ], dtype=np.float32)

    # Create VBO for the track
    return vbo.VBO(track_vertices)
