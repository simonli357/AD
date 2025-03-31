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


def grid_vbo(grid_size_x=35, grid_size_y=15):
    step = 1
    grid_vertices = []
    for x in range(-grid_size_x, grid_size_x + 4, step):
        grid_vertices.extend([x, -grid_size_y, 0, x, grid_size_y - 1, 0])
    for y in range(-grid_size_y, grid_size_y, step):
        grid_vertices.extend([-grid_size_x, y, 0, grid_size_x + 3, y, 0])

    grid_vbo = vbo.VBO(np.array(grid_vertices, dtype=np.float32))
    grid_vertex_count = len(grid_vertices) // 3
    return grid_vbo, grid_vertex_count
