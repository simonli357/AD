from OpenGL.arrays import vbo

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


def sign_vbo():
    # Define track quad vertices and texture coordinates
    # Format: [x, y, z, u, v]
    track_vertices = np.array([
        # Bottom-left
        0.0, 0.0, 0.0, 0.0, 0.0,
        # Bottom-right
        200.0, 0.0, 0.0, 1.0, 0.0,
        # Top-right
        200.0, 200.0, 0.0, 1.0, 1.0,
        # Top-left
        0.0, 200.0, 0.0, 0.0, 1.0
    ], dtype=np.float32)

    # Create VBO for the track
    return vbo.VBO(track_vertices)


def circle_vbo(radius=1.0, segments=64, z=0.0):
    """Create a VBO for a circle with specified radius and resolution.

    Args:
        radius (float): Radius of the circle
        segments (int): Number of vertices around the circumference
        z (float): Z-coordinate for all vertices

    Returns:
        tuple: (VBO object, vertex_count)
    """
    vertices = []

    # Center vertex (reused for all triangles)
    center = [0.0, 0.0, z]

    for i in range(segments):
        # Calculate angles for current and next segment
        theta1 = 2 * np.pi * i / segments
        theta2 = 2 * np.pi * (i + 1) / segments

        # Calculate perimeter points
        x1 = radius * np.cos(theta1)
        y1 = radius * np.sin(theta1)
        x2 = radius * np.cos(theta2)
        y2 = radius * np.sin(theta2)

        # Add triangle vertices (center -> point1 -> point2)
        vertices.extend(center)
        vertices.extend([x1, y1, z])
        vertices.extend([x2, y2, z])

    # Create numpy array and VBO
    vertex_array = np.array(vertices, dtype=np.float32)
    circle_buffer = vbo.VBO(vertex_array)

    # 3 vertices per triangle * number of segments
    vertex_count = len(vertices) // 3

    return circle_buffer, vertex_count
