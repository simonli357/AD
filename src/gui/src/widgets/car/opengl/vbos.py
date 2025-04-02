from OpenGL.arrays import vbo

import numpy as np


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
