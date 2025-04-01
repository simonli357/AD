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


def marker_vbo(radius=4.0, segments=32, line_scale=0.8):
    """Create a VBO for a crosshair marker with circle and cross.

    Args:
        radius (float): Outer radius of the marker
        segments (int): Number of vertices for the circle
        line_scale (float): Length of cross lines relative to radius (0.8 = 80%)

    Returns:
        tuple: (VBO object, (circle_vertex_count, cross_vertex_count))
    """
    vertices = []

    # 1. Create circle vertices (GL_LINE_LOOP)
    for i in range(segments):
        theta = 2 * np.pi * i / segments
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        vertices.extend([x, y, 0.0])

    # 2. Create crosshair vertices (GL_LINES)
    # Horizontal line (80% of radius length)
    h_length = radius * line_scale
    vertices.extend([-h_length, 0.0, 0.0])  # Start
    vertices.extend([h_length, 0.0, 0.0])   # End

    # Vertical line (80% of radius length)
    v_length = radius * line_scale
    vertices.extend([0.0, -v_length, 0.0])  # Start
    vertices.extend([0.0, v_length, 0.0])   # End

    # Create numpy array and VBO
    vertex_array = np.array(vertices, dtype=np.float32)
    marker_buffer = vbo.VBO(vertex_array)

    # Return vertex counts (circle, cross)
    circle_vertex_count = segments
    cross_vertex_count = 4  # 2 lines * 2 points each
    return marker_buffer, (circle_vertex_count, cross_vertex_count)
