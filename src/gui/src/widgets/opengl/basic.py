from OpenGL import GL as gl
from OpenGL.arrays import vbo
from collections import namedtuple

import numpy as np
import math

Model = namedtuple('Model', ['vao', 'vbo', 'ebo', 'vertex_count'])
Model2 = namedtuple('Model2', ['vao1', 'vao2', 'vbo1', 'vbo2', 'v_count1', 'v_count2'])


def line_model() -> Model:
    vertex_array = np.array([
        [0.0, 0.0],  # Start point
        [1.0, 1.0]    # End point
    ], dtype=np.float32).flatten()

    vao = gl.glGenVertexArrays(1)
    gl.glBindVertexArray(vao)

    # Create and configure VBO
    line_vbo = vbo.VBO(vertex_array)
    line_vbo.bind()

    # Set vertex attribute pointer
    gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))
    gl.glEnableVertexAttribArray(0)

    gl.glBindVertexArray(0)
    line_vbo.unbind()

    return Model(vao, line_vbo, None, 2)


def triangle_model() -> Model:
    vertex_array = np.array([
        0.0, 0.6667, 0.0,
        -0.5, -0.3333, 0.0,
        0.5, -0.3333, 0.0
    ], dtype=np.float32).flatten()

    vao = gl.glGenVertexArrays(1)
    gl.glBindVertexArray(vao)

    triangle_vbo = vbo.VBO(vertex_array)
    triangle_vbo.bind()

    gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))
    gl.glEnableVertexAttribArray(0)

    gl.glBindVertexArray(0)
    triangle_vbo.unbind()

    return Model(vao, triangle_vbo, None, 3)


def circle_model() -> Model:
    vertices = []
    for i in range(64):
        angle = 6.28318530718 * float(i) / 63.0
        vertices.append([math.cos(angle), math.sin(angle)])

    vao = gl.glGenVertexArrays(1)
    gl.glBindVertexArray(vao)

    # Create and configure VBO
    circle_vbo = vbo.VBO(np.array(vertices, dtype=np.float32))
    circle_vbo.bind()

    # Set vertex attribute pointer
    gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))
    gl.glEnableVertexAttribArray(0)

    gl.glBindVertexArray(0)
    circle_vbo.unbind()

    return Model(vao, circle_vbo, None, len(vertices))


def arrow_model() -> Model:
    vertices = np.array([
        # Rect Triangle 1
        -0.04, 0.0, 0.0,
        -0.04, 0.8, 0.0,
        0.04, 0.8, 0.0,
        # Rect Triangle 2
        0.04, 0.0, 0.0,
        0.04, 0.8, 0.0,
        -0.04, 0.0, 0.0,
        # Arrow Triangle
        -0.1, 0.8, 0.0,
        0.1, 0.8, 0.0,
        0, 1.0, 0.0
    ], dtype=np.float32)

    arrow_vao = gl.glGenVertexArrays(1)
    gl.glBindVertexArray(arrow_vao)

    arrow_vbo = vbo.VBO(vertices)
    arrow_vbo.bind()

    gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))
    gl.glEnableVertexAttribArray(0)

    gl.glBindVertexArray(0)
    arrow_vbo.unbind()

    return Model(arrow_vao, arrow_vbo, None, 9)


def crosshair_model() -> Model:
    vertices = []
    for i in range(64):
        angle = 6.28318530718 * float(i) / 63.0
        vertices.append([math.cos(angle), math.sin(angle)])

    circle_vao = gl.glGenVertexArrays(1)
    gl.glBindVertexArray(circle_vao)

    # Create and configure VBO
    circle_vbo = vbo.VBO(np.array(vertices, dtype=np.float32))
    circle_vbo.bind()

    # Set vertex attribute pointer
    gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))
    gl.glEnableVertexAttribArray(0)

    gl.glBindVertexArray(0)
    circle_vbo.unbind()

    cross_vertices = [
        [-1.25, 0, 0], [1.25, 0, 0],
        [0, -1.25, 0], [0, 1.25, 0]
    ]
    cross_vao = gl.glGenVertexArrays(1)
    gl.glBindVertexArray(cross_vao)

    cross_vbo = vbo.VBO(np.array(cross_vertices, dtype=np.float32))
    cross_vbo.bind()

    gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))
    gl.glEnableVertexAttribArray(0)

    gl.glBindVertexArray(0)
    cross_vbo.unbind()

    return Model2(circle_vao, cross_vao, circle_vbo, cross_vbo, len(vertices), len(cross_vertices))
