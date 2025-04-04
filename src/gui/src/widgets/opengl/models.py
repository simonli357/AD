from OpenGL import GL as gl
from OpenGL.arrays import vbo
from collections import namedtuple

import numpy as np
import math

Model = namedtuple('Model', ['vao', 'vbo', 'ebo', 'vertex_count'])


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
    gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, line_vbo)
    gl.glEnableVertexAttribArray(0)

    gl.glBindVertexArray(0)
    line_vbo.unbind()

    return Model(vao, line_vbo, None, 2)


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
    gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, circle_vbo)
    gl.glEnableVertexAttribArray(0)

    gl.glBindVertexArray(0)
    circle_vbo.unbind()

    return Model(vao, circle_vbo, None, len(vertices))
