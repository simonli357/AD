from OpenGL import GL as gl
from OpenGL.arrays import vbo
from collections import namedtuple
import numpy as np

Model = namedtuple('Model', ['vao', 'vbo', 'ebo', 'vertex_count'])


def line_model() -> Model:
    vao = gl.glGenVertexArrays(1)
    gl.glBindVertexArray(vao)

    # Proper line vertices (two points with x,y coordinates)
    vertex_array = np.array([
        [0.0, 0.0],  # Start point
        [1.0, 1.0]    # End point
    ], dtype=np.float32).flatten()

    # Create and configure VBO
    line_vbo = vbo.VBO(vertex_array)
    line_vbo.bind()

    # Set vertex attribute pointer
    gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, line_vbo)
    gl.glEnableVertexAttribArray(0)

    gl.glBindVertexArray(0)
    line_vbo.unbind()

    return Model(vao, line_vbo, None, 2)
