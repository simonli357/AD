from OpenGL import GL as gl
from OpenGL.arrays import vbo
from collections import namedtuple

import numpy as np

Model = namedtuple('Model', ['vao', 'vbo', 'ebo', 'vertex_count'])


def line_model() -> Model:
    vertex_array = np.array([
        [0.0, 0.0, 0.0],  # Start point
        [1.0, 1.0, 0.0]    # End point
    ], dtype=np.float32).flatten()

    vao = gl.glGenVertexArrays(1)
    gl.glBindVertexArray(vao)

    # Create and configure VBO
    line_vbo = vbo.VBO(vertex_array)
    line_vbo.bind()

    # Set vertex attribute pointer
    gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))
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
    vertices = [[0.0, 0.0]]  # center vertex
    for i in range(65):
        angle = 6.28318530718 * float(i) / 64.0
        vertices.append([np.cos(angle), np.sin(angle)])

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


def diamond_model() -> Model:
    vertices = np.array([
        # Positions (3D for proper matrix transformations)
        [0.0, 0.5, 0.1],  # Top
        [0.5, 0.0, 0.1],  # Right
        [-0.5, 0.0, 0.1],  # Left
        [-0.5, 0.0, 0.1],  # Left
        [0.5, 0.0, 0.1],  # Right
        [0.0, -0.5, 0.1],  # Bottom
    ], dtype=np.float32).flatten()

    vao = gl.glGenVertexArrays(1)
    gl.glBindVertexArray(vao)

    # Create and configure VBO
    diamond_vbo = vbo.VBO(vertices)
    diamond_vbo.bind()

    # Set vertex attribute pointer
    gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))
    gl.glEnableVertexAttribArray(0)

    gl.glBindVertexArray(0)
    diamond_vbo.unbind()

    return Model(vao, diamond_vbo, None, len(vertices))


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


def crosshair_model(thickness=0.075, inner=0.25, outer=1.25) -> Model:
    """
      - thickness: half‑width of each arm, in object‑space units
      - inner:   the start‑offset of each arm from center
      - outer:   the end‑offset of each arm from center
    """
    t = thickness
    i = inner
    o = outer

    # 4 strips, each 4 verts (x,y,z):
    verts = [
        # ── left horizontal arm ───────────────────
        -o, t, 0.0,
        -o, -t, 0.0,
        -i, t, 0.0,
        -i, -t, 0.0,

        # ── right horizontal arm ──────────────────
        i, t, 0.0,
        i, -t, 0.0,
        o, t, 0.0,
        o, -t, 0.0,

        # ── bottom vertical arm ───────────────────
        -t, -o, 0.0,
        t, -o, 0.0,
        -t, -i, 0.0,
        t, -i, 0.0,

        # ── top vertical arm ──────────────────────
        -t, i, 0.0,
        t, i, 0.0,
        -t, o, 0.0,
        t, o, 0.0,
    ]
    data = np.array(verts, dtype=np.float32)

    vao = gl.glGenVertexArrays(1)
    gl.glBindVertexArray(vao)

    cross_vbo = vbo.VBO(data)
    cross_vbo.bind()

    gl.glEnableVertexAttribArray(0)
    gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))

    gl.glBindVertexArray(0)
    cross_vbo.unbind()

    # total vertices = 16
    return Model(vao, cross_vbo, None, 16)


def quad_model() -> Model:
    verts = [
        [-1, -1, 0, 0, 0],
        [1, -1, 0, 1, 0],
        [-1, 1, 0, 0, 1],
        [1, 1, 0, 1, 1],
    ]
    vao = gl.glGenVertexArrays(1)
    gl.glBindVertexArray(vao)

    quad_vbo = vbo.VBO(np.array(verts, dtype=np.float32))
    quad_vbo.bind()

    # pos @ loc=0, uv @ loc=1
    stride = 5 * 4
    gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, False, stride, gl.ctypes.c_void_p(0))
    gl.glEnableVertexAttribArray(0)
    gl.glVertexAttribPointer(1, 2, gl.GL_FLOAT, False, stride, gl.ctypes.c_void_p(12))
    gl.glEnableVertexAttribArray(1)

    gl.glBindVertexArray(0)
    quad_vbo.unbind()

    return Model(vao, quad_vbo, None, 4)
