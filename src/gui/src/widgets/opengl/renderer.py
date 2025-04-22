import OpenGL.GL as gl
import numpy as np
import glm

from .utils import create_shader_program, shader_path


class InstanceRenderer:
    def __init__(self, base_vertices, positions, colors=(1.0, 1.0, 1.0, 1.0), rotations=None, scales=None, shader_program=None):
        if shader_program is not None:
            self.prog = shader_program
        else:
            self.prog = create_shader_program(shader_path('instance', 'instance.vert'), shader_path('instance', 'instance.frag'))

        self.p_loc = gl.glGetUniformLocation(self.prog, "projection")
        self.v_loc = gl.glGetUniformLocation(self.prog, "view")

        self.positions = np.array(positions, dtype=np.float32)
        n = len(self.positions)

        cols = np.array(colors, dtype=np.float32)
        if cols.ndim == 1 and cols.size == 4:
            self.colors = np.tile(cols, (n, 1))
        elif cols.ndim == 2 and cols.shape[1] == 4 and cols.shape[0] == n:
            self.colors = cols
        else:
            raise ValueError("colors must be length‑4 tuple or an (N,4) array")

        if rotations is None:
            self.rotations = None
        elif isinstance(rotations, (int, float)):
            self.rotations = np.full(n, rotations, dtype=np.float32)
        else:
            self.rotations = np.array(rotations, dtype=np.float32)

        if scales is None:
            self.scales = None
        elif isinstance(scales, (int, float)):
            self.scales = np.full(n, scales, dtype=np.float32)
        else:
            self.scales = np.array(scales, dtype=np.float32)

        self.vao = gl.glGenVertexArrays(1)
        gl.glBindVertexArray(self.vao)

        self.vertex_vbo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.vertex_vbo)
        verts = np.array(base_vertices, dtype=np.float32)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, verts.nbytes, verts, gl.GL_STATIC_DRAW)
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, False, 0, None)

        self.color_vbo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.color_vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, self.colors.nbytes, self.colors, gl.GL_STATIC_DRAW)
        gl.glEnableVertexAttribArray(1)
        gl.glVertexAttribPointer(1, 4, gl.GL_FLOAT, False, 0, None)
        gl.glVertexAttribDivisor(1, 1)  # advance per-instance

        self.instance_vbo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        empty = np.zeros((n, 4, 4), dtype=np.float32)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, empty.nbytes, None, gl.GL_DYNAMIC_DRAW)

        stride = 4 * 4 * 4
        for i in range(4):
            loc = 2 + i
            offset = gl.ctypes.c_void_p(i * 16)
            gl.glEnableVertexAttribArray(loc)
            gl.glVertexAttribPointer(
                loc, 4, gl.GL_FLOAT, False, stride, offset
            )
            gl.glVertexAttribDivisor(loc, 1)

        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
        gl.glBindVertexArray(0)

        self.vertex_count = len(base_vertices) // 3
        self.instance_count = n

        self.mats = np.zeros((self.instance_count, 4, 4), dtype=np.float32)
        for i, pos in enumerate(self.positions):
            m = glm.mat4(1.0)
            m = glm.translate(m, glm.vec3(*pos))

            if self.rotations is not None:
                r = self.rotations[i]
                if hasattr(r, "__len__") and len(r) == 4:
                    angle, x, y, z = r
                    m = glm.rotate(m, float(angle), glm.vec3(x, y, z))
                else:
                    angle = float(r)
                    m = glm.rotate(m, angle, glm.vec3(0, 0, 1))
            if self.scales is not None:
                s = self.scales[i]
                if hasattr(s, "__len__") and len(s) == 3:
                    sx, sy, sz = s
                    m = glm.scale(m, glm.vec3(float(sx), float(sy), float(sz)))
                else:
                    u = float(s)
                    m = glm.scale(m, glm.vec3(u, u, u))
            self.mats[i] = np.array(m.to_list(), dtype=np.float32)

    def _update_instance_matrix(self, index: int):
        # rebuild the model matrix for a single instance
        pos = self.positions[index]
        m = glm.mat4(1.0)
        m = glm.translate(m, glm.vec3(*pos))
        # apply rotation
        if self.rotations is not None:
            r = self.rotations[index]
            if hasattr(r, '__len__') and len(r) == 4:
                angle, x, y, z = r
                m = glm.rotate(m, float(angle), glm.vec3(x, y, z))
            else:
                m = glm.rotate(m, float(r), glm.vec3(0, 0, 1))
        # apply scale
        if self.scales is not None:
            s = self.scales[index]
            if hasattr(s, '__len__') and len(s) == 3:
                sx, sy, sz = s
                m = glm.scale(m, glm.vec3(float(sx), float(sy), float(sz)))
            else:
                u = float(s)
                m = glm.scale(m, glm.vec3(u, u, u))
        self.mats[index] = np.array(m.to_list(), dtype=np.float32)

    def scale_instance(self, index: int, scale: float):
        # update the scale and rebuild the matrix
        if self.scales is None:
            self.scales = np.ones(self.instance_count, dtype=np.float32)
        self.scales[index] = scale
        self._update_instance_matrix(index)

    def rotate_instance(self, index: int, rotation):
        # update the rotation and rebuild
        if self.rotations is None:
            self.rotations = np.zeros(self.instance_count, dtype=np.float32)
        self.rotations[index] = rotation
        self._update_instance_matrix(index)

    def translate_instance(self, index: int, x: float, y: float):
        # update the position (keeping z if present) and rebuild
        z = float(self.positions[index][2]) if self.positions.shape[1] >= 3 else 0.0
        self.positions[index] = np.array([x, y, z], dtype=np.float32)
        self._update_instance_matrix(index)

    def color_instance(self, index: int, color):
        c = np.array(color, dtype=np.float32)
        self.colors[index] = c
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.color_vbo)
        offset = index * 4 * 4
        gl.glBufferSubData(gl.GL_ARRAY_BUFFER, offset, c.nbytes, c)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    def render(self, proj_mat, view_mat):
        gl.glUseProgram(self.prog)

        gl.glUniformMatrix4fv(self.p_loc, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))
        gl.glUniformMatrix4fv(self.v_loc, 1, gl.GL_FALSE, glm.value_ptr(view_mat))

        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 0, self.mats.nbytes, self.mats)

        gl.glBindVertexArray(self.vao)
        gl.glDrawArraysInstanced(
            gl.GL_TRIANGLES,
            0,
            self.vertex_count,
            self.instance_count
        )
        gl.glBindVertexArray(0)
        gl.glUseProgram(0)
