import OpenGL.GL as gl

import numpy as np
import glm

from .utils import create_shader_program, shader_path


class InstanceRenderer:
    def __init__(self, base_vertices, positions, colors=(1.0, 1.0, 1.0, 1.0), rotations=None, scales=None, shader_program=None):
        # Compile or assign shader program
        if shader_program is not None:
            self.prog = shader_program
        else:
            self.prog = create_shader_program(
                shader_path('instance', 'instance.vert'),
                shader_path('instance', 'instance.frag')
            )

        # Initialize transformations and counts
        self.set_transformations(positions, rotations, scales)
        self.instance_count = self.n

        # Reserve extra space to avoid frequent reallocations
        self._capacity = max(self.n * 2, self.n + 100)

        # Uniform locations
        self.p_loc = gl.glGetUniformLocation(self.prog, "projection")
        self.v_loc = gl.glGetUniformLocation(self.prog, "view")

        # Prepare color array (N x 4)
        cols = np.array(colors, dtype=np.float32)
        if cols.ndim == 1 and cols.size == 4:
            self.colors = np.tile(cols, (self.n, 1))
        elif cols.ndim == 2 and cols.shape == (self.n, 4):
            self.colors = cols
        else:
            raise ValueError("colors must be length‑4 tuple or an (N,4) array")

        # Generate and bind VAO
        self.vao = gl.glGenVertexArrays(1)
        gl.glBindVertexArray(self.vao)

        # Vertex VBO
        self.vertex_vbo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.vertex_vbo)
        verts = np.array(base_vertices, dtype=np.float32)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, verts.nbytes, verts, gl.GL_STATIC_DRAW)
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, False, 0, None)

        # Color VBO: allocate capacity and upload initial block
        self.color_vbo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.color_vbo)
        color_capacity_bytes = self._capacity * 4 * 4  # capacity * RGBA * 4 bytes
        gl.glBufferData(gl.GL_ARRAY_BUFFER, color_capacity_bytes, None, gl.GL_STATIC_DRAW)
        gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 0, self.colors.nbytes, self.colors)
        gl.glEnableVertexAttribArray(1)
        gl.glVertexAttribPointer(1, 4, gl.GL_FLOAT, False, 0, None)
        gl.glVertexAttribDivisor(1, 1)

        # Instance matrix VBO: allocate capacity
        self.instance_vbo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        mat_capacity_bytes = self._capacity * 16 * 4  # capacity * (4x4 floats) * 4 bytes
        gl.glBufferData(gl.GL_ARRAY_BUFFER, mat_capacity_bytes, None, gl.GL_DYNAMIC_DRAW)

        # Setup matrix attribute pointers
        stride = 16 * 4
        for i in range(4):
            loc = 2 + i
            offset = gl.ctypes.c_void_p(i * 16)
            gl.glEnableVertexAttribArray(loc)
            gl.glVertexAttribPointer(loc, 4, gl.GL_FLOAT, False, stride, offset)
            gl.glVertexAttribDivisor(loc, 1)

        # Unbind
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
        gl.glBindVertexArray(0)

        # Set vertex count and build initial matrices
        self.vertex_count = len(base_vertices) // 3
        self.set_model_matrices()

    def set_transformations(self, positions, rotations, scales):
        self.positions = np.array(positions, dtype=np.float32)
        self.n = len(self.positions)

        if rotations is None:
            self.rotations = None
        elif isinstance(rotations, (int, float)):
            self.rotations = np.full(self.n, rotations, dtype=np.float32)
        else:
            self.rotations = np.array(rotations, dtype=np.float32)

        if scales is None:
            self.scales = None
        elif isinstance(scales, (int, float)):
            self.scales = np.full(self.n, scales, dtype=np.float32)
        else:
            self.scales = np.array(scales, dtype=np.float32)

    def set_model_matrices(self):
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
                    m = glm.rotate(m, float(r), glm.vec3(0, 0, 1))
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
        m = glm.mat4(1.0)
        m = glm.translate(m, glm.vec3(*self.positions[index]))
        if self.rotations is not None:
            r = self.rotations[index]
            if hasattr(r, '__len__') and len(r) == 4:
                angle, x, y, z = r
                m = glm.rotate(m, float(angle), glm.vec3(x, y, z))
            else:
                m = glm.rotate(m, float(r), glm.vec3(0, 0, 1))
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
        if self.scales is None:
            self.scales = np.ones(self.instance_count, dtype=np.float32)
        self.scales[index] = scale
        self._update_instance_matrix(index)

    def rotate_instance(self, index: int, rotation):
        if self.rotations is None:
            self.rotations = np.zeros(self.instance_count, dtype=np.float32)
        self.rotations[index] = rotation
        self._update_instance_matrix(index)

    def translate_instance(self, index: int, x: float, y: float):
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

    def remove_instance(self, index: int):
        """Remove the instance at `index` and update GPU buffers."""
        if index < 0 or index >= self.instance_count:
            raise IndexError(f"instance index {index} out of range")

        self.positions = np.delete(self.positions, index, axis=0)
        if self.rotations is not None:
            self.rotations = np.delete(self.rotations, index, axis=0)
        if self.scales is not None:
            self.scales = np.delete(self.scales, index, axis=0)
        self.colors = np.delete(self.colors, index, axis=0)
        self.mats = np.delete(self.mats, index, axis=0)

        self.instance_count -= 1

        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.color_vbo)
        color_bytes = self.colors.nbytes
        gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 0, color_bytes, self.colors)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        mats_bytes = self.mats.nbytes
        gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 0, mats_bytes, self.mats.tobytes())
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    def add_instance(self, x, y, z, rot, scale, color):
        idx = self.instance_count

        self.positions = np.vstack([
            self.positions,
            np.array([x, y, z], dtype=np.float32)
        ])

        if self.rotations is not None and rot is not None:
            self.rotations = np.append(self.rotations, rot)

        if isinstance(scale, (int, float)):
            if self.scales is None:
                self.scales = np.array([scale], dtype=np.float32)
            elif self.scales.ndim == 1:
                self.scales = np.append(self.scales, float(scale))
            else:
                vec = np.array([scale, scale, scale], dtype=np.float32)
                self.scales = np.vstack([self.scales, vec])
        elif hasattr(scale, '__len__') and len(scale) == 3:
            vec = np.array(scale, dtype=np.float32)
            if self.scales is None:
                self.scales = vec[np.newaxis, :]
            elif self.scales.ndim == 1:
                old = np.repeat(self.scales[:, np.newaxis], 3, axis=1)
                self.scales = np.vstack([old, vec])
            else:
                self.scales = np.vstack([self.scales, vec])
        else:
            raise ValueError("scale must be a float or 3-tuple")

        c = np.array(color, dtype=np.float32)
        self.colors = np.vstack([self.colors, c])

        self.instance_count += 1
        self.mats = np.pad(self.mats, ((0, 1), (0, 0), (0, 0)), mode="constant", constant_values=0.0)
        self._update_instance_matrix(idx)

        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        byte_off = idx * 16 * 4
        mb = self.mats[idx].tobytes()
        gl.glBufferSubData(gl.GL_ARRAY_BUFFER, byte_off, len(mb), mb)

        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.color_vbo)
        co = c.tobytes()
        col_off = idx * 4 * 4
        gl.glBufferSubData(gl.GL_ARRAY_BUFFER, col_off, len(co), co)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    def transform_all(self, positions, rotations, scales=None):
        self.set_transformations(positions, rotations, scales)
        self.set_model_matrices()

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
