import OpenGL.GL as gl
import numpy as np
import glm

from ..utils import create_shader_program, shader_path


class ArrowInstanceRenderer:
    def __init__(
        self,
        starts: np.ndarray,
        ends: np.ndarray,
        color=(1.0, 0.0, 0.0, 1.0),
        thickness: float = 1.0,
        edge_pairs: list = None,
        shader_program=None,
    ):
        # 1) per-instance data
        self.starts = np.array(starts, dtype=np.float32)
        self.ends = np.array(ends, dtype=np.float32)
        self.edge_pairs = list(edge_pairs) if edge_pairs is not None else []
        self.thickness = thickness
        self.instance_count = len(self.starts)

        # 2) base arrow mesh (shaft + tip)
        self.base_verts = np.array([
            # Shaft (two rectangles)
            -0.04, 0.0, 0.0,
            -0.04, 1.0, 0.0,
            0.04, 1.0, 0.0,
            0.04, 0.0, 0.0,
            0.04, 1.0, 0.0,
            -0.04, 0.0, 0.0,
            # Tip (triangle)
            0.0, 0.2222, 0.0,
            -0.1666, -0.1111, 0.0,
            0.1666, -0.1111, 0.0
        ], dtype=np.float32)

        # Split into shaft and tip
        shaft_vert_count = 6
        tip_vert_count = 3
        self.shaft_verts = self.base_verts[: shaft_vert_count * 3]
        self.tip_verts = self.base_verts[shaft_vert_count * 3:]
        self.vertex_count_shaft = shaft_vert_count
        self.vertex_count_tip = tip_vert_count

        # 3) compute instance matrices
        self.shaft_mats = np.zeros((self.instance_count, 4, 4), dtype=np.float32)
        self.tip_mats = np.zeros((self.instance_count, 4, 4), dtype=np.float32)
        for i in range(self.instance_count):
            x0, y0 = self.starts[i]
            x1, y1 = self.ends[i]
            dx, dy = x1 - x0, y1 - y0
            length = np.hypot(dx, dy)
            theta = np.arctan2(dy, dx) - np.pi / 2

            arrow_height = (0.1666 + 0.25) * thickness

            # Shaft matrix: scale Y by length
            mat_sh = glm.mat4(1.0)
            mat_sh = glm.translate(mat_sh, glm.vec3(x0, y0, 0.0))
            mat_sh = glm.rotate(mat_sh, float(theta), glm.vec3(0, 0, 1))
            mat_sh = glm.scale(mat_sh, glm.vec3(thickness, length - arrow_height, 1.0))
            self.shaft_mats[i] = np.array(mat_sh.to_list(), dtype=np.float32)

            # Tip matrix: translate to end of shaft and scale uniformly by thickness
            mat_tp = glm.mat4(1.0)
            mat_tp = glm.translate(mat_tp, glm.vec3(x0, y0, 0.0))
            mat_tp = glm.rotate(mat_tp, float(theta), glm.vec3(0, 0, 1))
            # model-tip base
            mat_tp = glm.translate(mat_tp, glm.vec3(0.0, length - arrow_height, 0.0))
            mat_tp = glm.scale(mat_tp, glm.vec3(thickness, thickness, 1.0))
            self.tip_mats[i] = np.array(mat_tp.to_list(), dtype=np.float32)

        # 4) shader
        if shader_program is not None:
            self.prog = shader_program
        else:
            vert = shader_path('arrow', 'arrows.vert')
            frag = shader_path('arrow', 'arrows.frag')
            self.prog = create_shader_program(vert, frag)

        self.p_loc = gl.glGetUniformLocation(self.prog, "projection")
        self.v_loc = gl.glGetUniformLocation(self.prog, "view")

        # 5) color buffer (shared)
        cols = np.array(color, dtype=np.float32)
        if cols.ndim == 1 and cols.size == 4:
            self.colors = np.tile(cols, (self.instance_count, 1))
        elif cols.ndim == 2 and cols.shape == (self.instance_count, 4):
            self.colors = cols
        else:
            raise ValueError("color must be (4,) or (N,4)")

        # 6) buffers and VAOs
        self.instance_vbo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        max_bytes = max(self.shaft_mats.nbytes, self.tip_mats.nbytes)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, max_bytes, None, gl.GL_DYNAMIC_DRAW)

        self.color_vbo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.color_vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, self.colors.nbytes, self.colors, gl.GL_STATIC_DRAW)

        # Shaft VAO
        self.shaft_vao = gl.glGenVertexArrays(1)
        gl.glBindVertexArray(self.shaft_vao)
        self.shaft_vbo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.shaft_vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, self.shaft_verts.nbytes, self.shaft_verts, gl.GL_STATIC_DRAW)
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, False, 0, None)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.color_vbo)
        gl.glEnableVertexAttribArray(1)
        gl.glVertexAttribPointer(1, 4, gl.GL_FLOAT, False, 0, None)
        gl.glVertexAttribDivisor(1, 1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        stride = 16 * 4
        for col in range(4):
            loc = 2 + col
            offset = gl.ctypes.c_void_p(col * 16)
            gl.glEnableVertexAttribArray(loc)
            gl.glVertexAttribPointer(loc, 4, gl.GL_FLOAT, False, stride, offset)
            gl.glVertexAttribDivisor(loc, 1)
        gl.glBindVertexArray(0)

        # Tip VAO
        self.tip_vao = gl.glGenVertexArrays(1)
        gl.glBindVertexArray(self.tip_vao)
        self.tip_vbo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.tip_vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, self.tip_verts.nbytes, self.tip_verts, gl.GL_STATIC_DRAW)
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, False, 0, None)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.color_vbo)
        gl.glEnableVertexAttribArray(1)
        gl.glVertexAttribPointer(1, 4, gl.GL_FLOAT, False, 0, None)
        gl.glVertexAttribDivisor(1, 1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        for col in range(4):
            loc = 2 + col
            offset = gl.ctypes.c_void_p(col * 16)
            gl.glEnableVertexAttribArray(loc)
            gl.glVertexAttribPointer(loc, 4, gl.GL_FLOAT, False, stride, offset)
            gl.glVertexAttribDivisor(loc, 1)
        gl.glBindVertexArray(0)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    def _rebuild_matrix(self, i: int):
        x0, y0 = self.starts[i]
        x1, y1 = self.ends[i]
        dx, dy = x1 - x0, y1 - y0
        length = np.hypot(dx, dy)
        theta = np.arctan2(dy, dx) - np.pi / 2

        arrow_height = (0.1666 + 0.25) * self.thickness

        # Shaft
        mat_sh = glm.mat4(1.0)
        mat_sh = glm.translate(mat_sh, glm.vec3(x0, y0, 0.0))
        mat_sh = glm.rotate(mat_sh, float(theta), glm.vec3(0, 0, 1))
        mat_sh = glm.scale(mat_sh, glm.vec3(self.thickness, length - arrow_height, 1.0))
        self.shaft_mats[i] = np.array(mat_sh.to_list(), dtype=np.float32)

        # Tip
        mat_tp = glm.mat4(1.0)
        mat_tp = glm.translate(mat_tp, glm.vec3(x0, y0, 0.0))
        mat_tp = glm.rotate(mat_tp, float(theta), glm.vec3(0, 0, 1))
        mat_tp = glm.translate(mat_tp, glm.vec3(0.0, length - arrow_height, 0.0))
        mat_tp = glm.scale(mat_tp, glm.vec3(self.thickness, self.thickness, 1.0))
        self.tip_mats[i] = np.array(mat_tp.to_list(), dtype=np.float32)

    def update_for_node(self, node_id: int, new_gl_pos):
        """Called when a node moves: updates the corresponding arrow instances."""
        for i, (ui, vi) in enumerate(self.edge_pairs):
            updated = False
            if ui == node_id:
                self.starts[i] = (new_gl_pos[0], new_gl_pos[1])
                updated = True
            if vi == node_id:
                self.ends[i] = (new_gl_pos[0], new_gl_pos[1])
                updated = True
            if updated:
                # recompute both shaft and tip matrices for this instance
                self._rebuild_matrix(i)

    def reset(self, starts: list[tuple[float, float]], ends: list[tuple[float, float]], edge_pairs: list[tuple[int, int]]):
        """
        Rebuild all per-instance data in place, including colors, and
        re-upload via glBufferData/SubData.
        """

        self.starts = np.array(starts, dtype=np.float32)
        self.ends = np.array(ends, dtype=np.float32)
        self.edge_pairs = list(edge_pairs)
        self.instance_count = len(self.starts)

        base_col = self.colors[0].copy()
        self.colors = np.tile(base_col, (self.instance_count, 1)).astype(np.float32)

        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.color_vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, self.colors.nbytes, self.colors, gl.GL_STATIC_DRAW)

        self.shaft_mats = np.zeros((self.instance_count, 4, 4), dtype=np.float32)
        self.tip_mats = np.zeros((self.instance_count, 4, 4), dtype=np.float32)
        for i in range(self.instance_count):
            self._rebuild_matrix(i)

        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        size_bytes = max(self.shaft_mats.nbytes, self.tip_mats.nbytes)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, size_bytes, None, gl.GL_DYNAMIC_DRAW)
        gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 0, self.shaft_mats.nbytes, self.shaft_mats.tobytes())
        gl.glBufferSubData(gl.GL_ARRAY_BUFFER, size_bytes - self.tip_mats.nbytes, self.tip_mats.nbytes, self.tip_mats.tobytes())
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    def render(self, proj_mat, view_mat):
        gl.glUseProgram(self.prog)
        gl.glUniformMatrix4fv(self.p_loc, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))
        gl.glUniformMatrix4fv(self.v_loc, 1, gl.GL_FALSE, glm.value_ptr(view_mat))
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 0, self.shaft_mats.nbytes, self.shaft_mats.tobytes())
        gl.glBindVertexArray(self.shaft_vao)
        gl.glDrawArraysInstanced(gl.GL_TRIANGLES, 0, self.vertex_count_shaft, self.instance_count)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 0, self.tip_mats.nbytes, self.tip_mats.tobytes())
        gl.glBindVertexArray(self.tip_vao)
        gl.glDrawArraysInstanced(gl.GL_TRIANGLES, 0, self.vertex_count_tip, self.instance_count)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
