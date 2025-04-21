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
        self.base_verts = np.array([
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
        # 1) shader
        if shader_program is not None:
            self.prog = shader_program
        else:
            vert = shader_path('arrow', 'arrows.vert')
            frag = shader_path('arrow', 'arrows.frag')
            self.prog = create_shader_program(vert, frag)

        # 2) per-instance data
        self.starts = np.array(starts, dtype=np.float32)
        self.ends = np.array(ends, dtype=np.float32)
        self.edge_pairs = list(edge_pairs) if edge_pairs is not None else []
        self.thickness = thickness
        N = len(self.starts)

        # 3) colors
        cols = np.array(color, dtype=np.float32)
        if cols.ndim == 1 and cols.size == 4:
            self.colors = np.tile(cols, (N, 1))
        elif cols.ndim == 2 and cols.shape[0] == N and cols.shape[1] == 4:
            self.colors = cols
        else:
            raise ValueError("color must be (4,) or (N,4)")

        # 4) base arrow mesh
        self.vertex_count = len(self.base_verts) // 3

        # 5) build all instance mats
        self.mats = np.zeros((N, 4, 4), dtype=np.float32)
        for i in range(N):
            x0, y0 = self.starts[i]
            x1, y1 = self.ends[i]
            dx, dy = x1 - x0, y1 - y0
            length = np.hypot(dx, dy)
            # skip zero-length arrows
            if length < 1e-6:
                mat = glm.mat4(0.0)  # will effectively draw nothing
            else:
                theta = np.arctan2(dy, dx)
                angle = theta - np.pi / 2

                mat = glm.mat4(1.0)
                mat = glm.translate(mat, glm.vec3(x0, y0, 0.0))
                mat = glm.rotate(mat, float(angle), glm.vec3(0, 0, 1))
                mat = glm.scale(mat, glm.vec3(thickness, length, 1.0))
            self.mats[i] = np.array(mat.to_list(), dtype=np.float32)

        # 6) build VAO + VBOs exactly like InstanceRenderer
        self.vao = gl.glGenVertexArrays(1)
        gl.glBindVertexArray(self.vao)

        # base-vertex VBO
        self.vertex_vbo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.vertex_vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER,
                        self.base_verts.nbytes,
                        self.base_verts,
                        gl.GL_STATIC_DRAW)
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, False, 0, None)

        # color VBO (per-instance)
        self.color_vbo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.color_vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER,
                        self.colors.nbytes,
                        self.colors,
                        gl.GL_STATIC_DRAW)
        gl.glEnableVertexAttribArray(1)
        gl.glVertexAttribPointer(1, 4, gl.GL_FLOAT, False, 0, None)
        gl.glVertexAttribDivisor(1, 1)

        # instance‐matrix VBO
        self.instance_vbo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER,
                        self.mats.nbytes,
                        self.mats,
                        gl.GL_DYNAMIC_DRAW)
        stride = 16 * 4  # 4×4 floats × 4 bytes
        for col in range(4):
            loc = 2 + col
            offset = gl.ctypes.c_void_p(col * 16)
            gl.glEnableVertexAttribArray(loc)
            gl.glVertexAttribPointer(loc, 4, gl.GL_FLOAT, False, stride, offset)
            gl.glVertexAttribDivisor(loc, 1)

        # unbind
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
        gl.glBindVertexArray(0)

        self.instance_count = N

    def _rebuild_matrix(self, i):
        """Recompute mats[i] from self.starts[i], self.ends[i]."""
        x0, y0 = self.starts[i]
        x1, y1 = self.ends[i]
        dx, dy = x1 - x0, y1 - y0
        length = np.hypot(dx, dy)
        if length < 1e-6:
            mat = glm.mat4(0.0)
        else:
            theta = np.arctan2(dy, dx)
            angle = theta - np.pi / 2
            mat = glm.mat4(1.0)
            mat = glm.translate(mat, glm.vec3(x0, y0, 0))
            mat = glm.rotate(mat, float(angle), glm.vec3(0, 0, 1))
            mat = glm.scale(mat, glm.vec3(self.thickness, length, 1.0))
        self.mats[i] = np.array(mat.to_list(), dtype=np.float32)

    def update_for_node(self, node_id, new_gl_pos):
        """
        Called when one node moves: only adjust the arrows
        whose ui==node_id or vi==node_id.
        """
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        stride_bytes = 16 * 4  # 16 floats × 4 bytes

        for i, (ui, vi) in enumerate(self.edge_pairs):
            updated = False
            # if this arrow starts on the moved node, update its start
            if ui == node_id:
                self.starts[i] = (new_gl_pos[0], new_gl_pos[1])
                updated = True
            # if it ends on the moved node, update its end
            if vi == node_id:
                self.ends[i] = (new_gl_pos[0], new_gl_pos[1])
                updated = True

            if updated:
                # rebuild that one matrix
                self._rebuild_matrix(i)
                # push *only* its 64 bytes
                offset = i * stride_bytes
                # note: .tobytes() flattens the 4×4 float32 matrix
                gl.glBufferSubData(
                    gl.GL_ARRAY_BUFFER,
                    offset,
                    stride_bytes,
                    self.mats[i].tobytes()
                )

        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    def render(self, proj_mat, view_mat):
        gl.glUseProgram(self.prog)
        p_loc = gl.glGetUniformLocation(self.prog, "projection")
        v_loc = gl.glGetUniformLocation(self.prog, "view")
        gl.glUniformMatrix4fv(p_loc, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))
        gl.glUniformMatrix4fv(v_loc, 1, gl.GL_FALSE, glm.value_ptr(view_mat))

        # update instance matrices if you’ve changed starts/ends/thickness
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
