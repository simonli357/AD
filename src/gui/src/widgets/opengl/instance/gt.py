from ..utils import create_shader_program, shader_path

import OpenGL.GL as gl
import numpy as np
import glm


class GTRenderer:
    def __init__(self, model, max_instances=1024):
        self.model = model
        self.vertex_count = model.mesh.vertex_count

        # --- (same shader setup as before) ---
        self.shader_program = create_shader_program(shader_path('models', 'models.vert'), shader_path('models', 'models.frag'))
        self._u_proj = gl.glGetUniformLocation(self.shader_program, "projection")
        self._u_view = gl.glGetUniformLocation(self.shader_program, "view")
        self._u_hasTex = gl.glGetUniformLocation(self.shader_program, "hasTexture")
        self._u_tex = gl.glGetUniformLocation(self.shader_program, "uTexture")

        # pre-allocate instance buffer
        self.stride = 16 * 4  # bytes per mat4
        self.capacity = max_instances
        self.num_instances = 0
        self.id_to_index = {}      # maps your id → slot index
        self._last_pose = {}

        # create + bind VBO
        self.instance_vbo = gl.glGenBuffers(1)
        gl.glBindVertexArray(model.mesh.vao)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        # allocate ‘empty’ storage up to capacity
        gl.glBufferData(gl.GL_ARRAY_BUFFER,
                        self.capacity * self.stride,
                        None,
                        gl.GL_DYNAMIC_DRAW)

        # set up the 4 matrix attributes (locations 3,4,5,6)
        for i in range(4):
            loc = 3 + i
            gl.glEnableVertexAttribArray(loc)
            gl.glVertexAttribPointer(
                loc, 4, gl.GL_FLOAT, gl.GL_FALSE,
                self.stride, gl.ctypes.c_void_p(i * 16)
            )
            gl.glVertexAttribDivisor(loc, 1)

        # unbind
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
        gl.glBindVertexArray(0)

    def _upload_matrix(self, index: int, mat_np: np.ndarray):
        """Update just one instance’s 16-float matrix at slot ‘index’."""
        offset = index * self.stride
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        gl.glBufferSubData(
            gl.GL_ARRAY_BUFFER,
            offset,
            mat_np.nbytes,
            mat_np
        )
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    def add_or_update_instance(self, id, x, y, yaw, scale, extra_rot=False):
        old = self._last_pose.get(id)
        if old is not None:
            old_x, old_y, old_yaw = old
            if (np.isclose(x, old_x) and np.isclose(y, old_y) and np.isclose(yaw, old_yaw)):
                return

        m = glm.mat4(1.0)
        m = glm.translate(m, glm.vec3(x, y, 0.0))
        m = glm.rotate(m, yaw, glm.vec3(0, 0, 1))
        if extra_rot:
            m = glm.rotate(m, np.radians(180), glm.vec3(0.0, 0.0, 1.0))
            m = glm.rotate(m, np.radians(90), glm.vec3(1.0, 0.0, 0.0))
        m = glm.scale(m, glm.vec3(*scale))
        mat_np = np.array(m, dtype=np.float32).T.flatten()

        if id in self.id_to_index:
            idx = self.id_to_index[id]
        else:
            idx = self.num_instances
            if idx >= self.capacity:
                raise RuntimeError("Exceeded max_instances!")
            self.id_to_index[id] = idx
            self.num_instances += 1

        self._upload_matrix(idx, mat_np)

        self._last_pose[id] = (x, y, yaw)

    def draw(self, projection: glm.mat4, view: glm.mat4):
        if self.num_instances == 0:
            return
        gl.glUseProgram(self.shader_program)
        gl.glUniformMatrix4fv(self._u_proj, 1, gl.GL_FALSE, glm.value_ptr(projection))
        gl.glUniformMatrix4fv(self._u_view, 1, gl.GL_FALSE, glm.value_ptr(view))

        if self.model.texture:
            gl.glUniform1i(self._u_hasTex, 1)
            gl.glActiveTexture(gl.GL_TEXTURE0)
            gl.glBindTexture(gl.GL_TEXTURE_2D, self.model.texture)
            gl.glUniform1i(self._u_tex, 0)
        else:
            gl.glUniform1i(self._u_hasTex, 0)

        gl.glBindVertexArray(self.model.mesh.vao)
        gl.glDrawArraysInstanced(
            gl.GL_TRIANGLES,
            0,
            self.vertex_count,
            self.num_instances
        )
        gl.glBindVertexArray(0)
        gl.glUseProgram(0)
