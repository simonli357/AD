from ..utils import create_shader_program, shader_path

import OpenGL.GL as gl
import numpy as np
import glm


class GTRenderer:
    """
    Instance renderer with incremental updates and hide-on-remove.
    """

    def __init__(self, model, max_instances=1024):
        self.model = model
        self.vertex_count = model.mesh.vertex_count

        # compile & cache shader
        self.shader_program = create_shader_program(shader_path('models', 'models.vert'), shader_path('models', 'models.frag'))
        self._u_proj = gl.glGetUniformLocation(self.shader_program, 'projection')
        self._u_view = gl.glGetUniformLocation(self.shader_program, 'view')
        self._u_hasTex = gl.glGetUniformLocation(self.shader_program, 'hasTexture')
        self._u_tex = gl.glGetUniformLocation(self.shader_program, 'uTexture')

        # instance bookkeeping
        self.capacity = max_instances
        self.id_to_index = {}    # id -> slot index
        self.current_count = 0   # number of assigned slots
        self._last_pose = {}     # id -> (x,y,yaw)

        hide_mat = glm.scale(glm.mat4(1.0), glm.vec3(0.0, 0.0, 0.0))
        self.hide_np = np.array(hide_mat, dtype=np.float32).T.flatten()

        # create & initialize VBO
        self.instance_vbo = gl.glGenBuffers(1)
        gl.glBindVertexArray(model.mesh.vao)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        gl.glBufferData(
            gl.GL_ARRAY_BUFFER,
            self.capacity * 16 * 4,
            None,
            gl.GL_DYNAMIC_DRAW
        )
        stride = 16 * 4
        for i in range(4):
            loc = 3 + i
            gl.glEnableVertexAttribArray(loc)
            gl.glVertexAttribPointer(
                loc, 4, gl.GL_FLOAT, gl.GL_FALSE,
                stride, gl.ctypes.c_void_p(i * 16)
            )
            gl.glVertexAttribDivisor(loc, 1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
        gl.glBindVertexArray(0)

    def _upload_matrix(self, index: int, mat_np: np.ndarray):
        """Upload one mat4 to VBO slot index."""
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        gl.glBufferSubData(
            gl.GL_ARRAY_BUFFER,
            index * 16 * 4,
            mat_np.nbytes,
            mat_np
        )
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    def set_ids(self, ids):
        """
        Hide any instances not in the provided ids set by moving them off-screen.
        ids: set of active instance IDs.
        """
        for inst_id, idx in self.id_to_index.items():
            if inst_id not in ids:
                idx = self.id_to_index.pop(inst_id)
                self._upload_matrix(idx, self.hide_np)
                del self._last_pose[inst_id]

    def add_or_update_instance(self, id, x, y, yaw, scale, extra_rot=False):
        """
        Assign or update an instance transform. Uses stable slots: once assigned, id retains index.
        """
        # skip if pose unchanged
        old = self._last_pose.get(id)
        if old and np.allclose((x, y, yaw), old):
            return
        # build transform matrix
        m = glm.mat4(1.0)
        m = glm.translate(m, glm.vec3(x, y, 0.0))
        m = glm.rotate(m, yaw, glm.vec3(0, 0, 1))
        if extra_rot:
            m = glm.rotate(m, glm.radians(180), glm.vec3(0, 0, 1))
            m = glm.rotate(m, glm.radians(90), glm.vec3(1, 0, 0))
        m = glm.scale(m, glm.vec3(*scale))
        mat_np = np.array(m, dtype=np.float32).T.flatten()

        # assign slot if new
        if id not in self.id_to_index:
            if self.current_count >= self.capacity:
                raise RuntimeError('Exceeded max_instances!')
            idx = self.current_count
            self.id_to_index[id] = idx
            self.current_count += 1
        else:
            idx = self.id_to_index[id]

        # upload matrix for this instance
        self._upload_matrix(idx, mat_np)
        self._last_pose[id] = (x, y, yaw)

    def draw(self, projection: glm.mat4, view: glm.mat4):
        """Draw all assigned slots (including hidden ones)."""
        if self.current_count == 0:
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
            gl.GL_TRIANGLES, 0, self.vertex_count,
            self.current_count
        )
        gl.glBindVertexArray(0)
        gl.glUseProgram(0)
