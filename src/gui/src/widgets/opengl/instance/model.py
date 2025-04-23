from ..utils import create_shader_program, shader_path
from typing import Tuple, List

import OpenGL.GL as gl
import numpy as np
import glm


class ModelInstanceRenderer:
    """
    Instance renderer for any road-object model. Queues up per-instance transforms
    and renders them all with one instanced draw call.
    """

    def __init__(self, model):
        self.model = model
        self.num_instances = 0

        # Base mesh VAO and vertex count
        self.vao = model.mesh.vao
        self.vertex_count = model.mesh.vertex_count

        # Load instanced shader
        self.shader_program = create_shader_program(shader_path('models', 'models.vert'), shader_path('models', 'models.frag'))

        # Create and configure instance VBO
        self.instance_vbo = gl.glGenBuffers(1)
        gl.glBindVertexArray(self.vao)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, 0, None, gl.GL_DYNAMIC_DRAW)

        # A 4x4 matrix = 16 floats; we use attrib locations 3,4,5,6
        stride = 16 * 4
        for i in range(4):
            loc = 3 + i
            gl.glEnableVertexAttribArray(loc)
            gl.glVertexAttribPointer(loc, 4, gl.GL_FLOAT, gl.GL_FALSE, stride, gl.ctypes.c_void_p(i * 16))
            gl.glVertexAttribDivisor(loc, 1)

        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
        gl.glBindVertexArray(0)

        # Cache uniform locations
        self._u_proj = gl.glGetUniformLocation(self.shader_program, "projection")
        self._u_view = gl.glGetUniformLocation(self.shader_program, "view")
        self._u_hasTex = gl.glGetUniformLocation(self.shader_program, "hasTexture")
        self._u_tex = gl.glGetUniformLocation(self.shader_program, "uTexture")

        # Buffer for new instance matrices each frame
        self._pending_matrices: List[np.ndarray] = []

    def clear_instances(self) -> None:
        """Clear all queued instances (call at start of frame)."""
        self._pending_matrices.clear()

    def add_instance(self, x: float, y: float, orientation: float, scale: Tuple[float, float, float]) -> None:
        """Queue one instance transform for (x,y), yaw=orientation, and XYZ scale."""
        m = glm.mat4(1.0)
        m = glm.translate(m, glm.vec3(x, y, 0.0))
        m = glm.rotate(m, orientation, glm.vec3(0.0, 0.0, 1.0))
        m = glm.scale(m, glm.vec3(*scale))
        mat_np = np.array(m, dtype=np.float32).T.flatten()
        self._pending_matrices.append(mat_np)

    def upload_instances(self) -> None:
        """Upload all queued instance matrices to the GPU buffer."""
        if not self._pending_matrices:
            self.num_instances = 0
            return

        arr = np.vstack(self._pending_matrices).astype(np.float32)
        self.num_instances = arr.shape[0]

        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, arr.nbytes, arr, gl.GL_DYNAMIC_DRAW)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    def draw(self, projection: glm.mat4, view: glm.mat4) -> None:
        """Render all queued instances in a single instanced draw call."""
        if self.num_instances == 0:
            return

        gl.glUseProgram(self.shader_program)
        gl.glBindVertexArray(self.vao)

        # Upload common uniforms
        gl.glUniformMatrix4fv(self._u_proj, 1, gl.GL_FALSE, glm.value_ptr(projection))
        gl.glUniformMatrix4fv(self._u_view, 1, gl.GL_FALSE, glm.value_ptr(view))

        # Bind texture if present
        if self.model.texture:
            gl.glUniform1i(self._u_hasTex, 1)
            gl.glActiveTexture(gl.GL_TEXTURE0)
            gl.glBindTexture(gl.GL_TEXTURE_2D, self.model.texture)
            gl.glUniform1i(self._u_tex, 0)
        else:
            gl.glUniform1i(self._u_hasTex, 0)

        # Draw instances
        gl.glDrawArraysInstanced(gl.GL_TRIANGLES, 0, self.vertex_count, self.num_instances)

        gl.glBindVertexArray(0)
        gl.glUseProgram(0)
