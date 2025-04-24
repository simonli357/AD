from OpenGL import GL as gl

import numpy as np
import glm


class CameraGrid:
    def __init__(self, shader_program):
        self.shader_program = shader_program
        # Uniform locations
        self.loc_uProj = gl.glGetUniformLocation(shader_program, "uProj")
        self.loc_uView = gl.glGetUniformLocation(shader_program, "uView")
        self.loc_uColor = gl.glGetUniformLocation(shader_program, "uColor")
        self.loc_cellSize = gl.glGetUniformLocation(shader_program, "cellSize")

        # Build a large XY-plane quad centered at origin
        e = 5
        verts = np.array([
            [-e, -e, 0.0],
            [e, -e, 0.0],
            [-e, e, 0.0],
            [e, e, 0.0],
        ], dtype=np.float32)

        # Setup VAO/VBO
        self.vao = gl.glGenVertexArrays(1)
        gl.glBindVertexArray(self.vao)

        self.vbo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, verts.nbytes, verts, gl.GL_STATIC_DRAW)

        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, 0, None)

        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
        gl.glBindVertexArray(0)

    def draw(self, proj_mat, view_mat, color=(1.0, 1.0, 1.0), cell_size=0.05):
        gl.glUseProgram(self.shader_program)

        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)

        gl.glUniformMatrix4fv(self.loc_uProj, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))
        gl.glUniformMatrix4fv(self.loc_uView, 1, gl.GL_FALSE, glm.value_ptr(view_mat))
        gl.glUniform3f(self.loc_uColor, *color)
        gl.glUniform1f(self.loc_cellSize, cell_size)

        gl.glBindVertexArray(self.vao)
        gl.glDrawArrays(gl.GL_TRIANGLE_STRIP, 0, 4)

        gl.glDisable(gl.GL_BLEND)
        gl.glBindVertexArray(0)
        gl.glUseProgram(0)
