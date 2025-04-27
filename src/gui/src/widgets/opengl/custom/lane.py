from OpenGL import GL as gl
from OpenGL.arrays import vbo

import numpy as np
import glm


class LaneIndicator():
    def __init__(self, text_renderer, shader_program):
        self.text_renderer = text_renderer
        self.shader_program = shader_program

        # Generate and bind VAO.
        self.vao = gl.glGenVertexArrays(1)
        gl.glBindVertexArray(self.vao)

        # Normalized coordinates for a unit rectangle.
        self.vertices = np.array([
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0, 1.0],
            [0.0, 1.0],
        ], dtype=np.float32)

        # Create and bind VBO.
        self.line_vbo = vbo.VBO(self.vertices)
        self.line_vbo.bind()

        # Set vertex attribute pointer.
        gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))
        gl.glEnableVertexAttribArray(0)

        # Unbind VAO and VBO.
        gl.glBindVertexArray(0)
        self.line_vbo.unbind()

    def draw(self, x1, y1, x2, y2, divisions, color, proj_mat):
        # Create a model transform using glm.
        # We translate the unit rectangle to (x1, y1) and scale it to the desired size.
        model = glm.translate(glm.mat4(1.0), glm.vec3(x1, y1, 0.0)) * glm.scale(glm.mat4(1.0), glm.vec3(x2 - x1, y2 - y1, 1.0))

        # Combine the projection matrix (a glm.mat4) with the model transform.
        # Order matters: typically final_matrix = projection * model.
        final_matrix = proj_mat * model

        # Convert the glm matrix to a NumPy array of type float32.
        final_np = np.array(final_matrix.to_list(), dtype=np.float32)

        # Bind the shader program and update uniforms.
        gl.glUseProgram(self.shader_program)

        # Uniform: dash divisions.
        div_loc = gl.glGetUniformLocation(self.shader_program, "divisions")
        gl.glUniform1i(div_loc, divisions)

        # Uniform: color (red in this example).
        color_loc = gl.glGetUniformLocation(self.shader_program, "color")
        gl.glUniform4f(color_loc, *color, 0.9)

        # Uniforms for sine modulation.
        amplitude_loc = gl.glGetUniformLocation(self.shader_program, "amplitude")
        frequency_loc = gl.glGetUniformLocation(self.shader_program, "frequency")
        gl.glUniform1f(amplitude_loc, 0.04)   # tweak amplitude as needed
        gl.glUniform1f(frequency_loc, 100.0)      # tweak frequency as needed

        # Dash gap
        dash_ratio_loc = gl.glGetUniformLocation(self.shader_program, "dashRatio")
        gl.glUniform1f(dash_ratio_loc, 0.6)

        # Uniform: combined transformation matrix.
        proj_loc = gl.glGetUniformLocation(self.shader_program, "projection")
        # Upload the flattened final matrix.
        gl.glUniformMatrix4fv(proj_loc, 1, gl.GL_FALSE, final_np.flatten())

        # Bind the VAO and draw the rectangle.
        gl.glBindVertexArray(self.vao)
        gl.glDrawArrays(gl.GL_TRIANGLE_FAN, 0, 4)
