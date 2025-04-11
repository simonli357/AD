from OpenGL import GL as gl
from OpenGL.arrays import vbo

import numpy as np


class DetectionBox():
    def __init__(self, text_renderer, shader_program):
        self.text_renderer = text_renderer
        self.shader_program = shader_program

    def draw(self, x1, y1, x2, y2, label_text, line_width, color, proj_mat):
        vertices = np.array([
            [x1, y1],
            [x2, y1],
            [x2, y2],
            [x1, y2],
        ], dtype=np.float32)

        gl.glLineWidth(line_width)

        gl.glUseProgram(self.shader_program)

        proj_location = gl.glGetUniformLocation(self.shader_program, "projection")
        proj_array = np.array(proj_mat.to_list(), dtype=np.float32)
        gl.glUniformMatrix4fv(proj_location, 1, gl.GL_FALSE, proj_array)

        color_loc = gl.glGetUniformLocation(self.shader_program, "color")
        gl.glUniform4f(color_loc, *color, 1.0)

        box_vbo = vbo.VBO(vertices)
        box_vbo.bind()

        position_loc = gl.glGetAttribLocation(self.shader_program, "position")
        gl.glEnableVertexAttribArray(position_loc)
        gl.glVertexAttribPointer(position_loc, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, box_vbo)

        gl.glDrawArrays(gl.GL_LINE_LOOP, 0, 4)

        gl.glDisableVertexAttribArray(position_loc)
        box_vbo.unbind()
        box_vbo.delete()

        # --- Draw the label background with padding ---
        # Compute the text dimensions (without padding)
        text_width, text_height = self.text_renderer.compute_text_size(label_text, 1.0)
        # Define padding in pixels
        padding = 5.0

        # Calculate the padded rectangle coordinates.
        # Assuming (x1, y1) is the bottom-left of the detection box where the label touches it:
        bg_x1 = x1
        bg_y1 = y1
        bg_x2 = max(x1 + text_width + padding * 2, x2)
        bg_y2 = y1 - text_height - padding * 2

        bg_vertices = np.array([
            [bg_x1, bg_y1],
            [bg_x2, bg_y1],
            [bg_x2, bg_y2],
            [bg_x1, bg_y2],
        ], dtype=np.float32)

        # Set the uniform color to a semi-transparent color for the background.
        gl.glUniform4f(color_loc, *color, 0.5)

        bg_vbo = vbo.VBO(bg_vertices)
        bg_vbo.bind()
        gl.glEnableVertexAttribArray(position_loc)
        gl.glVertexAttribPointer(position_loc, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, bg_vbo)
        gl.glDrawArrays(gl.GL_TRIANGLE_FAN, 0, 4)
        gl.glDisableVertexAttribArray(position_loc)
        bg_vbo.unbind()
        bg_vbo.delete()

        # --- Draw the text label ---
        # Calculate the center of the padded background rectangle.
        text_center_x = bg_x1 + padding + text_width / 2
        text_center_y = (bg_y1 + bg_y2) / 2.0

        # The text renderer's render_text method centers the text around the specified coordinates.
        self.text_renderer.render_text(label_text, text_center_x, text_center_y, 1.0, color, proj_mat)

        gl.glUseProgram(0)
