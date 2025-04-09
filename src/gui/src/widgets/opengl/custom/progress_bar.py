from OpenGL import GL as gl
from OpenGL.arrays import vbo
from PyQt5.Qt import QPainter, QFont, QColor

import numpy as np
import glm


class ProgressBar():
    """Custom progress bar"""

    def __init__(self, shader_program):
        super(ProgressBar, self).__init__()
        self.shader_program = shader_program
        self.backdrop_color = (1.0, 1.0, 1.0, 0.5)
        self.shear_fraction = 0.025  # Adjust this value for more or less slant

    def draw(self, qpainter, screen_width, screen_height, x_norm, y_norm, width_norm, height_norm, fill_color, percentage, proj_mat):
        """
        Draws a progress bar where (x,y) is the top-right corner.
        The bar is drawn as a parallelogram with a shear applied to the bottom edge.
        The filled portion is drawn based on 'percentage' and its shear is scaled.

        Coordinates are assumed to be in normalized (or projected) space,
        and proj_mat is the projection matrix.

        :param x: x-coordinate of the top-right corner.
        :param y: y-coordinate of the top-right corner.
        :param width: Full width of the progress bar.
        :param height: Height of the progress bar.
        :param fill_color: 4-tuple (r, g, b, a) for the fill.
        :param percentage: Progress fraction between 0.0 and 1.0.
        :param proj_mat: Projection matrix (e.g. glm.ortho).
        """
        # Denormalize coords
        x = x_norm * screen_width
        y = y_norm * screen_height
        width = width_norm * screen_width
        height = height_norm * screen_height

        # Calculate the shear offset for the full bar.
        s = self.shear_fraction * width

        # --- BACKDROP (full bar) vertices as a parallelogram ---
        # Top edge remains horizontal, anchored at (x,y):
        # Top-right: (x, y)
        # Top-left:  (x - width, y)
        # Bottom edge is shifted right by s:
        # Bottom-right: (x + s, y - height)
        # Bottom-left:  (x - width + s, y - height)
        backdrop_vertices = np.array([
            # First triangle:
            x - width, y,           # Top-left
            x, y,           # Top-right
            x + s, y - height,  # Bottom-right
            # Second triangle:
            x - width, y,           # Top-left
            x + s, y - height,  # Bottom-right
            x - width + s, y - height  # Bottom-left
        ], dtype=np.float32)

        # --- FILLED BAR vertices ---
        fill_width = percentage * width
        # Scale shear for the filled area, so the slant remains proportional.
        fill_s = s * (fill_width / width)

        filled_vertices = np.array([
            # First triangle
            x - width, y,            # Fill Top-left
            x - width + fill_width, y,            # Fill Top-right
            x - width + fill_width + fill_s, y - height,  # Fill Bottom-right
            # Second triangle
            x - width, y,            # Fill Top-left
            x - width + fill_width + fill_s, y - height,  # Fill Bottom-right
            x - width + s, y - height    # Fill Bottom-left
        ], dtype=np.float32)

        # --- Draw using the shader ---
        gl.glUseProgram(self.shader_program)

        # Set the projection matrix uniform "uProjection" in your shader.
        loc_proj = gl.glGetUniformLocation(self.shader_program, "uProjection")
        gl.glUniformMatrix4fv(loc_proj, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))

        # Get the uniform for the color "uColor".
        loc_color = gl.glGetUniformLocation(self.shader_program, "uColor")

        # Draw Backdrop:
        backdrop_vbo = vbo.VBO(backdrop_vertices)
        backdrop_vbo.bind()
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))
        gl.glUniform4f(loc_color, *self.backdrop_color)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, 6)
        backdrop_vbo.unbind()
        backdrop_vbo.delete()

        # Draw Filled Portion:
        fill_vbo = vbo.VBO(filled_vertices)
        fill_vbo.bind()
        gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))
        gl.glUniform4f(loc_color, *fill_color)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, 6)
        fill_vbo.unbind()
        fill_vbo.delete()

        gl.glDisableVertexAttribArray(0)
        gl.glUseProgram(0)

        # --- Render percentage text on the left of the progress bar ---
        percentage_value = int(percentage * 100)
        text_to_render = f" {percentage_value}%"
        margin = len(text_to_render) * 0.0105 * screen_width
        text_x = (x - width) - margin
        text_y = y + (height * 0.25)
        self.render_text(qpainter, text_to_render, 20, (0, 255, 0, 255), text_x, text_y)

    def render_text(self, painter: QPainter, text, size, color: (int, int, int, int), x, y) -> None:
        # Get current OpenGL color
        gl_color = gl.glGetDoublev(gl.GL_CURRENT_COLOR)
        text_color = QColor(
            int(gl_color[2] * color[2]),
            int(gl_color[1] * color[1]),
            int(gl_color[0] * color[0]),
            int(gl_color[3] * color[3])
        )

        # Set up font
        font = QFont()
        font.setBold(True)
        font.setStyleStrategy(QFont.PreferAntialias)

        painter.setPen(text_color)
        painter.setFont(font)
        painter.drawText(int(x), int(y), text)
