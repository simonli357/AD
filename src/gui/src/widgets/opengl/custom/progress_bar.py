from OpenGL import GL as gl
from OpenGL.arrays import vbo

import numpy as np
import glm


class ProgressBar():
    """Custom progress bar that draws as a parallelogram instead of a rectangle."""

    def __init__(self, shader_program):
        super(ProgressBar, self).__init__()
        self.shader_program = shader_program
        # Backdrop (frame) color (white at 50% opacity)
        self.backdrop_color = (1.0, 1.0, 1.0, 0.5)
        # Shear factor for full bar (as a fraction of the total width)
        self.shear_fraction = 0.025  # Adjust this value for more or less slant

    def draw_I(self, x, y, width, height, fill_color, percentage, proj_mat):
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
