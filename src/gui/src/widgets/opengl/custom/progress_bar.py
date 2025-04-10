import numpy as np
from OpenGL import GL as gl
from OpenGL.arrays import vbo
import glm


class ProgressBar():
    """Custom progress bar drawing a parallelogram with a shear effect.

    This refactored version separates the static geometry calculations from the dynamic
    parts, making it easier to maintain and cache if needed.
    """

    def __init__(self, shader_program):
        self.shader_program = shader_program
        self.backdrop_color = (1.0, 1.0, 1.0, 0.5)
        # This shear_fraction determines how much the bottom edge is slanted.
        self.shear_fraction = 0.025

        # Optionally, if the geometry rarely changes, you can cache it:
        self.cached_params = None  # e.g., a dict to store x, y, width, height, shear, etc.
        self.cached_backdrop_vertices = None

    def compute_geometry(self, screen_width, screen_height, x_norm, y_norm, width_norm, height_norm):
        """
        Compute the static geometry of the progress bar (backdrop) using the normalized position
        and dimensions. Returns a tuple (x, y, width, height, s, backdrop_vertices)
        where:
          - (x, y) are the absolute top-right coordinates,
          - width and height in pixels,
          - s is the shear offset (in pixels),
          - backdrop_vertices is a NumPy array of 6 vertices (for two triangles).
        """
        # Denormalize coordinates.
        x = x_norm * screen_width
        y = y_norm * screen_height
        width = width_norm * screen_width
        height = height_norm * screen_height

        # Compute shear offset (applied on the bottom edge).
        s = self.shear_fraction * width

        # Create vertices for a parallelogram as two triangles.
        # Coordinates:
        #   Top-right: (x, y)
        #   Top-left:  (x - width, y)
        #   Bottom-right: (x + s, y - height)
        #   Bottom-left:  (x - width + s, y - height)
        backdrop_vertices = np.array([
            # First triangle:
            x - width, y,              # Top-left
            x, y,                      # Top-right
            x + s, y - height,         # Bottom-right
            # Second triangle:
            x - width, y,              # Top-left
            x + s, y - height,         # Bottom-right
            x - width + s, y - height  # Bottom-left
        ], dtype=np.float32)

        return x, y, width, height, s, backdrop_vertices

    def draw(self, screen_width, screen_height, x_norm, y_norm, width_norm, height_norm,
             fill_color, percentage, proj_mat):
        """
        Draws the progress bar. (x,y) is the top-right corner in normalized space.
        The bar is rendered as a sheared parallelogram with a filled portion determined
        by 'percentage'. The projection matrix is used by the shader.
        """
        # You could cache these if the parameters don't change between frames.
        x, y, width, height, s, backdrop_vertices = self.compute_geometry(screen_width, screen_height,
                                                                          x_norm, y_norm, width_norm, height_norm)

        # Compute the filled portion geometry based on the current percentage.
        fill_width = percentage * width
        # Scale the shear for the filled portion in proportion to the fill width.
        fill_s = s * (fill_width / width)
        filled_vertices = np.array([
            # First triangle:
            x - width, y,                           # Fill Top-left
            x - width + fill_width, y,                # Fill Top-right
            x - width + fill_width + fill_s, y - height,  # Fill Bottom-right
            # Second triangle:
            x - width, y,                           # Fill Top-left
            x - width + fill_width + fill_s, y - height,  # Fill Bottom-right
            x - width + s, y - height                 # Fill Bottom-left
        ], dtype=np.float32)

        # --- Draw using the shader ---
        gl.glUseProgram(self.shader_program)
        # Set projection matrix.
        loc_proj = gl.glGetUniformLocation(self.shader_program, "uProjection")
        gl.glUniformMatrix4fv(loc_proj, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))

        # Get the uniform for the color.
        loc_color = gl.glGetUniformLocation(self.shader_program, "uColor")

        # Draw the backdrop.
        backdrop_vbo = vbo.VBO(backdrop_vertices)
        backdrop_vbo.bind()
        gl.glEnableVertexAttribArray(0)
        # We pass None for offset if we're using a bound VBO.
        gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, None)
        gl.glUniform4f(loc_color, *self.backdrop_color)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, 6)
        backdrop_vbo.unbind()
        backdrop_vbo.delete()

        # Draw the filled portion.
        fill_vbo = vbo.VBO(filled_vertices)
        fill_vbo.bind()
        gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, None)
        gl.glUniform4f(loc_color, *fill_color)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, 6)
        fill_vbo.unbind()
        fill_vbo.delete()

        gl.glDisableVertexAttribArray(0)
        gl.glUseProgram(0)
