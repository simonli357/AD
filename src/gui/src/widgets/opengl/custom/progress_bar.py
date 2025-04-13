import numpy as np
from OpenGL import GL as gl
from OpenGL.arrays import vbo
import glm


class ProgressBar():
    """Custom progress bar drawing a parallelogram with a shear effect."""

    def __init__(self, text_renderer, shader_program, texture_shader):
        self.text_renderer = text_renderer
        self.shader_program = shader_program
        self.texture_shader = texture_shader
        self.backdrop_color = (1.0, 1.0, 1.0, 0.5)
        # This shear_fraction determines how much the edges are slanted.
        self.shear_fraction = 0.025
        self.cached_width = None
        self.cached_height = None

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
        self.x = x_norm * screen_width
        self.y = y_norm * screen_height
        self.width = width_norm * screen_width
        self.height = height_norm * screen_height

        # Compute shear offset (applied on the bottom edge).
        self.s = self.shear_fraction * self.width

        # Create vertices for a parallelogram as two triangles.
        # Coordinates:
        #   Top-right: (x, y)
        #   Top-left:  (x - width, y)
        #   Bottom-right: (x + s, y - height)
        #   Bottom-left:  (x - width + s, y - height)
        self.backdrop_vertices = np.array([
            # First triangle:
            self.x - self.width, self.y,
            self.x, self.y,
            self.x + self.s, self.y - self.height,
            # Second triangle:
            self.x - self.width, self.y,
            self.x + self.s, self.y - self.height,
            self.x - self.width + self.s, self.y - self.height
        ], dtype=np.float32)

        if hasattr(self, 'backdrop_vbo'):
            self.backdrop_vbo.delete()
        self.backdrop_vbo = vbo.VBO(self.backdrop_vertices)

    def draw_texture(self, x, y, icon, scale, proj_matrix):
        gl.glUseProgram(self.texture_shader)

        # Set matrices
        model = glm.mat4(1.0)
        model = glm.translate(model, glm.vec3(x, y, 0))
        model = glm.scale(model, glm.vec3(scale, scale, 1.0))
        model = glm.rotate(model, np.radians(180), glm.vec3(0, 0, 1))

        gl.glUniformMatrix4fv(
            gl.glGetUniformLocation(self.texture_shader, "model"),
            1, gl.GL_FALSE, glm.value_ptr(model)
        )
        gl.glUniformMatrix4fv(
            gl.glGetUniformLocation(self.texture_shader, "projection"),
            1, gl.GL_FALSE, glm.value_ptr(proj_matrix)
        )

        # Bind texture
        gl.glActiveTexture(gl.GL_TEXTURE0)
        gl.glBindTexture(gl.GL_TEXTURE_2D, icon.texture_id)
        gl.glUniform1i(gl.glGetUniformLocation(self.texture_shader, "texture1"), 0)

        # Draw
        gl.glBindVertexArray(icon.vao)
        gl.glDrawElements(gl.GL_TRIANGLES, 6, gl.GL_UNSIGNED_INT, None)
        gl.glBindVertexArray(0)

    def draw(self, screen_width, screen_height, x_norm, y_norm, width_norm, height_norm, fill_color, percentage, proj_mat, icon):
        """
        Draws the progress bar. (x,y) is the top-right corner in normalized space.
        The bar is rendered as a sheared parallelogram with a filled portion determined
        by 'percentage'. The projection matrix is used by the shader.
        """
        if screen_width != self.cached_width or screen_width != self.cached_height:
            self.compute_geometry(screen_width, screen_height, x_norm, y_norm, width_norm, height_norm)
            self.cached_width = screen_width
            self.cached_height = screen_height

        # Compute the filled portion geometry based on the current percentage.
        fill_width = percentage * self.width
        # Scale the shear for the filled portion in proportion to the fill width.
        fill_s = self.s * (fill_width / self.width)
        filled_vertices = np.array([
            # First triangle:
            self.x - self.width, self.y,
            self.x - self.width + fill_width, self.y,
            self.x - self.width + fill_width + fill_s, self.y - self.height,
            # Second triangle:
            self.x - self.width, self.y,
            self.x - self.width + fill_width + fill_s, self.y - self.height,
            self.x - self.width + self.s, self.y - self.height
        ], dtype=np.float32)

        # --- Draw using the shader ---
        gl.glUseProgram(self.shader_program)
        # Set projection matrix.
        loc_proj = gl.glGetUniformLocation(self.shader_program, "uProjection")
        gl.glUniformMatrix4fv(loc_proj, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))

        # Get the uniform for the color.
        loc_color = gl.glGetUniformLocation(self.shader_program, "uColor")

        # Draw the backdrop.
        self.backdrop_vbo.bind()
        gl.glEnableVertexAttribArray(0)
        # We pass None for offset if we're using a bound VBO.
        gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, None)
        gl.glUniform4f(loc_color, *self.backdrop_color)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, 6)
        self.backdrop_vbo.unbind()

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

        x_text = self.x - self.width - (2 * 0.015) * screen_width
        y_text = self.y - self.height * 0.5

        x_img = x_text - 35
        y_img = y_text

        self.draw_texture(x_img, y_img, icon, 25.0, proj_mat)
        self.text_renderer.render_text(f"{percentage * 100:.0f}%", x_text, y_text, 1.0, (1.0, 1.0, 1.0), proj_mat)
