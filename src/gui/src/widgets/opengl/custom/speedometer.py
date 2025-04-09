from OpenGL import GL as gl
from OpenGL.arrays import vbo
import numpy as np
import glm


class Speedometer():
    """Custom circular speedometer bar"""

    def __init__(self, gauge_shader_program, tick_shader_program):
        super(Speedometer, self).__init__()
        self.gauge_shader_program = gauge_shader_program
        self.tick_shader_program = tick_shader_program

    def draw(self, qpainter, screen_width, screen_height, x_norm, y_norm, proj_mat,
             min_speed=0, max_speed=50, fill_color=(0.0, 0.5, 0.8, 0.5), current_speed=50):
        """
        Draw a circular speedometer gauge (a semicircular ring) with subdivisions.
        The fill starts at 180° (left) and proceeds clockwise depending on current_speed.

        :param qpainter: A QPainter instance (for optional text overlay).
        :param screen_width: Screen width in pixels.
        :param screen_height: Screen height in pixels.
        :param x_norm: Normalized x coordinate for the gauge center.
        :param y_norm: Normalized y coordinate for the gauge center.
        :param proj_mat: Projection matrix.
        :param min_speed: Minimum speed (start).
        :param max_speed: Maximum speed (end).
        :param fill_color: RGBA tuple for the filled portion.
        :param current_speed: Current speed value.
        """
        # Denormalize center coordinates.
        cx = x_norm * screen_width
        cy = y_norm * screen_height

        # Compute normalized progress (clamped to 0-1).
        progress = (current_speed - min_speed) / (max_speed - min_speed)
        progress = max(0.0, min(1.0, progress))

        # Define radii of the gauge (in pixels).
        innerRadius = 0.19 * screen_height  # adjust as desired
        outerRadius = 0.2 * screen_height  # adjust as desired

        # Create a quad that covers the full circle that encloses the outer radius.
        left = cx - outerRadius
        bottom = cy - outerRadius
        quad_width = outerRadius * 2
        quad_height = outerRadius * 2

        # Define quad vertices (in pixel coordinates).
        quad_vertices = np.array([
            left, bottom + quad_height,   # top-left
            left, bottom,                 # bottom-left
            left + quad_width, bottom,    # bottom-right

            left, bottom + quad_height,   # top-left
            left + quad_width, bottom,    # bottom-right
            left + quad_width, bottom + quad_height  # top-right
        ], dtype=np.float32)

        # Use the speedometer shader.
        gl.glUseProgram(self.gauge_shader_program)

        # Set the projection matrix.
        loc_proj = gl.glGetUniformLocation(self.gauge_shader_program, "uProjection")
        gl.glUniformMatrix4fv(loc_proj, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))

        # Set uniform values for the gauge.
        loc_center = gl.glGetUniformLocation(self.gauge_shader_program, "uCenter")
        gl.glUniform2f(loc_center, cx, cy)
        loc_inner = gl.glGetUniformLocation(self.gauge_shader_program, "uInnerRadius")
        gl.glUniform1f(loc_inner, innerRadius)
        loc_outer = gl.glGetUniformLocation(self.gauge_shader_program, "uOuterRadius")
        gl.glUniform1f(loc_outer, outerRadius)
        loc_progress = gl.glGetUniformLocation(self.gauge_shader_program, "uProgress")
        gl.glUniform1f(loc_progress, progress)
        loc_fill = gl.glGetUniformLocation(self.gauge_shader_program, "uFillColor")
        gl.glUniform4f(loc_fill, *fill_color)
        # Set a background color for unfilled areas.
        loc_bg = gl.glGetUniformLocation(self.gauge_shader_program, "uBgColor")
        # For example, dark grey:
        gl.glUniform4f(loc_bg, 0.0, 0.0, 0.0, 0.0)

        # Bind and draw the quad.
        quad_vbo = vbo.VBO(quad_vertices)
        quad_vbo.bind()
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, 6)
        quad_vbo.unbind()
        quad_vbo.delete()

        gl.glDisableVertexAttribArray(0)
        gl.glUseProgram(0)
