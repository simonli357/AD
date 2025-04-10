import math
import numpy as np
from OpenGL import GL as gl
from OpenGL.arrays import vbo
import glm


class Speedometer():
    """Custom circular speedometer bar with ticks.

    This version precomputes geometry (the gauge quad and tick vertices)
    when the widget dimensions or gauge position change.
    """

    def __init__(self, text_renderer, gauge_shader_program, tick_shader_program):
        # Shader programs for drawing the gauge arc and tick marks.
        self.gauge_shader_program = gauge_shader_program
        self.tick_shader_program = tick_shader_program

        # Ratios and constants
        self.innerRadius_ratio = 0.23
        self.outerRadius_ratio = 0.24
        self.large_tick_length_ratio = 0.02
        self.small_tick_length_ratio = 0.01
        self.maxSweep = 280.0      # total sweep, in degrees
        self.tick_step = 10.0      # degrees between ticks
        self.large_tick_interval = 40.0  # every 40° is a large tick

        # Cached geometry values (will be recomputed if dimensions change)
        self.cached_screen_width = None
        self.cached_screen_height = None
        self.cached_center = None        # (cx, cy)
        self.quad_vertices = None        # gauge arc quad vertices as a numpy array
        self.large_tick_vertices = None  # large tick vertices (flattened float array)
        self.small_tick_vertices = None  # small tick vertices
        self.label_data = None           # list of (label, x, y) for large ticks

    def update_geometry(self, screen_width, screen_height, x_norm, y_norm, min_speed, max_speed):
        """Compute and cache the geometry for the gauge and tick marks."""
        # Compute gauge center based on normalized values.
        cx = x_norm * screen_width
        cy = y_norm * screen_height
        self.cached_center = (cx, cy)

        # Compute radii in pixels.
        innerRadius = self.innerRadius_ratio * screen_height
        outerRadius = self.outerRadius_ratio * screen_height

        # Precompute gauge quad vertices (covers full circle enclosing outerRadius).
        left = cx - outerRadius
        bottom = cy - outerRadius
        quad_width = outerRadius * 2
        quad_height = outerRadius * 2
        self.quad_vertices = np.array([
            left, bottom + quad_height,   # top-left
            left, bottom,                 # bottom-left
            left + quad_width, bottom,    # bottom-right

            left, bottom + quad_height,   # top-left
            left + quad_width, bottom,    # bottom-right
            left + quad_width, bottom + quad_height  # top-right
        ], dtype=np.float32)

        # Precompute tick geometry.
        large_ticks = []
        small_ticks = []
        labels = []

        # Loop over each tick (custom angle r from 0 to maxSweep).
        # In our gauge coordinate system, custom angle 0 corresponds to standard -90°.
        r = 0.0
        while r <= self.maxSweep + 0.001:
            # Determine tick type (large if r is a multiple of large_tick_interval).
            if abs(r % self.large_tick_interval) < 1e-5:
                tick_length = self.large_tick_length_ratio * screen_height
                tick_type = 'large'
            else:
                tick_length = self.small_tick_length_ratio * screen_height
                tick_type = 'small'

            # For clockwise drawing, map custom angle r to standard angle:
            # custom angle 0 -> standard -90°, increasing r rotates clockwise.
            standard_angle = -90.0 - r
            rad = math.radians(standard_angle)

            if tick_type == 'large':
                # For large ticks, center the tick on the gauge edge: split half-in, half-out.
                x_in = cx + (innerRadius - tick_length / 2) * math.cos(rad)
                y_in = cy - (innerRadius - tick_length / 2) * math.sin(rad)
                x_out = cx + (innerRadius + tick_length) * math.cos(rad)
                y_out = cy - (innerRadius + tick_length) * math.sin(rad)
                large_ticks.extend([x_in, y_in, x_out, y_out])

                x_in = cx + innerRadius * 0.80 * math.cos(rad)
                y_in = cy - innerRadius * 0.80 * math.sin(rad)
                # Map the fractional position along the sweep to a speed value.
                value = min_speed + (max_speed - min_speed) * (r / self.maxSweep)
                label = str(int(round(value)))
                # Position the label slightly inward from the tick's inner endpoint.
                labels.append((label, x_in, y_in))
            else:
                # For small ticks, the tick starts at innerRadius and extends outward.
                x_in = cx + innerRadius * math.cos(rad)
                y_in = cy - innerRadius * math.sin(rad)
                x_out = cx + (innerRadius + tick_length) * math.cos(rad)
                y_out = cy - (innerRadius + tick_length) * math.sin(rad)
                small_ticks.extend([x_in, y_in, x_out, y_out])
            r += self.tick_step

        self.large_tick_vertices = np.array(large_ticks, dtype=np.float32)
        self.small_tick_vertices = np.array(small_ticks, dtype=np.float32)
        self.label_data = labels

        # Cache the screen dimensions.
        self.cached_screen_width = screen_width
        self.cached_screen_height = screen_height

    def draw(self, screen_width, screen_height, x_norm, y_norm, proj_mat,
             min_speed=0, max_speed=50, fill_color=(0.0, 0.5, 0.8, 0.75), current_speed=50):
        """
        Draw the gauge arc and tick marks using precomputed geometry.
        Assumes that update_geometry() has been called if screen dimensions or gauge
        normalized position have changed.
        """
        # if self.label_data is not None:
        #     for label, x, y in self.label_data:
        #         self.render_text(qpainter, label, 10, (255, 255, 255, 255), x, y)

        # If dimensions or gauge position have changed, update the geometry.
        if (self.cached_screen_width != screen_width or self.cached_screen_height != screen_height or self.cached_center is None):
            self.update_geometry(screen_width, screen_height, x_norm, y_norm, min_speed, max_speed)

        cx, cy = self.cached_center
        # Compute normalized progress.
        progress = (current_speed - min_speed) / (max_speed - min_speed)
        progress = max(0.0, min(1.0, progress))

        # Use precomputed radii.
        innerRadius = self.innerRadius_ratio * screen_height
        outerRadius = self.outerRadius_ratio * screen_height

        # Draw the gauge arc quad.
        gl.glUseProgram(self.gauge_shader_program)
        loc_proj = gl.glGetUniformLocation(self.gauge_shader_program, "uProjection")
        gl.glUniformMatrix4fv(loc_proj, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))

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
        loc_bg = gl.glGetUniformLocation(self.gauge_shader_program, "uBgColor")
        gl.glUniform4f(loc_bg, 0.0, 0.0, 0.0, 0.0)

        quad_vbo = vbo.VBO(self.quad_vertices)
        quad_vbo.bind()
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, quad_vbo)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, 6)
        quad_vbo.unbind()
        quad_vbo.delete()
        gl.glDisableVertexAttribArray(0)
        gl.glUseProgram(0)

        # Draw the ticks.
        self.draw_ticks(proj_mat)

    def draw_ticks(self, proj_mat):
        """Draw ticks using the precomputed tick buffers and (optionally) text labels."""
        # Draw large ticks.
        if self.large_tick_vertices is not None and len(self.large_tick_vertices) > 0:
            gl.glUseProgram(self.tick_shader_program)
            loc_proj = gl.glGetUniformLocation(self.tick_shader_program, "uProjection")
            gl.glUniformMatrix4fv(loc_proj, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))
            loc_color = gl.glGetUniformLocation(self.tick_shader_program, "uColor")
            gl.glUniform4f(loc_color, 1.0, 1.0, 1.0, 1.0)
            gl.glLineWidth(3.0)

            tick_vbo = vbo.VBO(self.large_tick_vertices)
            tick_vbo.bind()
            gl.glEnableVertexAttribArray(0)
            gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, tick_vbo)
            num_vertices = len(self.large_tick_vertices) // 2
            gl.glDrawArrays(gl.GL_LINES, 0, num_vertices)
            tick_vbo.unbind()
            tick_vbo.delete()
            gl.glDisableVertexAttribArray(0)
            gl.glUseProgram(0)

        # Draw small ticks.
        if self.small_tick_vertices is not None and len(self.small_tick_vertices) > 0:
            gl.glUseProgram(self.tick_shader_program)
            loc_proj = gl.glGetUniformLocation(self.tick_shader_program, "uProjection")
            gl.glUniformMatrix4fv(loc_proj, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))
            loc_color = gl.glGetUniformLocation(self.tick_shader_program, "uColor")
            gl.glUniform4f(loc_color, 1.0, 1.0, 1.0, 1.0)
            gl.glLineWidth(1.0)

            tick_vbo = vbo.VBO(self.small_tick_vertices)
            tick_vbo.bind()
            gl.glEnableVertexAttribArray(0)
            gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, tick_vbo)
            num_vertices = len(self.small_tick_vertices) // 2
            gl.glDrawArrays(gl.GL_LINES, 0, num_vertices)
            tick_vbo.unbind()
            tick_vbo.delete()
            gl.glDisableVertexAttribArray(0)
            gl.glUseProgram(0)
