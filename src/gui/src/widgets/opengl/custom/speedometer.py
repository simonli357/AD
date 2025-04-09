import math
import numpy as np
from OpenGL import GL as gl
from OpenGL.arrays import vbo
import glm


class Speedometer():
    """Custom circular speedometer bar with ticks."""

    def __init__(self, shader_program, tick_shader_program):
        # shader_program: for drawing the gauge arc
        # tick_shader_program: for drawing tick marks
        self.shader_program = shader_program
        self.tick_shader_program = tick_shader_program

    def draw(self, qpainter, screen_width, screen_height, x_norm, y_norm, proj_mat,
             min_speed=0, max_speed=50, fill_color=(0.0, 0.5, 0.8, 0.5), current_speed=50):
        """
        Draw the gauge arc and then overlay the tick marks.
        """
        # Denormalize center coordinates.
        cx = x_norm * screen_width
        cy = y_norm * screen_height

        # Compute normalized progress (clamped to 0-1).
        progress = (current_speed - min_speed) / (max_speed - min_speed)
        progress = max(0.0, min(1.0, progress))

        # Define gauge radii (in pixels).
        innerRadius = 0.19 * screen_height  # adjust as desired
        outerRadius = 0.2 * screen_height     # adjust as desired

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

        # Use the gauge shader to draw the circular arc.
        gl.glUseProgram(self.shader_program)
        loc_proj = gl.glGetUniformLocation(self.shader_program, "uProjection")
        gl.glUniformMatrix4fv(loc_proj, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))

        # Set uniforms for the gauge.
        loc_center = gl.glGetUniformLocation(self.shader_program, "uCenter")
        gl.glUniform2f(loc_center, cx, cy)
        loc_inner = gl.glGetUniformLocation(self.shader_program, "uInnerRadius")
        gl.glUniform1f(loc_inner, innerRadius)
        loc_outer = gl.glGetUniformLocation(self.shader_program, "uOuterRadius")
        gl.glUniform1f(loc_outer, outerRadius)
        loc_progress = gl.glGetUniformLocation(self.shader_program, "uProgress")
        gl.glUniform1f(loc_progress, progress)
        loc_fill = gl.glGetUniformLocation(self.shader_program, "uFillColor")
        gl.glUniform4f(loc_fill, *fill_color)
        # Set a transparent background color for unfilled areas.
        loc_bg = gl.glGetUniformLocation(self.shader_program, "uBgColor")
        gl.glUniform4f(loc_bg, 0.0, 0.0, 0.0, 0.0)

        # Bind and draw the quad.
        quad_vbo = vbo.VBO(quad_vertices)
        quad_vbo.bind()
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, quad_vbo)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, 6)
        quad_vbo.unbind()
        quad_vbo.delete()
        gl.glDisableVertexAttribArray(0)
        gl.glUseProgram(0)

        # Now draw the ticks.
        self.draw_ticks(cx, cy, innerRadius, screen_height, proj_mat)

    def draw_ticks(self, cx, cy, outerRadius, screen_height, proj_mat):
        """
        Draw tick marks along the gauge’s outer edge.
        A large tick is drawn every 20° and 3 small ticks are drawn between large ticks (every 5°).
        We use our custom angle system: the gauge fill starts at custom angle 0 (which maps to 90° standard).
        Now, ticks will be drawn clockwise from 90°.
        """
        # Set the total sweep of the gauge.
        maxSweep = 280.0  # in degrees (as used by the gauge shader)
        tick_step = 10.0   # degrees between ticks

        # Define tick lengths (in pixels).
        large_tick_length = 0.02 * screen_height  # e.g. 2% of screen height
        small_tick_length = 0.01 * screen_height    # e.g. 1% of screen height

        # Separate lists for large and small tick vertices.
        large_tick_vertices = []  # for large ticks
        small_tick_vertices = []  # for small ticks

        # Loop over each tick position (custom angle 'r' from 0 to maxSweep)
        # Custom angle 0 corresponds to standard angle 90°
        r = 0.0
        while r <= maxSweep + 0.001:  # include endpoint with tolerance
            # Determine if this is a large tick or a small tick.
            if abs(r % 40.0) < 1e-5:
                tick_length = large_tick_length
                tick_type = 'large'
            else:
                tick_length = small_tick_length
                tick_type = 'small'

            # Convert custom angle to standard angle (clockwise drawing)
            # Instead of 90 + r, use 90 - r.
            standard_angle = -90.0 - r
            rad = math.radians(standard_angle)

            # Append the two vertices for this tick line segment.
            if tick_type == 'large':
                # Compute the inner tick point (at the gauge outer radius).
                x_in = cx + (outerRadius - tick_length / 2) * math.cos(rad)
                y_in = cy - (outerRadius - tick_length / 2) * math.sin(rad)
                # Compute the outer tick point (extending outward).
                x_out = cx + (outerRadius + tick_length) * math.cos(rad)
                y_out = cy - (outerRadius + tick_length) * math.sin(rad)
                large_tick_vertices.extend([x_in, y_in, x_out, y_out])
            else:
                # Compute the inner tick point (at the gauge outer radius).
                x_in = cx + outerRadius * math.cos(rad)
                y_in = cy - outerRadius * math.sin(rad)
                # Compute the outer tick point (extending outward).
                x_out = cx + (outerRadius + tick_length) * math.cos(rad)
                y_out = cy - (outerRadius + tick_length) * math.sin(rad)
                small_tick_vertices.extend([x_in, y_in, x_out, y_out])

            r += tick_step

        # Draw large ticks (fat)
        if large_tick_vertices:
            large_tick_vertices_np = np.array(large_tick_vertices, dtype=np.float32)
            gl.glUseProgram(self.tick_shader_program)
            loc_proj = gl.glGetUniformLocation(self.tick_shader_program, "uProjection")
            gl.glUniformMatrix4fv(loc_proj, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))
            loc_color = gl.glGetUniformLocation(self.tick_shader_program, "uColor")
            gl.glUniform4f(loc_color, 1.0, 1.0, 1.0, 1.0)
            gl.glLineWidth(3.0)  # Thicker line for large ticks

            tick_vbo = vbo.VBO(large_tick_vertices_np)
            tick_vbo.bind()
            gl.glEnableVertexAttribArray(0)
            gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, tick_vbo)
            num_vertices = len(large_tick_vertices_np) // 2
            gl.glDrawArrays(gl.GL_LINES, 0, num_vertices)
            tick_vbo.unbind()
            tick_vbo.delete()
            gl.glDisableVertexAttribArray(0)
            gl.glUseProgram(0)

        # Draw small ticks (thin)
        if small_tick_vertices:
            small_tick_vertices_np = np.array(small_tick_vertices, dtype=np.float32)
            gl.glUseProgram(self.tick_shader_program)
            loc_proj = gl.glGetUniformLocation(self.tick_shader_program, "uProjection")
            gl.glUniformMatrix4fv(loc_proj, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))
            loc_color = gl.glGetUniformLocation(self.tick_shader_program, "uColor")
            gl.glUniform4f(loc_color, 1.0, 1.0, 1.0, 1.0)
            gl.glLineWidth(1.0)  # Thinner line for small ticks

            tick_vbo = vbo.VBO(small_tick_vertices_np)
            tick_vbo.bind()
            gl.glEnableVertexAttribArray(0)
            gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, tick_vbo)
            num_vertices = len(small_tick_vertices_np) // 2
            gl.glDrawArrays(gl.GL_LINES, 0, num_vertices)
            tick_vbo.unbind()
            tick_vbo.delete()
            gl.glDisableVertexAttribArray(0)
            gl.glUseProgram(0)
