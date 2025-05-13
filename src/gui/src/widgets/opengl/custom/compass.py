from OpenGL import GL as gl
from OpenGL.arrays import vbo
from PyQt5 import QtWidgets

import math
import numpy as np
import glm


class Compass():
    """Custom circular angleometer bar with ticks."""

    def __init__(self, text_renderer, medium_text_renderer, large_text_renderer, tick_shader_program, circle_shader_program, compass_shader_program):
        self.text_renderer = text_renderer
        self.medium_text_renderer = medium_text_renderer
        self.large_text_renderer = large_text_renderer
        # Shader programs for drawing the gauge arc and tick marks.
        self.tick_shader_program = tick_shader_program
        self.circle_shader_program = circle_shader_program
        self.compass_shader_program = compass_shader_program

        # Uniforms
        self.tick_u_proj = gl.glGetUniformLocation(self.tick_shader_program, "uProjection")
        self.tick_u_color = gl.glGetUniformLocation(self.tick_shader_program, "uColor")

        self.circle_u_proj = gl.glGetUniformLocation(self.circle_shader_program, 'uProjection')
        self.circle_u_center = gl.glGetUniformLocation(self.circle_shader_program, 'uCenter')
        self.circle_u_color = gl.glGetUniformLocation(self.circle_shader_program, 'color')
        self.circle_u_radius = gl.glGetUniformLocation(self.circle_shader_program, 'radius')
        self.circle_u_lineWidth = gl.glGetUniformLocation(self.circle_shader_program, 'lineWidth')

        self.compass_u_proj = gl.glGetUniformLocation(self.compass_shader_program, "uProjection")
        self.compass_u_model = gl.glGetUniformLocation(self.compass_shader_program, "uModel")
        self.compass_u_color = gl.glGetUniformLocation(self.compass_shader_program, "color")

        # Ratios and constants
        self.innerRadius_ratio = 0.13
        self.outerRadius_ratio = 0.14
        self.large_tick_length_ratio = 0.02
        self.small_tick_length_ratio = 0.01
        self.maxSweep = 360.0
        self.tick_step = 9.0
        self.large_tick_interval = 45.0

        # Cached geometry values (will be recomputed if dimensions change)
        self.cached_screen_width = None
        self.cached_screen_height = None
        self.cached_center = None        # (cx, cy)
        self.quad_vertices = None        # gauge arc quad vertices as a numpy array
        self.circle_vertices = None
        self.compass_vertices = None
        self.large_tick_vertices = None  # large tick vertices (flattened float array)
        self.small_tick_vertices = None  # small tick vertices
        self.label_data = None           # list of (label, x, y) for large ticks

        # VBO's
        quad_placeholder = np.zeros((6 * 2,), dtype=np.float32)
        self.quad_vbo = vbo.VBO(quad_placeholder, usage=gl.GL_DYNAMIC_DRAW)

        # Large ticks: maxSweep=280°, interval=40° → 8 ticks → 8 lines → 16 verts
        large_ticks_count = int(self.maxSweep / self.large_tick_interval) + 1
        large_placeholder = np.zeros((large_ticks_count * 2 * 2,), dtype=np.float32)
        self.large_tick_vbo = vbo.VBO(large_placeholder, usage=gl.GL_DYNAMIC_DRAW)

        # Small ticks: (total ticks) – (large ticks)
        total_ticks = int(self.maxSweep / self.tick_step) + 1
        small_ticks_count = total_ticks - large_ticks_count
        small_placeholder = np.zeros((small_ticks_count * 2 * 2,), dtype=np.float32)
        self.small_tick_vbo = vbo.VBO(small_placeholder, usage=gl.GL_DYNAMIC_DRAW)

        # Inner circle (LINE_LOOP): num_segments points → num_segments verts
        self.num_segments = 128
        circle_placeholder = np.zeros((self.num_segments * 2,), dtype=np.float32)
        self.circle_vbo = vbo.VBO(circle_placeholder, usage=gl.GL_DYNAMIC_DRAW)

        # Compass needle: 6 verts → 6 × (x,y)
        needle_placeholder = np.zeros((6 * 2,), dtype=np.float32)
        self.needle_vbo = vbo.VBO(needle_placeholder, usage=gl.GL_DYNAMIC_DRAW)

    def update_geometry(self, screen_width, screen_height, x_norm, y_norm, min_angle, max_angle):
        """Compute and cache the geometry for the gauge and tick marks."""
        # Compute gauge center based on normalized values.
        cx = x_norm * screen_width
        cy = y_norm * screen_height
        self.cached_center = (cx, cy)

        # Compute radii in pixels.
        screen = QtWidgets.QApplication.primaryScreen()
        max_screen_height = screen.size().height() / 2.3
        self.min_dim = max(screen_height, max_screen_height)
        innerRadius = self.innerRadius_ratio * self.min_dim
        outerRadius = self.outerRadius_ratio * self.min_dim

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

        # Precompute circle geometry
        self.inner_radius = innerRadius * 0.8
        self.num_segments = 128
        theta = np.linspace(0, 2 * np.pi, self.num_segments, endpoint=False)
        x = self.inner_radius * np.cos(theta)
        y = self.inner_radius * np.sin(theta)
        vertices = np.stack((x, y), axis=-1).astype(np.float32)
        self.circle_vertices = vertices + np.array([cx, cy], dtype=np.float32)

        # Precompute compass needle geometry
        self.needle_vertices = np.array([
            -0.5, -0.5,  # A
            0.0, 1.0,  # B
            0.0, -0.2,  # C
            0.5, -0.5,  # A
            0.0, 1.0,  # C
            0.0, -0.2   # D
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
            standard_angle = r
            rad = math.radians(standard_angle)

            if tick_type == 'large':
                # For large ticks, center the tick on the gauge edge: split half-in, half-out.
                x_in = cx + (innerRadius - tick_length / 2) * math.cos(rad)
                y_in = cy - (innerRadius - tick_length / 2) * math.sin(rad)
                x_out = cx + (innerRadius + tick_length) * math.cos(rad)
                y_out = cy - (innerRadius + tick_length) * math.sin(rad)
                large_ticks.extend([x_in, y_in, x_out, y_out])

                x_out = cx + innerRadius * 1.4 * math.cos(rad)
                y_out = cy - innerRadius * 1.4 * math.sin(rad)

                x_in = cx + innerRadius * 0.6 * math.cos(rad)
                y_in = cy - innerRadius * 0.6 * math.sin(rad)
                # Map the fractional position along the sweep to a angle value.
                value = min_angle + (max_angle - min_angle) * (r / self.maxSweep)
                if value != 360:
                    label = str(int(round(value)))
                    # Position the label slightly outwards from the tick's inner endpoint.
                    labels.append((label, x_out, y_out))
                if value == 0:
                    labels.append(('E', x_in, y_in))
                elif value == 90:
                    labels.append(('N', x_in, y_in))
                elif value == 180:
                    labels.append(('W', x_in, y_in))
                elif value == 270:
                    labels.append(('S', x_in, y_in))

            else:
                # For small ticks, the tick starts at innerRadius and extends outward.
                x_in = cx + innerRadius * math.cos(rad)
                y_in = cy - innerRadius * math.sin(rad)
                x_out = cx + (innerRadius + tick_length) * math.cos(rad)
                y_out = cy - (innerRadius + tick_length) * math.sin(rad)
                small_ticks.extend([x_in, y_in, x_out, y_out])
            r += self.tick_step

        self.angle_label_loc = (cx, cy)

        self.large_tick_vertices = np.array(large_ticks, dtype=np.float32)
        self.small_tick_vertices = np.array(small_ticks, dtype=np.float32)
        self.label_data = labels

        # Cache the screen dimensions.
        self.cached_screen_width = screen_width
        self.cached_screen_height = screen_height

        # Reset vbo's
        self.quad_vbo.bind()
        gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 0, self.quad_vertices.nbytes, self.quad_vertices)
        self.quad_vbo.unbind()

        self.large_tick_vbo.bind()
        gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 0, self.large_tick_vertices.nbytes, self.large_tick_vertices)
        self.large_tick_vbo.unbind()

        self.small_tick_vbo.bind()
        gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 0, self.small_tick_vertices.nbytes, self.small_tick_vertices)
        self.small_tick_vbo.unbind()

        self.circle_vbo.bind()
        gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 0, self.circle_vertices.nbytes, self.circle_vertices)
        self.circle_vbo.unbind()

        self.needle_vbo.bind()
        gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 0, self.needle_vertices.nbytes, self.needle_vertices)
        self.needle_vbo.unbind()

    def draw(self, screen_width, screen_height, x_norm, y_norm, proj_mat, current_angle=0, min_angle=0, max_angle=360):
        """
        Draw the gauge arc and tick marks using precomputed geometry.
        Assumes that update_geometry() has been called if screen dimensions or gauge
        normalized position have changed.
        """
        # If dimensions or gauge position have changed, update the geometry.
        if (self.cached_screen_width != screen_width or self.cached_screen_height != screen_height or self.cached_center is None):
            self.update_geometry(screen_width, screen_height, x_norm, y_norm, min_angle, max_angle)

        if self.label_data is not None:
            for label, x, y in self.label_data:
                self.text_renderer.render_text(label, x, y, 1.0, (1.0, 1.0, 1.0), proj_mat)
        if hasattr(self, 'angle_label_loc'):
            self.medium_text_renderer.render_text(f"{(current_angle) % 360:.1f}°", self.angle_label_loc[0], self.angle_label_loc[1], 1.0, (1.0, 1.0, 0.0), proj_mat)

        cx, cy = self.cached_center
        # Compute normalized progress.
        progress = (current_angle - min_angle) / (max_angle - min_angle)
        progress = max(0.0, min(1.0, progress))

        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)

        # Draw the ticks.
        self.draw_ticks(proj_mat)

        # Draw inner circle.
        self.draw_circle(proj_mat)

        # Draw yaw needle
        self.draw_compass_needle(proj_mat, -current_angle)

        gl.glDisable(gl.GL_BLEND)

    def draw_ticks(self, proj_mat):
        """Draw ticks using the precomputed tick buffers and text labels."""
        # Draw large ticks.
        if self.large_tick_vertices is not None and len(self.large_tick_vertices) > 0:
            gl.glUseProgram(self.tick_shader_program)
            gl.glUniformMatrix4fv(self.tick_u_proj, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))
            gl.glUniform4f(self.tick_u_color, 1.0, 1.0, 1.0, 1.0)
            gl.glLineWidth(3.0)

            self.large_tick_vbo.bind()
            gl.glEnableVertexAttribArray(0)
            gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))
            num_vertices = len(self.large_tick_vertices) // 2
            gl.glDrawArrays(gl.GL_LINES, 0, num_vertices)
            self.large_tick_vbo.unbind()
            gl.glDisableVertexAttribArray(0)

        # Draw small ticks.
        if self.small_tick_vertices is not None and len(self.small_tick_vertices) > 0:
            gl.glUseProgram(self.tick_shader_program)
            gl.glUniformMatrix4fv(self.tick_u_proj, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))
            gl.glUniform4f(self.tick_u_color, 1.0, 1.0, 1.0, 1.0)
            gl.glLineWidth(1.0)

            self.small_tick_vbo.bind()
            gl.glEnableVertexAttribArray(0)
            gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))
            num_vertices = len(self.small_tick_vertices) // 2
            gl.glDrawArrays(gl.GL_LINES, 0, num_vertices)
            self.small_tick_vbo.unbind()
            gl.glDisableVertexAttribArray(0)

    def draw_circle(self, proj_mat):
        gl.glUseProgram(self.circle_shader_program)
        cx, cy = self.cached_center
        gl.glUniformMatrix4fv(self.circle_u_proj, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))
        gl.glUniform2f(self.circle_u_center, cx, cy)
        gl.glUniform4f(self.circle_u_color, 1.0, 1.0, 1.0, 1.0)
        gl.glUniform1f(self.circle_u_radius, self.inner_radius)
        gl.glUniform1f(self.circle_u_lineWidth, 0.01)

        self.circle_vbo.bind()
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))
        gl.glDrawArrays(gl.GL_LINE_LOOP, 0, self.num_segments)
        self.circle_vbo.unbind()
        gl.glDisableVertexAttribArray(0)

    def draw_compass_needle(self, proj_mat, angle):
        gl.glUseProgram(self.compass_shader_program)
        cx, cy = self.cached_center
        scale_factor = 0.032 * self.cached_screen_height

        needle_x = cx + self.inner_radius * 1.2 * np.cos(np.radians(angle))
        needle_y = cy + self.inner_radius * 1.2 * np.sin(np.radians(angle))

        model = glm.translate(glm.mat4(1.0), glm.vec3(needle_x, needle_y, 0.0))
        model = glm.rotate(model, glm.radians(angle - 90), glm.vec3(0.0, 0.0, 1.0))
        model = glm.scale(model, glm.vec3(scale_factor, scale_factor, 1.0))

        gl.glUniformMatrix4fv(self.compass_u_proj, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))
        gl.glUniformMatrix4fv(self.compass_u_model, 1, gl.GL_FALSE, glm.value_ptr(model))

        # Bind the needle VBO.
        self.needle_vbo.bind()
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))

        # Draw first triangle in bright red.
        gl.glUniform4f(self.compass_u_color, 1.0, 0.0, 0.0, 1.0)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, 3)

        # Now draw the second triangle in dark red.
        gl.glUniform4f(self.compass_u_color, 0.6, 0.0, 0.0, 1.0)  # adjust these values as needed for "dark red"
        gl.glDrawArrays(gl.GL_TRIANGLES, 3, 3)
        self.needle_vbo.unbind()

        gl.glDisableVertexAttribArray(0)
