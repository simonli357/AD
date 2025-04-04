from OpenGL import GL as gl
from OpenGL.arrays import vbo
from ..enums import MapData
from .shader import create_shader_program
from .shader import shader_path

import glm
import numpy as np


class WaypointsRenderer:
    def __init__(self):
        self.vbo = None
        self.vertex_count = 0
        self.num_waypoints = 0
        self.base_shape = self._create_base_shape()
        self._offsets = []
        self._counts = []
        self.shader_program = create_shader_program(shader_path('diamond', 'diamond.vert'), shader_path('diamond', 'diamond.frag'))
        self.projection_uniform = gl.glGetUniformLocation(self.shader_program, b"projection")
        self.ATTRIBUTES = {
            0: (1.0, 1.0, 0.0),    # Yellow
            1: (0.0, 1.0, 0.0),    # Green
            2: (0.0, 0.0, 1.0),    # Blue
            3: (1.0, 0.5, 0.0),    # Orange
            4: (0.5, 0.0, 0.5),    # Purple
            5: (0.8, 0.7, 1.0),    # Light pink
            6: (1.0, 1.0, 1.0),    # White
            7: (0.0, 1.0, 1.0),    # Cyan
            8: (0.4, 0.5, 0.7),    # Steel blue
            9: (0.5, 0.0, 0.5),     # Purple
        }

    def _create_base_shape(self):
        """Create a diamond shape using 2 triangles (6 vertices)"""
        wp_size = 0.5
        return np.array([
            [0.0, wp_size, 0],    # Top
            [wp_size, 0.0, 0],    # Right
            [-wp_size, 0.0, 0],    # Left
            [-wp_size, 0.0, 0],    # Left
            [wp_size, 0.0, 0],     # Right
            [0.0, -wp_size, 0],    # Bottom
        ], dtype=np.float32)

    def get_gl_coords(self, real_x, real_y, widget_width, widget_height):
        if widget_height == 0 or widget_width == 0:
            return (0.0, 0.0)

        # Convert real-world to OpenGL world coordinates
        world_x = (real_x * widget_width / MapData.REAL_WORLD_WIDTH.value) - (widget_width / 2)
        world_y = (real_y * widget_height / MapData.REAL_WORLD_HEIGHT.value) - (widget_height / 2)

        return world_x, world_y

    def update_waypoints(self, state_refs_np, attributes_np, widget_width, widget_height, projection_matrix, view_matrix):
        if state_refs_np is None or state_refs_np.shape[1] == 0:
            return
        self.projection_matrix = projection_matrix
        self.view_matrix = view_matrix
        verts = []
        self._offsets = []
        self._counts = []

        waypoints = state_refs_np[:, ::1]  # Process all waypoints
        self.num_waypoints = waypoints.shape[1]

        for idx in range(self.num_waypoints):
            x, y = self.get_gl_coords(waypoints[0, idx], waypoints[1, idx], widget_width, widget_height)

            attr = attributes_np[idx] % 10
            color = self.ATTRIBUTES.get(attr, (1.0, 1.0, 0.0))

            # Create translated diamond
            translated = self.base_shape.copy()
            translated[:, 0] += x
            translated[:, 1] += y

            # Add color to each vertex
            colored_verts = np.hstack((
                translated,
                np.tile(color, (len(translated), 1))
            ))

            verts.append(colored_verts)
            self._offsets.append(idx * 6)  # 6 vertices per diamond
            self._counts.append(6)

        if verts:
            vertex_data = np.concatenate(verts, dtype=np.float32)
            if self.vbo is None:
                self.vbo = vbo.VBO(vertex_data)
            else:
                self.vbo.set_array(vertex_data)
            self.vertex_count = vertex_data.shape[0]
        else:
            self.vertex_count = 0

    def draw(self):
        if self.vbo is None or self.vertex_count == 0:
            return
        if not hasattr(self, 'projection_matrix') or not hasattr(self, 'view_matrix'):
            return

        gl.glUseProgram(self.shader_program)
        self.vbo.bind()

        try:
            stride = 6 * 4  # 3 pos + 3 color * 4 bytes

            gl.glUniformMatrix4fv(
                gl.glGetUniformLocation(self.shader_program, "projection"),
                1, gl.GL_FALSE, glm.value_ptr(self.projection_matrix)
            )

            gl.glUniformMatrix4fv(
                gl.glGetUniformLocation(self.shader_program, "view"),
                1, gl.GL_FALSE, glm.value_ptr(self.view_matrix)
            )

            # Position attribute (location 0)
            gl.glEnableVertexAttribArray(0)
            gl.glVertexAttribPointer(
                0, 3, gl.GL_FLOAT, gl.GL_FALSE, stride, self.vbo
            )

            # Color attribute (location 1)
            gl.glEnableVertexAttribArray(1)
            gl.glVertexAttribPointer(
                1, 3, gl.GL_FLOAT, gl.GL_FALSE, stride, self.vbo + 12
            )

            gl.glMultiDrawArrays(
                gl.GL_TRIANGLES,
                (gl.GLint * len(self._offsets))(*self._offsets),
                (gl.GLsizei * len(self._counts))(*self._counts),
                self.num_waypoints
            )
        finally:
            self.vbo.unbind()
            gl.glDisableVertexAttribArray(0)
            gl.glDisableVertexAttribArray(1)
            gl.glUseProgram(0)
