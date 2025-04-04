from OpenGL import GL as gl
import numpy as np
from OpenGL.arrays import vbo
from ...enums import MapData


class WaypointsRenderer:
    def __init__(self):
        self.vbo = None
        self.vertex_count = 0
        self.num_waypoints = 0
        self.base_shape = self._create_base_shape()
        self._offsets = []
        self._counts = []
        self.ATTRIBUTES = {
            0: (1.0, 1.0, 0.0),    # Yellow (normal)
            1: (0.0, 1.0, 0.0),    # Green (crosswalk)
            2: (0.0, 0.0, 1.0),    # Red (intersection)
            3: (0.2, 0.5, 1.0),    # Orange (oneway)
            4: (0.5, 0.0, 0.5),    # Indigo (highwayLeft)
            5: (0.8, 0.7, 1.0),     # Light pink (highwayRight)
            6: (1.0, 1.0, 1.0),     # White (roundabout)
            7: (0.0, 1.0, 1.0),     # Cyan (stopline)
            8: (0.4, 0.5, 0.7),    # Steel blue (dotted)
            9: (0.5, 0.0, 0.5),     # Purple (dotted_crosswalk)
        }

    def _create_base_shape(self):
        angle_step = 2 * np.pi / 10
        wp_size = 0.5
        return np.array([
            [np.cos(angle), np.sin(angle), 0]
            for angle in np.arange(0, 2 * np.pi, angle_step)
        ], dtype=np.float32) * wp_size

    def update_waypoints(self, state_refs_np, attributes_np, widget_width, widget_height):
        if state_refs_np is None or state_refs_np.shape[1] == 0:
            return

        verts = []
        self._offsets = []
        self._counts = []

        waypoint_indices = range(0, state_refs_np.shape[1], 1)
        self.num_waypoints = len(waypoint_indices)

        for idx, i in enumerate(waypoint_indices):
            # Get position
            x = state_refs_np[0, i] / MapData.REAL_WORLD_WIDTH.value * widget_width
            y = state_refs_np[1, i] / MapData.REAL_WORLD_HEIGHT.value * widget_height

            # Get color from attributes
            attr = attributes_np[i] % 10  # Handle 100-series attributes
            color = self.ATTRIBUTES.get(attr, (1.0, 1.0, 0.0))  # Default yellow

            # Create shape with color
            transformed = self.base_shape.copy()
            transformed[:, 0] += x
            transformed[:, 1] += y

            # Add color to each vertex (position + color)
            colored_verts = np.hstack((
                transformed,
                np.tile(color, (len(transformed), 1))
            ))

            verts.append(colored_verts)
            self._offsets.append(idx * len(self.base_shape))
            self._counts.append(len(self.base_shape))

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

        gl.glPushMatrix()
        gl.glPushAttrib(gl.GL_CURRENT_BIT)
        try:
            self.vbo.bind()

            # Set up interleaved arrays (position + color)
            gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
            gl.glEnableClientState(gl.GL_COLOR_ARRAY)

            # Stride = 6 floats (3 position, 3 color) * 4 bytes
            stride = 6 * 4
            gl.glVertexPointer(3, gl.GL_FLOAT, stride, self.vbo)
            gl.glColorPointer(3, gl.GL_FLOAT, stride, self.vbo + 12)

            gl.glMultiDrawArrays(
                gl.GL_LINE_LOOP,
                (gl.GLint * len(self._offsets))(*self._offsets),
                (gl.GLsizei * len(self._counts))(*self._counts),
                self.num_waypoints
            )
        finally:
            gl.glDisableClientState(gl.GL_COLOR_ARRAY)
            gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
            self.vbo.unbind()
            gl.glLineWidth(1.0)
            gl.glPopAttrib()
            gl.glPopMatrix()
