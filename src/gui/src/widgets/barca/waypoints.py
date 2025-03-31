from OpenGL import GL as gl
import numpy as np
from OpenGL.arrays import vbo


class WaypointsRenderer:
    def __init__(self):
        self.vbo = None
        self.vertex_count = 0
        self.num_waypoints = 0
        self.base_shape = self._create_base_shape()
        self._offsets = []
        self._counts = []

    def _create_base_shape(self):
        angle_step = 2 * np.pi / 10
        return np.array([
            [np.cos(angle), np.sin(angle), 0]
            for angle in np.arange(0, 2 * np.pi, angle_step)
        ], dtype=np.float32) * 0.02

    def update_waypoints(self, state_refs_np):
        if state_refs_np is None or state_refs_np.shape[1] == 0:
            return

        self.num_waypoints = state_refs_np.shape[1]
        verts = []
        self._offsets = []
        self._counts = []

        for i in range(self.num_waypoints):
            transformed = self.base_shape.copy()
            transformed[:, 0] += state_refs_np[0, i]
            transformed[:, 1] += state_refs_np[1, i]
            verts.append(transformed)
            self._offsets.append(i * len(self.base_shape))
            self._counts.append(len(self.base_shape))

        vertex_data = np.concatenate(verts, dtype=np.float32)

        if self.vbo is None:
            self.vbo = vbo.VBO(vertex_data)
        else:
            self.vbo.set_array(vertex_data)

        self.vertex_count = vertex_data.shape[0]

    def draw(self):
        if self.vbo is None or self.vertex_count == 0:
            return

        gl.glPushMatrix()
        gl.glRotatef(90, 0, 0, 1)

        gl.glColor3f(1, 1, 0)
        self.vbo.bind()
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, self.vbo)

        gl.glMultiDrawArrays(
            gl.GL_LINE_LOOP,
            self._offsets,
            self._counts,
            self.num_waypoints
        )

        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        self.vbo.unbind()
        gl.glPopMatrix()
