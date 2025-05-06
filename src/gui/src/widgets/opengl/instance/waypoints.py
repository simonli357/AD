from OpenGL import GL as gl
from ..utils import create_shader_program, shader_path
from ...enums import MapData
import numpy as np
import glm
import math


class WaypointsRenderer:
    def __init__(self, track, max_instances=20000):
        self.track = track
        self.num_instances = 0
        self.max_instances = max_instances

        # ——— 1) VAO + base geometry ———
        self.vao = gl.glGenVertexArrays(1)
        gl.glBindVertexArray(self.vao)

        self.base_vertices = np.array([
            [0.0, 0.5, 0.1],
            [0.5, 0.0, 0.1],
            [-0.5, 0.0, 0.1],
            [-0.5, 0.0, 0.1],
            [0.5, 0.0, 0.1],
            [0.0, -0.5, 0.1],
        ], dtype=np.float32).flatten()

        self.base_vbo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.base_vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, self.base_vertices.nbytes, self.base_vertices, gl.GL_STATIC_DRAW)

        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))

        # ——— 2) Instance buffer: [pos.x, pos.y, angle, scale, r, g, b, a] ———
        self.instance_data = np.zeros((self.max_instances, 8), dtype=np.float32)
        self.instance_vbo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, self.instance_data.nbytes, None, gl.GL_DYNAMIC_DRAW)

        stride = 8 * 4  # 8 floats per instance

        # Define instanced attributes
        offsets = [0, 2 * 4, 3 * 4, 4 * 4]  # byte offsets for pos, angle, scale, color
        sizes = [2, 1, 1, 4]  # vec2, float, float, vec4

        for i, (offset, size) in enumerate(zip(offsets, sizes), start=1):
            gl.glEnableVertexAttribArray(i)
            gl.glVertexAttribPointer(i, size, gl.GL_FLOAT, gl.GL_FALSE, stride, gl.ctypes.c_void_p(offset))
            gl.glVertexAttribDivisor(i, 1)

        # Clean up
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
        gl.glBindVertexArray(0)

        # Attribute to color map
        self.ATTRIBUTES = {
            0: (1.0, 1.0, 0.0),
            1: (0.0, 1.0, 0.0),
            2: (1.0, 0.0, 0.0),
            3: (1.0, 0.5, 0.0),
            4: (0.5, 0.0, 0.5),
            5: (0.8, 0.7, 1.0),
            6: (1.0, 1.0, 1.0),
            7: (0.0, 1.0, 1.0),
            8: (0.4, 0.5, 0.7),
            9: (0.5, 0.0, 0.5),
            10: (0.0, 0.0, 1.0)
        }

        # Compile shaders
        self.shader_program = create_shader_program(
            shader_path('diamonds', 'diamonds.vert'),
            shader_path('diamonds', 'diamonds.frag')
        )
        self.loc_p = gl.glGetUniformLocation(self.shader_program, "projection")
        self.loc_v = gl.glGetUniformLocation(self.shader_program, "view")

    def get_gl_coords(self, real_x, real_y, w, h):
        if w == 0 or h == 0:
            return 0.0, 0.0
        gx = (real_x * w / MapData.REAL_WORLD_WIDTH.value) - (w / 2)
        gy = (real_y * h / MapData.REAL_WORLD_HEIGHT.value) - (h / 2)
        return gx, gy

    def update_waypoints(self, state_refs_np, attributes_np, widget_width, widget_height):
        if state_refs_np is None or state_refs_np.size == 0:
            self.num_instances = 0
            return

        N = min(state_refs_np.shape[1], self.max_instances)
        scale = 2.0
        lane_scale = 1.0
        lane_width = 0.37
        instance_idx = 0

        for i in range(N):
            # Center point
            xw, yw = state_refs_np[0, i], state_refs_np[1, i]
            gx, gy = self.get_gl_coords(xw, yw, widget_width, widget_height)
            angle = math.radians(90.0) if self.track == 'barca' else 0.0
            attr = int(attributes_np[i]) % 10
            color = self.ATTRIBUTES.get(attr, (1.0, 1.0, 0.0))
            self.instance_data[instance_idx] = [gx, gy, angle, scale, *color, 1.0]
            instance_idx += 1

            # Lane edges
            theta = state_refs_np[2, i]
            nx, ny = -np.sin(theta), np.cos(theta)
            dx, dy = (lane_width / 2) * nx, (lane_width / 2) * ny
            for sign in [+1, -1]:
                gx_lane, gy_lane = self.get_gl_coords(xw + sign * dx, yw + sign * dy, widget_width, widget_height)
                self.instance_data[instance_idx] = [gx_lane, gy_lane, 0.0, lane_scale, *self.ATTRIBUTES[10], 1.0]
                instance_idx += 1

        self.num_instances = instance_idx

        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 0, self.instance_data[:instance_idx].nbytes, self.instance_data[:instance_idx])
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    def draw(self, projection, view):
        if self.num_instances == 0:
            return

        gl.glUseProgram(self.shader_program)
        gl.glBindVertexArray(self.vao)

        gl.glUniformMatrix4fv(self.loc_p, 1, gl.GL_FALSE, glm.value_ptr(projection))
        gl.glUniformMatrix4fv(self.loc_v, 1, gl.GL_FALSE, glm.value_ptr(view))

        gl.glDrawArraysInstanced(gl.GL_TRIANGLES, 0, 6, self.num_instances)

        gl.glBindVertexArray(0)
        gl.glUseProgram(0)
