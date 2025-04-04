from OpenGL import GL as gl
from OpenGL.arrays import vbo
from .shader import create_shader_program, shader_path
from ..enums import MapData
import numpy as np
import glm


class WaypointsRenderer:
    def __init__(self):
        self.vao = None
        self.base_vbo = None
        self.instance_vbo = None
        self.num_instances = 0

        # Diamond geometry (2 triangles)
        self.base_vertices = np.array([
            # Positions (3D for proper matrix transformations)
            [0.0, 0.5, 0.1],  # Top
            [0.5, 0.0, 0.1],  # Right
            [-0.5, 0.0, 0.1],  # Left
            [-0.5, 0.0, 0.1],  # Left
            [0.5, 0.0, 0.1],  # Right
            [0.0, -0.5, 0.1],  # Bottom
        ], dtype=np.float32).flatten()

        self.ATTRIBUTES = {  # (Same color definitions as before)
            0: (1.0, 1.0, 0.0),   # Yellow
            1: (0.0, 1.0, 0.0),    # Green
            2: (0.0, 0.0, 1.0),    # Blue
            3: (1.0, 0.5, 0.0),    # Orange
            4: (0.5, 0.0, 0.5),    # Purple
            5: (0.8, 0.7, 1.0),    # Light pink
            6: (1.0, 1.0, 1.0),    # White
            7: (0.0, 1.0, 1.0),    # Cyan
            8: (0.4, 0.5, 0.7),    # Steel blue
            9: (0.5, 0.0, 0.5),    # Purple
        }

        self.shader_program = create_shader_program(
            shader_path('diamond', 'diamond.vert'),
            shader_path('diamond', 'diamond.frag')
        )

    def get_gl_coords(self, real_x, real_y, widget_width, widget_height):
        if widget_height == 0 or widget_width == 0:
            return (0.0, 0.0)

        # Convert real-world to OpenGL world coordinates
        world_x = (real_x * widget_width / MapData.REAL_WORLD_WIDTH.value) - (widget_width / 2)
        world_y = (real_y * widget_height / MapData.REAL_WORLD_HEIGHT.value) - (widget_height / 2)

        return world_x, world_y

    # def get_gl_coords(self, real_x, real_y):
    #     # Map to [-1, 1] range
    #     gl_x = (2 * real_x / MapData.REAL_WORLD_WIDTH.value) - 1
    #     gl_y = (2 * real_y / MapData.REAL_WORLD_HEIGHT.value) - 1
    #     return gl_x, gl_y

    def setup_vao(self, instance_array):
        self.vao = gl.glGenVertexArrays(1)
        gl.glBindVertexArray(self.vao)

        # Base vertex data (diamond shape)
        self.base_vbo = vbo.VBO(self.base_vertices)
        self.base_vbo.bind()

        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, 3 * 4, gl.ctypes.c_void_p(0))

        self.base_vbo.unbind()

        # Instance VBO (color + model matrix)
        if self.instance_vbo is not None:
            self.instance_vbo.delete()

        self.instance_vbo = vbo.VBO(instance_array)
        self.instance_vbo.bind()

        # Color attribute (vec3)
        gl.glEnableVertexAttribArray(1)
        gl.glVertexAttribPointer(1, 3, gl.GL_FLOAT, gl.GL_FALSE, 19 * 4, gl.ctypes.c_void_p(0))
        gl.glVertexAttribDivisor(1, 1)

        # Model matrix attributes (4 vec4s)
        for i in range(4):
            gl.glEnableVertexAttribArray(2 + i)
            offset = 12 + i * 16  # 3 floats (color) * 4 + i * 16
            gl.glVertexAttribPointer(2 + i, 4, gl.GL_FLOAT, gl.GL_FALSE, 19 * 4, gl.ctypes.c_void_p(offset))
            gl.glVertexAttribDivisor(2 + i, 1)

        self.instance_vbo.unbind()

        gl.glBindVertexArray(0)

    def update_waypoints(self, state_refs_np, attributes_np, widget_width, widget_height):
        if state_refs_np is None or state_refs_np.size == 0:
            self.num_instances = 0
            return

        self.num_instances = state_refs_np.shape[1]
        instance_data = []
        scale = 10.0  # Fixed scale factor

        for i in range(self.num_instances):
            x, y = self.get_gl_coords(state_refs_np[0, i], state_refs_np[1, i], widget_width, widget_height)

            attr = int(attributes_np[i]) % 10
            color = self.ATTRIBUTES.get(attr, (1.0, 1.0, 0.0))

            # Create model matrix
            model = glm.translate(glm.mat4(1.0), glm.vec3(x, y, 0.0))
            model = glm.scale(model, glm.vec3(scale, scale, 1.0))
            model_data = glm.value_ptr(model)

            # Add color and matrix data
            instance_data.extend(color)
            instance_data.extend(model_data[:16])

        instance_array = np.array(instance_data, dtype=np.float32)
        print("First instance data:", instance_array[:19])
        self.setup_vao(instance_array.flatten())

    def draw(self, projection, view):
        if self.num_instances == 0:
            return

        gl.glUseProgram(self.shader_program)
        gl.glBindVertexArray(self.vao)

        # Set matrices
        gl.glUniformMatrix4fv(
            gl.glGetUniformLocation(self.shader_program, "projection"),
            1, gl.GL_FALSE, glm.value_ptr(projection)
        )
        gl.glUniformMatrix4fv(
            gl.glGetUniformLocation(self.shader_program, "view"),
            1, gl.GL_FALSE, glm.value_ptr(view)
        )

        # Draw all instances
        gl.glDrawArraysInstanced(gl.GL_TRIANGLES, 0, 6, self.num_instances)

        gl.glBindVertexArray(0)
        gl.glUseProgram(0)
