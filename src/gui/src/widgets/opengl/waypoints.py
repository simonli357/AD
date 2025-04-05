from OpenGL import GL as gl
from OpenGL.arrays import vbo
from .shader import create_shader_program, shader_path
from ..enums import MapData
import numpy as np
import glm


class WaypointsRenderer:
    def __init__(self, track):
        self.track = track
        self.num_instances = 0
        self.vao = gl.glGenVertexArrays(1)
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
        self.base_vbo = vbo.VBO(self.base_vertices)
        self.instance_vbo = vbo.VBO(np.array([], dtype=np.float32), usage=gl.GL_DYNAMIC_DRAW)

        # VAO setup
        gl.glBindVertexArray(self.vao)

        # Base vertex attributes (position)
        self.base_vbo.bind()
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, 0, None)

        # Instance vertex attributes (color + matrix)
        self.instance_vbo.bind()
        # Color attribute
        gl.glEnableVertexAttribArray(1)
        gl.glVertexAttribPointer(1, 4, gl.GL_FLOAT, gl.GL_FALSE, 20 * 4, None)
        gl.glVertexAttribDivisor(1, 1)

        # Matrix columns
        for i in range(4):
            gl.glEnableVertexAttribArray(2 + i)
            gl.glVertexAttribPointer(2 + i, 4, gl.GL_FLOAT, gl.GL_FALSE, 20 * 4, gl.ctypes.c_void_p(16 + i * 16))
            gl.glVertexAttribDivisor(2 + i, 1)

        gl.glBindVertexArray(0)
        self.base_vbo.unbind()
        self.instance_vbo.unbind()

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

    def update_waypoints(self, state_refs_np, attributes_np, widget_width, widget_height):
        if state_refs_np is None or state_refs_np.size == 0:
            self.num_instances = 0
            return

        self.num_instances = state_refs_np.shape[1]
        instance_data = []
        scale = 2.0  # Scale

        for i in range(self.num_instances):
            x, y = self.get_gl_coords(state_refs_np[0, i], state_refs_np[1, i], widget_width, widget_height)

            attr = int(attributes_np[i]) % 10
            color = self.ATTRIBUTES.get(attr, (1.0, 1.0, 0.0))

            # Create model matrix
            model = glm.mat4(1.0)
            if self.track == 'bfmc':
                model = glm.translate(model, glm.vec3(x, y, 0.0))
                model = glm.scale(model, glm.vec3(scale, scale, 1.0))
            if self.track == 'barca':
                x, y = state_refs_np[0, i], state_refs_np[1, i]
                model = glm.rotate(model, glm.radians(90.0), glm.vec3(0, 0, 1))
                model = glm.translate(model, glm.vec3(x, y, 0.0))
                model = glm.scale(model, glm.vec3(0.05, 0.05, 1.0))
            model_data = glm.value_ptr(model)

            # Add color and matrix data
            instance_data.extend([color[0], color[1], color[2], 1.0])
            instance_data.extend(model_data[:16])

        instance_array = np.array(instance_data, dtype=np.float32)

        # Update VBO data without recreating objects
        self.instance_vbo.set_array(instance_array)
        self.num_instances = state_refs_np.shape[1]

        # Ensure proper buffer binding
        self.instance_vbo.bind()
        gl.glBufferData(
            gl.GL_ARRAY_BUFFER,
            instance_array.nbytes,
            instance_array,
            gl.GL_DYNAMIC_DRAW
        )
        self.instance_vbo.unbind()

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
