from OpenGL import GL as gl
from OpenGL.arrays import vbo
from .shader import create_shader_program, shader_path
from ..enums import MapData
import numpy as np
import glm


class DestinationsRenderer:
    def __init__(self):
        self.num_instances = 0
        self.vao = gl.glGenVertexArrays(1)

        # Circle geometry
        vertices = [[0.0, 0.0]]  # center vertex
        for i in range(65):
            angle = 6.28318530718 * float(i) / 64.0
            vertices.append([np.cos(angle), np.sin(angle)])

        self.base_vertices = np.array(vertices, dtype=np.float32)
        self.num_vertices = len(vertices)

        self.base_vbo = vbo.VBO(self.base_vertices)
        self.instance_vbo = vbo.VBO(np.array([], dtype=np.float32), usage=gl.GL_DYNAMIC_DRAW)

        # VAO setup
        gl.glBindVertexArray(self.vao)

        # Base vertex attributes (position)
        self.base_vbo.bind()
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))

        # Instance vertex attributes
        self.instance_vbo.bind()

        for i in range(4):
            index = 1 + i
            gl.glEnableVertexAttribArray(index)
            gl.glVertexAttribPointer(index, 4, gl.GL_FLOAT, gl.GL_FALSE, 64, gl.ctypes.c_void_p(i * 16))
            gl.glVertexAttribDivisor(index, 1)

        gl.glBindVertexArray(0)
        self.base_vbo.unbind()
        self.instance_vbo.unbind()

        self.shader_program = create_shader_program(
            shader_path('circles', 'circles.vert'),
            shader_path('circles', 'circles.frag')
        )

    def get_gl_coords(self, real_x, real_y, widget_width, widget_height):
        if widget_height == 0 or widget_width == 0:
            return (0.0, 0.0)

        # Convert real-world to OpenGL world coordinates
        world_x = (real_x * widget_width / MapData.REAL_WORLD_WIDTH.value) - (widget_width / 2)
        world_y = (real_y * widget_height / MapData.REAL_WORLD_HEIGHT.value) - (widget_height / 2)

        return world_x, world_y

    def update_data(self, data, widget_width, widget_height):
        if data is None:
            self.num_instances = 0
            return
        instance_matrices = []
        for index, row in data:
            # Compute OpenGL coordinates
            # (Note: In your original code you call get_gl_coords with Y inverted.)
            x, y = self.get_gl_coords(row['X'], row['Y'], widget_width, widget_height)
            m_type, orientation = row['Type'], row['Orientation']

            if m_type != 'Destination':
                continue

            # Build a model transformation matrix:
            model_matrix = glm.mat4(1.0)
            model_matrix = glm.translate(model_matrix, glm.vec3(x, y, 0.1))
            model_matrix = glm.rotate(model_matrix, orientation, glm.vec3(0.0, 0.0, 1.0))
            model_matrix = glm.scale(model_matrix, glm.vec3(8.0, 8.0, 8.0))

            # Convert the matrix to a numpy array.
            # Because glm (PyGLM) produces column-major matrices (which OpenGL expects),
            # we use .T.flatten() to ensure the data are in the right order.
            matrix_np = np.array(model_matrix, dtype=np.float32).T.flatten()
            instance_matrices.append(matrix_np)

        if instance_matrices:
            instance_matrices = np.array(instance_matrices, dtype=np.float32)
            self.num_instances = len(instance_matrices)
            # Upload the instance matrices to the instance VBO
            self.instance_vbo.bind()
            gl.glBufferData(gl.GL_ARRAY_BUFFER, instance_matrices.nbytes, instance_matrices, gl.GL_DYNAMIC_DRAW)
            self.instance_vbo.unbind()
        else:
            self.num_instances = 0

    def draw(self, color, projection, view):
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
        gl.glUniform4fv(gl.glGetUniformLocation(self.shader_program, "color"), 1, color)

        # Draw all instances
        gl.glDrawArraysInstanced(gl.GL_TRIANGLE_FAN, 0, self.num_vertices, self.num_instances)

        gl.glBindVertexArray(0)
        gl.glUseProgram(0)
