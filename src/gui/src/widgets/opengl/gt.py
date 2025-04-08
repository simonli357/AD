import OpenGL.GL as gl
from .shader import create_shader_program, shader_path
from ..enums import MapData

import numpy as np
import glm


class GTRenderer:
    def __init__(self, model, entity_type, category=""):
        """
        Initialize the instance renderer using a loaded Model.
        The model is expected to come from your OBJ loader and contains:
          - mesh: which holds the base VAO and vertex count (attributes 0: position, 1: texcoord, 2: color)
          - texture: an OpenGL texture id (or None)
          - shader_program: we override this by loading our instancing shader below
        """
        self.model = model
        self.entity_type = entity_type
        self.category = category
        self.num_instances = 0
        # Use the mesh's VAO and vertex count for instancing
        self.vao = model.mesh.vao
        self.vertex_count = model.mesh.vertex_count

        # Load the instanced shader (models.vert/models.frag)
        self.shader_program = create_shader_program(
            shader_path('models', 'models.vert'),
            shader_path('models', 'models.frag')
        )

        # Create an instance VBO to hold per-instance model matrices
        self.instance_vbo = gl.glGenBuffers(1)
        gl.glBindVertexArray(self.vao)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
        # Allocate with no data initially; we'll update it later with instance matrices.
        gl.glBufferData(gl.GL_ARRAY_BUFFER, 0, None, gl.GL_DYNAMIC_DRAW)

        # A 4x4 matrix occupies 16 floats, and we use 4 attribute locations for it (locations 3,4,5,6).
        stride = 16 * 4  # 16 floats * 4 bytes per float
        for i in range(4):
            attrib_location = 3 + i
            gl.glEnableVertexAttribArray(attrib_location)
            # Each attribute is a vec4: 4 floats. The offset for each vec4 is i * sizeof(vec4)
            gl.glVertexAttribPointer(attrib_location, 4, gl.GL_FLOAT, gl.GL_FALSE, stride, gl.ctypes.c_void_p(i * 4 * 4))
            # Set the divisor to 1 so that these attributes change per instance, not per vertex.
            gl.glVertexAttribDivisor(attrib_location, 1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
        gl.glBindVertexArray(0)

    def get_gl_coords(self, real_x, real_y, widget_width, widget_height):
        """
        Convert real-world coordinates to OpenGL coordinates.
        Adjusts x and y based on widget dimensions and the map size.
        """
        world_x = real_x / MapData.REAL_WORLD_WIDTH.value * widget_width
        world_y = (MapData.REAL_WORLD_HEIGHT.value - real_y) / MapData.REAL_WORLD_HEIGHT.value * widget_height
        return world_x, world_y

    def update_data(self, data, widget_width, widget_height):
        """
        For each row in the provided data (for example a DataFrame iterator),
        compute a model matrix based on its position and orientation.
        Then, update the instance VBO with the flattened matrices.

        'data' is assumed to be an iterable of (index, row) pairs, where row has:
           - 'X': the world x-coordinate,
           - 'Y': the world y-coordinate,
           - 'Orientation': the object's orientation (in radians).
        """
        instance_matrices = []
        for index, row in data:
            # Compute OpenGL coordinates
            # (Note: In your original code you call get_gl_coords with Y inverted.)
            x, y = self.get_gl_coords(row['X'], MapData.REAL_WORLD_HEIGHT.value - row['Y'], widget_width, widget_height)
            m_type, orientation = row['Type'], row['Orientation']

            if self.entity_type != m_type:
                continue

            # Build a model transformation matrix:
            model_matrix = glm.mat4(1.0)
            model_matrix = glm.translate(model_matrix, glm.vec3(x, y, 0.0))
            model_matrix = glm.rotate(model_matrix, orientation, glm.vec3(0.0, 0.0, 1.0))

            if self.entity_type == 'Car':
                model_matrix = glm.scale(model_matrix, glm.vec3(0.2, 0.2, 0.2))
            if self.category == 'Sign':
                model_matrix = glm.scale(model_matrix, glm.vec3(16.0, 16.0, 16.0))
            if self.entity_type == 'Destination':
                model_matrix = glm.scale(model_matrix, glm.vec3(2.0, 2.0, 2.0))

            # Convert the matrix to a numpy array.
            # Because glm (PyGLM) produces column-major matrices (which OpenGL expects),
            # we use .T.flatten() to ensure the data are in the right order.
            matrix_np = np.array(model_matrix, dtype=np.float32).T.flatten()
            instance_matrices.append(matrix_np)

        if instance_matrices:
            instance_matrices = np.array(instance_matrices, dtype=np.float32)
            self.num_instances = len(instance_matrices)
            # Upload the instance matrices to the instance VBO
            gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.instance_vbo)
            gl.glBufferData(gl.GL_ARRAY_BUFFER, instance_matrices.nbytes, instance_matrices, gl.GL_DYNAMIC_DRAW)
            gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
        else:
            self.num_instances = 0

    def draw(self, projection, view):
        """
        Render all instances with a single draw call.
        Binds the instanced shader, sets the view and projection matrices,
        binds the texture (if any), and then calls glDrawArraysInstanced.
        """
        if self.num_instances == 0:
            return

        gl.glUseProgram(self.shader_program)
        gl.glBindVertexArray(self.vao)

        # Set projection and view uniforms
        proj_location = gl.glGetUniformLocation(self.shader_program, "projection")
        view_location = gl.glGetUniformLocation(self.shader_program, "view")
        gl.glUniformMatrix4fv(proj_location, 1, gl.GL_FALSE, glm.value_ptr(projection))
        gl.glUniformMatrix4fv(view_location, 1, gl.GL_FALSE, glm.value_ptr(view))

        # Bind texture if available and set the uniform
        if self.model.texture:
            gl.glUniform1i(gl.glGetUniformLocation(self.shader_program, "hasTexture"), True)
            gl.glActiveTexture(gl.GL_TEXTURE0)
            gl.glBindTexture(gl.GL_TEXTURE_2D, self.model.texture)
            gl.glUniform1i(gl.glGetUniformLocation(self.shader_program, "uTexture"), 0)
        else:
            gl.glUniform1i(gl.glGetUniformLocation(self.shader_program, "hasTexture"), False)

        # Draw the instanced geometry:
        gl.glDrawArraysInstanced(gl.GL_TRIANGLES, 0, self.vertex_count, self.num_instances)

        gl.glBindVertexArray(0)
        gl.glUseProgram(0)
