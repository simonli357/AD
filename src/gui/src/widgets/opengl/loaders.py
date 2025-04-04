from OpenGL import GL as gl
from OpenGL.arrays import vbo
from collections import namedtuple
from PIL import Image

import numpy as np

Model = namedtuple('Model', ['vertices', 'faces', 'vbo', 'vertex_count'])
Material = namedtuple('Material', ['texture_id', 'vao', 'vbo', 'ebo', 'vertex_count'])
Mesh = namedtuple('Mesh', ['vao', 'vbo', 'vertex_count'])


def load_obj(model_path) -> Model:
    vertices = []
    faces = []
    with open(model_path, 'r') as f:
        for line in f:
            if line.startswith('v '):
                vertices.append(list(map(float, line.strip().split()[1:4])))
            elif line.startswith('f '):
                faces.append([int(v.split('/')[0]) - 1 for v in line.strip().split()[1:]])

    # Convert to flat array of vertices
    vertex_data = []
    for face in faces:
        for v_idx in face:
            vertex_data.extend(vertices[v_idx])

    vertex_array = np.array(vertex_data, dtype=np.float32)
    model_vbo = vbo.VBO(vertex_array)
    return Model(vertices=vertices, faces=faces, vbo=model_vbo, vertex_count=len(vertex_data) // 3)


def load_texture(filename):
    """Load PNG image as texture using Pillow"""
    try:
        image = Image.open(filename)
        if image.mode == 'RGB':
            gl_format = gl.GL_RGB
        else:
            image = image.convert("RGBA")
            gl_format = gl.GL_RGBA

        width, height = image.size
        # Flip image vertically (OpenGL expects origin at bottom-left)
        image_data = image.transpose(Image.FLIP_TOP_BOTTOM).tobytes()
        image.close()

        # Generate OpenGL texture
        texture_id = gl.glGenTextures(1)
        gl.glBindTexture(gl.GL_TEXTURE_2D, texture_id)

        # Set texture parameters
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_REPEAT)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_REPEAT)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_NEAREST)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)

        # Upload texture data
        gl.glTexImage2D(
            gl.GL_TEXTURE_2D, 0, gl_format, width, height, 0,
            gl_format, gl.GL_UNSIGNED_BYTE, image_data
        )
        gl.glGenerateMipmap(gl.GL_TEXTURE_2D)

        # Unbind texture
        gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
        return texture_id
    except Exception as e:
        print(f"Error loading texture {filename}: {e}")
        return 0


def load_mesh(filename):
    vertices = []
    faces = []
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('v '):
                vertices.append(list(map(float, line.strip().split()[1:4])))
            elif line.startswith('f '):
                faces.append([int(v.split('/')[0]) - 1 for v in line.strip().split()[1:]])

    # Convert to flat array of vertices
    vertex_data = []
    for face in faces:
        for v_idx in face:
            vertex_data.extend(vertices[v_idx])

    vertex_array = np.array(vertex_data, dtype=np.float32)
    mesh_vbo = vbo.VBO(vertex_array)
    vao = gl.glGenVertexArrays(1)
    gl.glBindVertexArray(vao)
    mesh_vbo.bind()
    gl.glEnableVertexAttribArray(0)
    gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, 0, mesh_vbo)
    gl.glBindVertexArray(0)
    mesh_vbo.unbind()
    return Mesh(
        vao=vao,
        vbo=mesh_vbo,
        vertex_count=len(vertex_data) // 3
    )


def load_material(filename):
    """Load texture material with proper VBO/VAO management"""
    try:
        # Load texture first
        texture_id = load_texture(filename)

        # Vertex data: positions + texture coordinates
        vertices = np.array([
            # Positions   # Texture Coords
            0.0, 0.0, 0.0, 0.0, 0.0,  # Bottom-left
            1.0, 0.0, 0.0, 1.0, 0.0,   # Bottom-right
            1.0, 1.0, 0.0, 1.0, 1.0,    # Top-right
            0.0, 1.0, 0.0, 0.0, 1.0    # Top-left
        ], dtype=np.float32)

        # Index data for triangles
        indices = np.array([0, 1, 2, 0, 2, 3], dtype=np.uint32)

        # Create VBO objects
        vertex_vbo = vbo.VBO(vertices)
        index_vbo = vbo.VBO(
            indices,
            target=gl.GL_ELEMENT_ARRAY_BUFFER,
            usage=gl.GL_STATIC_DRAW
        )

        # Create and configure VAO
        vao = gl.glGenVertexArrays(1)
        gl.glBindVertexArray(vao)

        # Configure vertex attributes
        vertex_vbo.bind()
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(
            0, 3, gl.GL_FLOAT, gl.GL_FALSE,
            5 * 4,  # Stride (5 floats * 4 bytes each)
            vertex_vbo
        )

        gl.glEnableVertexAttribArray(1)
        gl.glVertexAttribPointer(
            1, 2, gl.GL_FLOAT, gl.GL_FALSE,
            5 * 4,
            vertex_vbo
        )

        # Bind element buffer
        index_vbo.bind()

        # Cleanup state
        gl.glBindVertexArray(0)
        vertex_vbo.unbind()
        index_vbo.unbind()

        return Material(
            texture_id=texture_id,
            vao=vao,
            vbo=vertex_vbo,
            ebo=index_vbo,
            vertex_count=6
        )

    except Exception as e:
        print(f"Material load failed: {e}")
        # Cleanup resources if created
        if 'vertex_vbo' in locals():
            vertex_vbo.delete()
        if 'index_vbo' in locals():
            index_vbo.delete()
        if 'vao' in locals():
            gl.glDeleteVertexArrays(1, [vao])
        return None
