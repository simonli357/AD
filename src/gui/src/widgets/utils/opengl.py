from OpenGL import GL as gl
from OpenGL.arrays import vbo
from collections import namedtuple
from PIL import Image

import numpy as np

Model = namedtuple('Model', ['vertices', 'faces', 'vbo', 'vertex_count'])


def qt_save_gl_state():
    gl.glPushClientAttrib(gl.GL_CLIENT_ALL_ATTRIB_BITS)
    gl.glPushAttrib(gl.GL_ALL_ATTRIB_BITS)
    gl.glMatrixMode(gl.GL_TEXTURE)
    gl.glPushMatrix()
    gl.glLoadIdentity()
    gl.glMatrixMode(gl.GL_PROJECTION)
    gl.glPushMatrix()
    gl.glMatrixMode(gl.GL_MODELVIEW)
    gl.glPushMatrix()

    gl.glShadeModel(gl.GL_FLAT)
    gl.glDisable(gl.GL_CULL_FACE)
    gl.glDisable(gl.GL_LIGHTING)
    gl.glDisable(gl.GL_STENCIL_TEST)
    gl.glDisable(gl.GL_DEPTH_TEST)
    gl.glEnable(gl.GL_BLEND)
    gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)


def qt_restore_gl_state():
    gl.glMatrixMode(gl.GL_TEXTURE)
    gl.glPopMatrix()
    gl.glMatrixMode(gl.GL_PROJECTION)
    gl.glPopMatrix()
    gl.glMatrixMode(gl.GL_MODELVIEW)
    gl.glPopMatrix()
    gl.glPopAttrib()
    gl.glPopClientAttrib()


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
    return Model(vertices=vertices, faces=faces, vbo=model_vbo,
                 vertex_count=len(vertex_data) // 3)


def load_texture(filename):
    """Load PNG image as texture using Pillow"""
    try:
        image = Image.open(filename)
        image = image.convert("RGBA")
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
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR_MIPMAP_LINEAR)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)

        # Upload texture data
        gl.glTexImage2D(
            gl.GL_TEXTURE_2D, 0, gl.GL_RGBA, width, height, 0,
            gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, image_data
        )
        gl.glGenerateMipmap(gl.GL_TEXTURE_2D)

        # Unbind texture
        gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
        return texture_id
    except Exception as e:
        print(f"Error loading texture {filename}: {e}")
        return 0
