import OpenGL.GL as gl
from OpenGL.arrays import vbo
from collections import namedtuple
from PIL import Image
from .shader import create_shader_program, shader_path

import numpy as np

Mesh = namedtuple('Mesh', ['vao', 'vbo', 'ebo', 'vertex_count'])
Model = namedtuple('Model', ['mesh', 'texture', 'shader_program'])


def load_mtl(mtl_filename):
    """
    Very basic MTL parser:
    Returns the first 'map_Kd' texture path found, or None if not found.
    """
    texture_path = None
    with open(mtl_filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('map_Kd'):
                parts = line.split()
                if len(parts) >= 2:
                    texture_path = parts[1]
                    break
    return texture_path


def load_mesh(obj_filename):
    """
    Loads positions (v) and texcoords (vt) from a .obj file.
    Returns a Mesh containing VAO, VBOs, and vertex count.
    """
    vertices = []      # list of [x, y, z]
    texcoords = []     # list of [u, v]
    faces_pos = []     # list of int indices for positions
    faces_uv = []      # list of int indices for texcoords

    with open(obj_filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('v '):
                # e.g. v 1.0 2.0 3.0
                _, x, y, z = line.split()
                vertices.append([float(x), float(y), float(z)])
            elif line.startswith('vt '):
                # e.g. vt 0.5 1.0
                parts = line.split()
                # Sometimes there's a third component (u, v, w), ignoring w
                _, u, v = parts[:3]
                texcoords.append([float(u), float(v)])
            elif line.startswith('f '):
                # e.g. f v1/vt1 v2/vt2 v3/vt3
                face_elems = line.split()[1:]
                pos_indices = []
                uv_indices = []
                for fe in face_elems:
                    # Typical .obj face element: vIndex/vtIndex(/vnIndex)
                    sub = fe.split('/')
                    # Convert to zero-based
                    v_idx = int(sub[0]) - 1
                    pos_indices.append(v_idx)

                    if len(sub) > 1 and sub[1] != '':
                        t_idx = int(sub[1]) - 1
                    else:
                        t_idx = -1
                    uv_indices.append(t_idx)

                faces_pos.append(pos_indices)
                faces_uv.append(uv_indices)

    # Flatten into "triangle soup"
    position_data = []
    texcoord_data = []

    for i in range(len(faces_pos)):
        pos_indices = faces_pos[i]
        uv_indices = faces_uv[i]
        for j in range(len(pos_indices)):
            # Position
            vx, vy, vz = vertices[pos_indices[j]]
            position_data.extend([vx, vy, vz])

            # TexCoord
            if uv_indices[j] != -1 and uv_indices[j] < len(texcoords):
                u, v = texcoords[uv_indices[j]]
            else:
                u, v = 0.0, 0.0
            texcoord_data.extend([u, v])

    position_array = np.array(position_data, dtype=np.float32)
    texcoord_array = np.array(texcoord_data, dtype=np.float32)

    # Set up VBOs
    vbo_positions = vbo.VBO(position_array)
    vbo_texcoords = vbo.VBO(texcoord_array)

    # Create VAO
    vao_id = gl.glGenVertexArrays(1)
    gl.glBindVertexArray(vao_id)

    # Positions => layout(location=0)
    vbo_positions.bind()
    gl.glEnableVertexAttribArray(0)
    gl.glVertexAttribPointer(
        0, 3, gl.GL_FLOAT, gl.GL_FALSE, 0, None
    )

    # TexCoords => layout(location=1)
    vbo_texcoords.bind()
    gl.glEnableVertexAttribArray(1)
    gl.glVertexAttribPointer(
        1, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, None
    )

    # Unbind VAO
    gl.glBindVertexArray(0)
    vbo_positions.unbind()
    vbo_texcoords.unbind()

    vertex_count = len(position_data) // 3
    return Mesh(vao_id, vbo_positions, vbo_texcoords, vertex_count)


def load_texture(image_path):
    """
    Loads an image file (using Pillow) into an OpenGL texture.
    Returns the texture ID.
    """
    # Open with Pillow
    img = Image.open(image_path).convert('RGBA')
    img_data = img.tobytes("raw", "RGBA", 0, -1)
    width, height = img.size

    # Create a new OpenGL texture
    tex_id = gl.glGenTextures(1)
    gl.glBindTexture(gl.GL_TEXTURE_2D, tex_id)

    # Set some default texture parameters
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_REPEAT)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_REPEAT)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)

    # Upload pixel data
    gl.glTexImage2D(
        gl.GL_TEXTURE_2D,
        0,
        gl.GL_RGBA,
        width,
        height,
        0,
        gl.GL_RGBA,
        gl.GL_UNSIGNED_BYTE,
        img_data
    )

    # Unbind
    gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
    return tex_id


def load_obj(mtl, obj) -> Model:
    texture_path = load_mtl(mtl)
    mesh = load_mesh(obj)
    texture_id = None
    if texture_path is not None:
        texture_id = load_texture(texture_path)
    shader_program = create_shader_program(shader_path('model', 'model.vert'), shader_path('model', 'model.frag'))
    return Model(mesh, texture_id, shader_program)
