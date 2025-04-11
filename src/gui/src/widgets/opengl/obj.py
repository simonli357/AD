import OpenGL.GL as gl
from OpenGL.arrays import vbo
from collections import namedtuple
from .utils import create_shader_program, shader_path, asset_dir
from .loaders import load_texture

import numpy as np
import os

MeshData = namedtuple('MeshData', ['vao', 'vbo_positions', 'vbo_texcoords', 'vbo_colors', 'vertex_count'])
Model = namedtuple('Model', ['mesh', 'texture', 'shader_program'])


def parse_mtl(mtl_filename):
    """
    Parse an .mtl file, returning a dict of:
      materials = {
         "MaterialName": {
             "Kd": (r, g, b),
             "map_Kd": "textureFile.png" or None
         },
         ...
      }
    """
    materials = {}
    current_mat = None

    with open(mtl_filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith("newmtl "):
                # e.g. "newmtl MyMaterial"
                _, mat_name = line.split(None, 1)
                current_mat = mat_name
                # Initialize defaults
                materials[current_mat] = {
                    "Kd": (1.0, 1.0, 1.0),
                    "map_Kd": None
                }

            elif current_mat is not None:  # We have a material in progress
                if line.startswith("Kd "):
                    # e.g. "Kd 0.8 0.2 0.2"
                    _, r, g, b = line.split()
                    materials[current_mat]["Kd"] = (float(r), float(g), float(b))

                elif line.startswith("map_Kd "):
                    # e.g. "map_Kd myTexture.png"
                    _, tex_path = line.split(None, 1)
                    materials[current_mat]["map_Kd"] = tex_path

    return materials


def load_obj_with_materials(obj_filename, materials):
    """
    Load a .obj, parse v, vt, faces, and track which material each face uses.
    'materials' is the dict from parse_mtl().
    Returns arrays for position, texcoord, color, etc.
    """
    vertices = []  # list of [x, y, z]
    texcoords = []  # list of [u, v]
    # We'll collect faces as a list of (pos_indices[], uv_indices[], material_name)
    faces = []

    current_material = None

    with open(obj_filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                # Skip empty lines or comments
                continue

            if line.startswith('v '):
                # Safely parse "v x y z" lines
                parts = line.split()
                # If there's NOT exactly 4 parts, skip or handle it
                if len(parts) < 4:
                    # e.g. line might just be "v" with no coords
                    print(f"Skipping incomplete vertex line: {line}")
                    continue
                # e.g. "v 1.0 2.0 3.0"
                x, y, z = parts[1], parts[2], parts[3]
                vertices.append([float(x), float(y), float(z)])

            elif line.startswith('vt '):
                # e.g. "vt u v [w]" => ignore w if present
                parts = line.split()
                if len(parts) < 3:
                    print(f"Skipping incomplete texcoord line: {line}")
                    continue
                u, v = parts[1], parts[2]
                texcoords.append([float(u), float(v)])

            elif line.startswith('usemtl '):
                # e.g. "usemtl MyMaterial"
                _, mat_name = line.split(None, 1)
                if mat_name in materials:
                    current_material = mat_name
                else:
                    current_material = None

            elif line.startswith('f '):
                # e.g. "f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3"
                face_elems = line.split()[1:]
                pos_indices = []
                uv_indices = []
                for fe in face_elems:
                    # 'fe' might look like "v_idx/vt_idx/vn_idx" or "v_idx//vn_idx" or "v_idx/vt_idx"
                    sub = fe.split('/')
                    # Position index (sub[0]) is required
                    v_idx = int(sub[0]) - 1
                    pos_indices.append(v_idx)

                    # Texcoord index (sub[1]) if present & not empty
                    t_idx = -1
                    if len(sub) > 1 and sub[1] != '':
                        t_idx = int(sub[1]) - 1
                    uv_indices.append(t_idx)

                    # Normal index (sub[2]) is ignored here, but you could parse if you need it

                faces.append((pos_indices, uv_indices, current_material))

    # Flatten face data into final arrays
    position_data = []
    texcoord_data = []
    color_data = []

    for (pos_indices, uv_indices, mat_name) in faces:
        # If we have a valid material name, get its color from MTL
        if mat_name and mat_name in materials:
            r, g, b = materials[mat_name]["Kd"]
        else:
            r, g, b = 1.0, 1.0, 1.0  # fallback color

        for i in range(len(pos_indices)):
            px, py, pz = vertices[pos_indices[i]]
            position_data.extend([px, py, pz])

            if uv_indices[i] != -1 and uv_indices[i] < len(texcoords):
                u, v = texcoords[uv_indices[i]]
            else:
                u, v = 0.0, 0.0
            texcoord_data.extend([u, v])

            # Add color for each vertex
            color_data.extend([r, g, b])

    # Convert to numpy arrays
    position_array = np.array(position_data, dtype=np.float32)
    texcoord_array = np.array(texcoord_data, dtype=np.float32)
    color_array = np.array(color_data, dtype=np.float32)

    return position_array, texcoord_array, color_array


def create_mesh(position_array, texcoord_array, color_array):
    """Given raw float arrays for positions, texture coords, colors, build a VAO."""
    vao_id = gl.glGenVertexArrays(1)
    gl.glBindVertexArray(vao_id)

    # Positions => layout(location=0)
    vbo_positions = vbo.VBO(position_array)
    vbo_positions.bind()
    gl.glEnableVertexAttribArray(0)
    gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, 0, None)
    vbo_positions.unbind()

    # TexCoords => layout(location=1)
    vbo_texcoords = vbo.VBO(texcoord_array)
    vbo_texcoords.bind()
    gl.glEnableVertexAttribArray(1)
    gl.glVertexAttribPointer(1, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, None)
    vbo_texcoords.unbind()

    # Colors => layout(location=2)
    vbo_colors = vbo.VBO(color_array)
    vbo_colors.bind()
    gl.glEnableVertexAttribArray(2)
    gl.glVertexAttribPointer(2, 3, gl.GL_FLOAT, gl.GL_FALSE, 0, None)
    vbo_colors.unbind()

    gl.glBindVertexArray(0)

    vertex_count = len(position_array) // 3
    return MeshData(vao_id, vbo_positions, vbo_texcoords, vbo_colors, vertex_count)


def load_obj(dirname, mtl_path, obj_path):
    """
    High-level function:
    1) Parse MTL for materials (colors + optional texture)
    2) Parse OBJ for geometry + which material each face uses
    3) Create buffers + load texture if any
    4) Return Model namedtuple
    """
    # 1) Parse all materials
    materials = parse_mtl(mtl_path)

    # 2) Parse geometry with material references
    position_array, texcoord_array, color_array = load_obj_with_materials(obj_path, materials)

    # 3) Create the mesh
    mesh_data = create_mesh(position_array, texcoord_array, color_array)

    # If there's exactly one material with a map_Kd, you could do:
    #   texture_path = next((m["map_Kd"] for m in materials.values() if m["map_Kd"]), None)
    # Or handle multi-texture differently. For simplicity, let’s pick the first non-empty map_Kd:
    texture_path = None
    for mat_name, mat_info in materials.items():
        if mat_info["map_Kd"] is not None:
            texture_path = mat_info["map_Kd"]
            break

    if texture_path:
        texture_path = os.path.join(asset_dir, dirname, texture_path)

    texture_id = load_texture(texture_path) if texture_path else None

    shader_program = create_shader_program(shader_path('model', 'model.vert'), shader_path('model', 'model.frag'))

    return Model(mesh_data, texture_id, shader_program)
