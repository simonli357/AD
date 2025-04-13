from OpenGL import GL as gl
from OpenGL.GL.shaders import compileProgram, compileShader

import os
import glob

current_dir = os.path.dirname(os.path.abspath(__file__))
asset_dir = os.path.join(current_dir, 'assets')
shader_dir = os.path.join(current_dir, 'shaders')


def create_shader_module(filepath: str, module_type: int) -> int:
    source_code = ""
    with open(filepath, "r") as file:
        source_code = file.readlines()
    return compileShader(source_code, module_type)


def create_shader_program(vertex_filepath: str, fragment_filepath: str, geometry_filepath=None) -> int:
    print(f"Compiling shader: {vertex_filepath}")
    print(f"Compiling shader: {fragment_filepath}")
    if geometry_filepath is not None:
        print(f"Compiling shader: {geometry_filepath}")
    vertex_module = create_shader_module(vertex_filepath, gl.GL_VERTEX_SHADER)
    fragment_module = create_shader_module(fragment_filepath, gl.GL_FRAGMENT_SHADER)
    modules = (vertex_module, fragment_module)
    if geometry_filepath is not None:
        modules + (geometry_filepath,)
    shader = compileProgram(*modules)
    gl.glDeleteShader(vertex_module)
    gl.glDeleteShader(fragment_module)
    return shader


def shader_path(dirname: str, filename: str):
    return os.path.join(shader_dir, dirname, filename)


def asset_path(filename: str):
    return os.path.join(asset_dir, filename)


def object_path(dirname: str, mtl_name: str):
    folder = os.path.join(asset_dir, dirname)
    # Find .obj file
    obj_candidates = glob.glob(os.path.join(folder, "*.obj"))
    obj = obj_candidates[0] if obj_candidates else None

    # Find .mtl file
    mtl_candidates = glob.glob(os.path.join(folder, f"{mtl_name}.mtl"))
    mtl = mtl_candidates[0] if mtl_candidates else None

    if obj is None or mtl is None:
        print("Error loading model")
        exit(1)

    return dirname, mtl, obj
