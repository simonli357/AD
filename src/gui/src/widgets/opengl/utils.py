from OpenGL import GL as gl
from OpenGL.GL.shaders import compileProgram, compileShader
from OpenGL.arrays import ArrayDatatype, GLintArray, GLenumArray

import os
import glob
import hashlib
import struct
import ctypes

current_dir = os.path.dirname(os.path.abspath(__file__))
asset_dir = os.path.join(current_dir, 'assets')
shader_dir = os.path.join(current_dir, 'shaders')
binary_cache_dir = os.path.join(current_dir, 'binaries')


def check_gl_error(context=""):
    error = gl.glGetError()
    if error != gl.GL_NO_ERROR:
        print(f"OpenGL error {error} in {context}")
    return error


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


def create_shader_module(filepath: str, module_type: int) -> int:
    source_code = ""
    with open(filepath, "r") as file:
        source_code = file.readlines()
    return compileShader(source_code, module_type)


def create_shader_program(vertex_filepath: str, fragment_filepath: str, geometry_filepath=None) -> int:
    # Generate content-based hash
    shader_files = [vertex_filepath, fragment_filepath]
    if geometry_filepath is not None:
        shader_files.append(geometry_filepath)

    hasher = hashlib.sha256()
    for filepath in shader_files:
        with open(filepath, 'rb') as f:
            hasher.update(f.read())
    content_hash = hasher.hexdigest()

    # Get OpenGL vendor/renderer information
    vendor = gl.glGetString(gl.GL_VENDOR).decode('utf-8')
    renderer = gl.glGetString(gl.GL_RENDERER).decode('utf-8')
    vr_hasher = hashlib.sha256(f"{vendor}{renderer}".encode()).hexdigest()[:16]
    cache_key = f"{content_hash}_{vr_hasher}"
    cache_filename = os.path.join(binary_cache_dir, f"{cache_key}.bin")

    # Try loading from cache
    if os.path.exists(cache_filename):
        program = gl.glCreateProgram()
        try:
            with open(cache_filename, 'rb') as f:
                data = f.read()
                if len(data) < 4:
                    raise ValueError("Invalid cache file")

                binary_format = struct.unpack('I', data[:4])[0]
                binary_data = data[4:]

                gl.glProgramBinary(program, binary_format, binary_data)

                # Verify program status
                link_status = gl.glGetProgramiv(program, gl.GL_LINK_STATUS)
                if link_status == gl.GL_TRUE:
                    print(f"Loaded cached shader: {cache_key}")
                    return program
                else:
                    gl.glDeleteProgram(program)
                    print("Cached shader invalid, recompiling...")
        except Exception as e:
            gl.glDeleteProgram(program)
            print(f"Error loading cached shader: {str(e)}, recompiling...")

    # Compile from source if cache miss
    print(f"Compiling shader: {vertex_filepath}")
    print(f"Compiling shader: {fragment_filepath}")
    if geometry_filepath is not None:
        print(f"Compiling shader: {geometry_filepath}")

    # Compile shaders
    vertex_module = compileShader(open(vertex_filepath).read(), gl.GL_VERTEX_SHADER)
    fragment_module = compileShader(open(fragment_filepath).read(), gl.GL_FRAGMENT_SHADER)
    modules = [vertex_module, fragment_module]

    if geometry_filepath is not None:
        geometry_module = compileShader(open(geometry_filepath).read(), gl.GL_GEOMETRY_SHADER)
        modules.append(geometry_module)

    # Link program
    program = compileProgram(*modules)

    # Cleanup shaders
    for shader in modules:
        gl.glDeleteShader(shader)

    # Save to cache
    try:
        # Get binary length using PyOpenGL's array type
        binary_length = GLintArray.zeros((1,))
        gl.glGetProgramiv(program, gl.GL_PROGRAM_BINARY_LENGTH, binary_length)

        if binary_length[0] <= 0:
            print("Empty program binary, skipping cache")
            return program

        # Prepare buffers using PyOpenGL compatible types
        binary_format = GLenumArray.zeros((1,))
        buffer = gl.arrays.GLubyteArray.zeros((binary_length[0],))

        # Get program binary with proper array handling
        actual_length = GLintArray.zeros((1,))
        gl.glGetProgramBinary(
            program,
            binary_length[0],
            actual_length,
            binary_format,
            buffer
        )

        # Verify length match
        if actual_length[0] != binary_length[0]:
            print(f"Length mismatch {actual_length[0]} vs {binary_length[0]}")
            return program

        # Save to cache
        os.makedirs(binary_cache_dir, exist_ok=True)
        with open(cache_filename, 'wb') as f:
            f.write(struct.pack('I', binary_format[0]))
            f.write(buffer.tobytes())

        print(f"Cached shader: {cache_filename}")

    except Exception as e:
        print(f"Failed to cache shader: {str(e)}")
        import traceback
        traceback.print_exc()

    return program
