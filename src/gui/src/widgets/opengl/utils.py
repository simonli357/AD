from OpenGL import GL as gl
from OpenGL.GL.shaders import compileProgram, compileShader
from OpenGL.arrays import GLintArray, GLenumArray
from pathlib import Path

import os
import glob
import hashlib
import struct
import json

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


def get_file_signature(filepath: str) -> dict:
    """Get both content hash and metadata for cache validation"""
    stat = os.stat(filepath)
    with open(filepath, 'rb') as f:
        content_hash = hashlib.sha256(f.read()).hexdigest()
    return {
        'path': filepath,
        'content_hash': content_hash,
        'size': stat.st_size,
        'mtime': stat.st_mtime,
        'ino': stat.st_ino,
    }


def compile_shaders(vertex_filepath: str, fragment_filepath: str, geometry_filepath=None) -> int:
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


def load_cached_program(cache_path: Path) -> int:
    """Load program from cache file with validation"""
    with open(cache_path, 'rb') as f:
        # Skip metadata line
        f.readline()
        data = f.read()

    program = gl.glCreateProgram()
    binary_format = GLenumArray.zeros((1,))
    binary_format[0] = struct.unpack('I', data[:4])[0]

    # Validate binary format support
    num_formats = GLintArray.zeros((1,))
    gl.glGetIntegerv(gl.GL_NUM_PROGRAM_BINARY_FORMATS, num_formats)
    if num_formats[0] < 1:
        return None

    gl.glProgramBinary(program, binary_format[0], data[4:], len(data[4:]))

    # Validate program
    link_status = GLintArray.zeros((1,))
    gl.glGetProgramiv(program, gl.GL_LINK_STATUS, link_status)
    if link_status[0] != gl.GL_TRUE:
        gl.glDeleteProgram(program)
        return None

    return program


def save_program_cache(program: int, cache_path: Path, signatures: list):
    """Save compiled program to cache with metadata"""
    binary_length = GLintArray.zeros((1,))
    gl.glGetProgramiv(program, gl.GL_PROGRAM_BINARY_LENGTH, binary_length)

    if binary_length[0] <= 0:
        return

    binary_format = GLenumArray.zeros((1,))
    buffer = gl.arrays.GLubyteArray.zeros((binary_length[0],))
    actual_length = GLintArray.zeros((1,))

    gl.glGetProgramBinary(
        program,
        binary_length[0],
        actual_length,
        binary_format,
        buffer
    )

    # Create metadata header
    meta = {
        'driver': {
            'vendor': gl.glGetString(gl.GL_VENDOR).decode(),
            'renderer': gl.glGetString(gl.GL_RENDERER).decode(),
        },
        'files': signatures,
    }

    # Write to cache
    cache_path.parent.mkdir(exist_ok=True, parents=True)
    with open(cache_path, 'wb') as f:
        f.write(json.dumps(meta).encode() + b'\n')
        f.write(struct.pack('I', binary_format[0]))
        f.write(buffer.tobytes())


def create_shader_program(vertex_filepath: str, fragment_filepath: str, geometry_filepath=None) -> int:
    # Generate cache key with file signatures
    files = [vertex_filepath, fragment_filepath]
    if geometry_filepath:
        files.append(geometry_filepath)

    # Get file signatures
    signatures = [get_file_signature(f) for f in files]

    # Generate content-based key
    hasher = hashlib.sha256()
    hasher.update(json.dumps(signatures, sort_keys=True).encode())
    content_key = hasher.hexdigest()

    # Get driver-specific key component
    vendor = gl.glGetString(gl.GL_VENDOR).decode()
    renderer = gl.glGetString(gl.GL_RENDERER).decode()
    driver_key = hashlib.sha256(f"{vendor}{renderer}".encode()).hexdigest()[:16]

    # Final cache key
    cache_key = f"{content_key}_{driver_key}"
    cache_path = Path(binary_cache_dir) / f"{cache_key}.bin"

    # Try loading from cache
    if cache_path.exists():
        try:
            with open(cache_path, 'rb') as f:
                meta = json.loads(f.readline().decode())
                current_sigs = [get_file_signature(f['path']) for f in meta['files']]

                if meta['files'] == current_sigs:
                    # Load binary only if metadata matches
                    program = load_cached_program(cache_path)
                    if program:
                        print(f"Loaded cached shader: {cache_key}")
                        return program
        except Exception as e:
            print(f"Cache load failed: {str(e)}")

    # Compile and cache new program
    program = compile_shaders(vertex_filepath, fragment_filepath, geometry_filepath)

    # Save to cache with metadata
    try:
        save_program_cache(program, cache_path, signatures)
    except Exception as e:
        print(f"Cache save failed: {str(e)}")

    return program
