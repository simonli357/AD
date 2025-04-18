import OpenGL.GL as gl
from .shader import create_shader_program, shader_path
from ..enums import MapData

import numpy as np
import glm


class InstanceRenderer:
    def __init__(self, positions, base_vertices, shader_program, instance_scale, instance_rotation):
        self.positions = positions
        self.base_vertices = base_vertices
        self.shader_program = shader_program
