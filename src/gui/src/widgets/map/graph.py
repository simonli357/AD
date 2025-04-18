from ..opengl.shader import ShaderRenderer
from ..opengl.renderer import InstanceRenderer
from ..enums import OpenGLContextName
from networkx import DiGraph

import numpy as np


class InstanceData:
    def __init__(self):
        self.ids = []
        self.positions = []


class Shapes:
    def __init__(self):
        self.node_base_vertices = np.array([
            # Positions (3D for proper matrix transformations)
            [0.0, 0.5, 0.1],  # Top
            [0.5, 0.0, 0.1],  # Right
            [-0.5, 0.0, 0.1],  # Left
            [-0.5, 0.0, 0.1],  # Left
            [0.5, 0.0, 0.1],  # Right
            [0.0, -0.5, 0.1],  # Bottom
        ], dtype=np.float32).flatten()


class GraphEditor:
    def __init__(self):
        self.shader_renderer = ShaderRenderer(OpenGLContextName.GRAPH)
        self.instance_data = InstanceData()
        self.node_instance_renderer = None
        self.G = DiGraph()

    def draw(self, proj_mat, view_mat):
        if self.G is None:
            return
        if len(self.instance_data.positions) == 0:
            self.update_instance_data()
            self.node_instance_renderer = InstanceRenderer(self.node_base_vertices, self.instance_data.positions)
        if self.node_instance_renderer is not None:
            self.node_instance_renderer.render(proj_mat, view_mat)

    def update_instance_data(self):
        for node_id, data in self.G.nodes(data=True):
            int_id = int(node_id.lstrip('n'))
            x = float(data.get('x', 0))
            y = float(data.get('y', 0))
            z = 0.0
            self.instance_data.ids.append(int_id)
            self.instance_data.positions.append((x, y, z))
