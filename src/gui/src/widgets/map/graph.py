from ..opengl.shader import ShaderRenderer
from ..enums import OpenGLContextName
from networkx import DiGraph


class InstanceData:
    def __init__(self):
        self.positions = []
        self.ids = []


class GraphEditor:
    def __init__(self):
        self.shader_renderer = ShaderRenderer(OpenGLContextName.GRAPH)
        self.G = DiGraph()
        self.instance_data = InstanceData()

    def draw(self, proj_mat, view_mat):
        if self.G is None:
            return
        if len(self.instance_data.positions) == 0:
            self.update_instance_data()

    def update_instance_data(self):
        for node_id, data in self.G.nodes(data=True):
            # GraphML attributes are strings by default, so cast to float
            x = float(data.get('x', 0))
            y = float(data.get('y', 0))
            print(f"Node {node_id}: ({x:.3f}, {y:.3f})")
