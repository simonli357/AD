from ..opengl.shader import ShaderRenderer
from ..enums import OpenGLContextName
from networkx import Graph


class GraphEditor:
    def __init__(self):
        self.shader_renderer = ShaderRenderer(OpenGLContextName.GRAPH)
        self.G = Graph()

    def draw(self, proj_mat, view_mat):
        if self.G is None:
            return
        print(self.G.number_of_nodes)
