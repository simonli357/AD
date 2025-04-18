from ..opengl.shader import ShaderRenderer
from ..enums import OpenGLContextName


class GraphEditor:
    def __init__(self):
        self.shader_renderer = ShaderRenderer(OpenGLContextName.GRAPH)

    def draw(self, proj_mat, view_mat):
        pass
