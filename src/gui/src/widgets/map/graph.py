from ..opengl.shader import ShaderRenderer
from ..opengl.renderer import InstanceRenderer
from ..opengl.utils import create_shader_program, shader_path
from ..enums import OpenGLContextName, NamedColor

import numpy as np
import networkx as nx


class InstanceData:
    def __init__(self):
        self.ids = []
        self.positions = []


class Shapes:
    def __init__(self):
        self.node_shader = create_shader_program(shader_path('node', 'node.vert'), shader_path('node', 'node.frag'))
        self.node_default_scale = 8.0
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
    def __init__(self, map_widget):
        self.map_widget = map_widget
        self.shapes = Shapes()
        self.shader_renderer = ShaderRenderer(OpenGLContextName.GRAPH)
        self.instance_data = InstanceData()
        self.node_instance_renderer = None
        self.prev_hovered = None
        self.G = map_widget.main_window.database.graph_queries.fetch_graph()

    def draw(self, proj_mat, view_mat):
        if self.G is None:
            return

        mouse_pos = self.map_widget.current_mouse_pos
        mouse_x, mouse_y = self.map_widget.get_gl_coords(*self.map_widget.get_real_world_coords(mouse_pos.x(), mouse_pos.y()))

        if len(self.instance_data.positions) == 0:
            self.update_instance_data()
            self.node_instance_renderer = InstanceRenderer(self.shapes.node_base_vertices, self.instance_data.positions, NamedColor.INDIGO.value, scales=self.shapes.node_default_scale, shader_program=self.shapes.node_shader)
        if self.node_instance_renderer is not None:
            self.node_instance_renderer.render(proj_mat, view_mat)

        self.draw_node_border(mouse_x, mouse_y)

    def draw_node_border(self, mouse_x, mouse_y):
        if self.node_instance_renderer is None:
            return

        nearest = None
        m_radius = 0.1
        n_radius = 4.0

        for i, pos in enumerate(self.instance_data.positions):
            if self.is_near(mouse_x, mouse_y, pos[0], pos[1], m_radius, n_radius):
                nearest = i
                break

        if nearest != self.prev_hovered:
            # restore previous
            if self.prev_hovered is not None:
                self.node_instance_renderer.scale_instance(self.prev_hovered, self.shapes.node_default_scale)
            # hide new
            if nearest is not None:
                self.node_instance_renderer.scale_instance(nearest, 0.0)
                # Draw selected node
            self.prev_hovered = nearest

    def update_instance_data(self):
        for node_id, data in self.G.nodes(data=True):
            int_id = int(node_id.lstrip('n'))
            x_real = float(data.get('x', 0))
            y_real = float(data.get('y', 0))
            x, y = self.map_widget.get_gl_coords(x_real, y_real)
            z = 0.0
            self.instance_data.ids.append(int_id)
            self.instance_data.positions.append((x, y, z))

    def is_near(self, x1: float, y1: float, x2: float, y2: float, rad1: float, rad2: float):
        return (x2 - x1)**2 + (y2 - y1)**2 <= (rad1 + rad2)**2

    def export(self, path):
        nx.write_graphml(self.G, path)

    ##############
    # Events
    ##############

    # Return false if we want parent behavior
    def mousePressEvent(self, event) -> bool:
        return False

    # Return false if we want parent behavior
    def mouseMoveEvent(self, event) -> bool:
        return False

    def mouseReleaseEventNonDrag(self, event):
        pass

    def mouseReleaseEventDragging(self, event):
        pass
