from ..opengl.shader import ShaderRenderer
from ..opengl.renderer import InstanceRenderer
from ..opengl.utils import create_shader_program, shader_path
from ..enums import OpenGLContextName, NamedColor
from ..opengl.instance.arrows import ArrowInstanceRenderer
from PyQt5.QtCore import Qt

import numpy as np
import networkx as nx


class InstanceData:
    def __init__(self):
        self.ids = []
        self.keys = []
        self.positions = []
        self.starts = []
        self.ends = []
        self.edge_pairs = []


class Shapes:
    def __init__(self):
        self.node_shader = create_shader_program(shader_path('node', 'node.vert'), shader_path('node', 'node.frag'))
        self.node_default_scale = 8.0
        self.arrow_thickness = 20.0
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
        self.arrow_instance_renderer = None
        self.prev_hovered = None
        self.G = map_widget.main_window.database.graph_queries.fetch_graph()

        self._dragging = False
        self._drag_index = None

    def draw(self, proj_mat, view_mat):
        self.proj_mat = proj_mat
        self.view_mat = view_mat

        if self.G is None:
            return

        mouse_pos = self.map_widget.current_mouse_pos
        mouse_x, mouse_y = self.map_widget.get_gl_coords(*self.map_widget.get_real_world_coords(mouse_pos.x(), mouse_pos.y()))

        if len(self.instance_data.positions) == 0:
            self.update_instance_data()
            self.node_instance_renderer = InstanceRenderer(self.shapes.node_base_vertices, self.instance_data.positions, NamedColor.INDIGO.value, scales=self.shapes.node_default_scale, shader_program=self.shapes.node_shader)
            self.arrow_instance_renderer = ArrowInstanceRenderer(self.instance_data.starts, self.instance_data.ends, color=(1, 0, 0, 1), thickness=self.shapes.arrow_thickness, edge_pairs=self.instance_data.edge_pairs)
        if self.node_instance_renderer is not None:
            self.node_instance_renderer.render(proj_mat, view_mat)
        if self.arrow_instance_renderer is not None:
            self.arrow_instance_renderer.render(proj_mat, view_mat)

        self.highlight_selected_instance(mouse_x, mouse_y)

    def highlight_selected_instance(self, mouse_x, mouse_y):
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
                self.node_instance_renderer.color_instance(self.prev_hovered, NamedColor.INDIGO.value)
            # hide new
            if nearest is not None:
                self.node_instance_renderer.color_instance(nearest, NamedColor.YELLOW.value)
            self.prev_hovered = nearest

    def update_instance_data(self):
        for node_id, data in self.G.nodes(data=True):
            int_id = int(node_id.lstrip('n'))
            x_real = float(data.get('x', 0))
            y_real = float(data.get('y', 0))
            x, y = self.map_widget.get_gl_coords(x_real, y_real)
            z = 0.0
            self.instance_data.keys.append(node_id)
            self.instance_data.ids.append(int_id)
            self.instance_data.positions.append((x, y, z))

        id2pos = {
            nid: pos
            for nid, pos in zip(self.instance_data.ids, self.instance_data.positions)
        }

        for u, v in self.G.edges():
            try:
                ui = int(str(u).lstrip('n'))
                vi = int(str(v).lstrip('n'))
            except ValueError:
                continue

            start = id2pos.get(ui)
            end = id2pos.get(vi)
            if start is None or end is None:
                continue

            self.instance_data.starts.append((start[0], start[1]))
            self.instance_data.ends.append((end[0], end[1]))
            self.instance_data.edge_pairs.append((ui, vi))

    def is_near(self, x1: float, y1: float, x2: float, y2: float, rad1: float, rad2: float):
        return (x2 - x1)**2 + (y2 - y1)**2 <= (rad1 + rad2)**2

    def export(self, path):
        for u, v, data in self.G.edges(data=True):
            x1 = float(self.G.nodes[u]['x'])
            y1 = float(self.G.nodes[u]['y'])
            x2 = float(self.G.nodes[v]['x'])
            y2 = float(self.G.nodes[v]['y'])
            data['dist'] = np.hypot(x2 - x1, y2 - y1)
        nx.write_graphml(self.G, path)

    ##############
    # Events
    ##############

    # Return false if we want parent behavior
    def mousePressEvent(self, event) -> bool:
        if event.button() == Qt.LeftButton and self.prev_hovered is not None:
            self._dragging = True
            self._drag_index = self.prev_hovered
        return False

    # Return false if we want parent behavior
    def mouseMoveEvent(self, event) -> bool:
        if self._dragging:
            # map widget gives real‐world coords, then GL coords
            xw, yw = self.map_widget.get_real_world_coords(event.x(), event.y())
            x_gl, y_gl = self.map_widget.get_gl_coords(xw, yw)

            # update your stored position
            z = self.instance_data.positions[self._drag_index][2]
            self.instance_data.positions[self._drag_index] = (x_gl, y_gl, z)

            # tell the instancer to move it
            self.node_instance_renderer.translate_instance(self._drag_index, x_gl, y_gl)
            self.node_instance_renderer.color_instance(self._drag_index, NamedColor.YELLOW.value)
            node_id = self.instance_data.ids[self._drag_index]
            self.arrow_instance_renderer.update_for_node(
                node_id,
                (x_gl, y_gl)
            )
            return True
        return False

    def mouseReleaseEventNonDrag(self, event):
        if event.button() == Qt.LeftButton:
            self._dragging = False
            xw, yw = self.map_widget.get_real_world_coords(event.x(), event.y())
            if self._drag_index is None:
                return
            node_key = self.instance_data.keys[self._drag_index]
            self.G.nodes[node_key]['x'] = float(xw)
            self.G.nodes[node_key]['y'] = float(yw)
            self.node_instance_renderer.color_instance(self._drag_index, NamedColor.INDIGO.value)
            self._drag_index = None

    def mouseReleaseEventDragging(self, event):
        pass
