from ..opengl.shader import ShaderRenderer
from ..opengl.renderer import InstanceRenderer
from ..opengl.utils import create_shader_program, shader_path
from ..enums import OpenGLContextName
from ..opengl.instance.arrows import ArrowInstanceRenderer
from ..forms.node_form import NodeFormWidget
from PyQt5.QtCore import Qt

import numpy as np
import networkx as nx


class InstanceData:
    def __init__(self):
        self.ids = []
        self.keys = []
        self.positions = []
        self.real_positions = []
        self.starts = []
        self.ends = []
        self.edge_pairs = []
        self.attributes = []
        self.colors = []


class Shapes:
    def __init__(self):
        self.node_shader = create_shader_program(shader_path('node', 'node.vert'), shader_path('node', 'node.frag'))
        self.node_default_scale = 8.0
        self.arrow_thickness = 16.0
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

        self.ATTRIBUTES_COLORS = {
            0: (1.0, 1.0, 0.0, 1.0),
            1: (0.0, 1.0, 0.0, 1.0),
            2: (1.0, 0.0, 0.0, 1.0),
            3: (1.0, 0.5, 0.0, 1.0),
            4: (0.5, 0.0, 0.5, 1.0),
            5: (0.8, 0.7, 1.0, 1.0),
            6: (1.0, 1.0, 1.0, 1.0),
            7: (0.0, 1.0, 1.0, 1.0),
            8: (0.4, 0.5, 0.7, 1.0),
            9: (0.5, 0.0, 0.5, 1.0),
        }

    def draw(self, proj_mat, view_mat):
        self.proj_mat = proj_mat
        self.view_mat = view_mat

        if self.G is None:
            return

        mouse_pos = self.map_widget.current_mouse_pos
        mouse_x, mouse_y = self.map_widget.get_gl_coords(*self.map_widget.get_real_world_coords(mouse_pos.x(), mouse_pos.y()))

        if len(self.instance_data.positions) == 0:
            self.update_instance_data()
            self.node_instance_renderer = InstanceRenderer(self.shapes.node_base_vertices, self.instance_data.positions, self.instance_data.colors, scales=self.shapes.node_default_scale, shader_program=self.shapes.node_shader)
            self.arrow_instance_renderer = ArrowInstanceRenderer(self.instance_data.starts, self.instance_data.ends, color=(0.0, 0.7, 0, 1), thickness=self.shapes.arrow_thickness, edge_pairs=self.instance_data.edge_pairs)
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
                self.node_instance_renderer.scale_instance(self.prev_hovered, self.shapes.node_default_scale)
            # grow selected
            if nearest is not None:
                self.node_instance_renderer.scale_instance(nearest, self.shapes.node_default_scale * 1.5)
            self.prev_hovered = nearest

    def update_instance_data(self):
        for node_id, data in self.G.nodes(data=True):
            int_id = int(node_id.lstrip('n'))
            x_real = float(data.get('x', 0))
            y_real = float(data.get('y', 0))
            attr = int(data.get('attr', 0))
            x, y = self.map_widget.get_gl_coords(x_real, y_real)
            z = 0.0
            self.instance_data.keys.append(node_id)
            self.instance_data.ids.append(int_id)
            self.instance_data.positions.append((x, y, z))
            self.instance_data.real_positions.append((x_real, y_real))
            self.instance_data.colors.append(self.ATTRIBUTES_COLORS[attr])
            self.instance_data.attributes.append(attr)

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
        """
        Apply all in-memory instance_data changes back to self.G and write GraphML.
        """
        self.G.clear()

        id_to_key = {i: k for i, k in zip(self.instance_data.ids, self.instance_data.keys)}

        for node_id, node_key, (x_real, y_real), attr in zip(self.instance_data.ids, self.instance_data.keys, self.instance_data.real_positions, self.instance_data.attributes):
            self.G.add_node(node_key, id=int(node_id), x=float(x_real), y=float(y_real), attr=int(attr))

        for u_id, v_id in self.instance_data.edge_pairs:
            u_key = id_to_key[u_id]
            v_key = id_to_key[v_id]
            ux, uy = next(rp for i, rp in zip(self.instance_data.ids, self.instance_data.real_positions) if i == u_id)
            vx, vy = next(rp for i, rp in zip(self.instance_data.ids, self.instance_data.real_positions) if i == v_id)
            dist = float(np.hypot(vx - ux, vy - uy))
            self.G.add_edge(u_key, v_key, dist=dist)

        nx.write_graphml(self.G, path)

    def update_instance(self, index, x_real, y_real, attr):
        self.instance_data.real_positions[index] = (x_real, y_real)
        self.instance_data.attributes[index] = attr
        self.node_instance_renderer.color_instance(index, self.ATTRIBUTES_COLORS[attr])

    def update_instance_pos(self, index, x, y, x_real, y_real):
        z = self.instance_data.positions[index][2]
        self.instance_data.positions[index] = (x, y, z)
        self.instance_data.real_positions[index] = (x_real, y_real)

    def translate_node(self, index, x_real, y_real):
        x, y = self.map_widget.get_gl_coords(x_real, y_real)
        self.update_instance_pos(index, x, y, x_real, y_real)
        self.node_instance_renderer.translate_instance(index, x, y)
        node_id = self.instance_data.ids[index]
        self.arrow_instance_renderer.update_for_node(node_id, (x, y))

    def fix_edges(self):
        for index in range(len(self.instance_data.ids)):
            node_id = self.instance_data.ids[index]
            x, y, _ = self.instance_data.positions[index]
            self.arrow_instance_renderer.update_for_node(node_id, (x, y))

    ##############
    # Events
    ##############

    def mouseDoubleClickEvent(self, event):
        # only handle left-button double-click when we have a hovered node
        if event.button() != Qt.LeftButton or self.prev_hovered is None:
            return super().mouseDoubleClickEvent(event)

        prev_idx = self.prev_hovered
        prev_id = self.instance_data.ids[prev_idx]

        # find the “next” in our edge_pairs, if any
        next_pair = next(((u, v) for u, v in self.instance_data.edge_pairs if u == prev_id), None)
        next_id = next_pair[1] if next_pair is not None else None

        # compute new node coords (real + GL)
        epsilon = 0.1
        x_real, y_real = self.instance_data.real_positions[prev_idx]
        new_x_real = x_real + epsilon
        new_y_real = y_real + epsilon
        new_gl_x, new_gl_y = self.map_widget.get_gl_coords(new_x_real, new_y_real)
        z = self.instance_data.positions[prev_idx][2]

        # pick new ID/key, attribute, color
        new_id = max(self.instance_data.ids) + 1
        new_key = f"n{new_id}"
        new_attr = self.instance_data.attributes[prev_idx]
        new_color = self.ATTRIBUTES_COLORS[new_attr]

        # append the new node to instance_data
        self.instance_data.keys.append(new_key)
        self.instance_data.ids.append(new_id)
        self.instance_data.positions.append((new_gl_x, new_gl_y, z))
        self.instance_data.real_positions.append((new_x_real, new_y_real))
        self.instance_data.attributes.append(new_attr)
        self.instance_data.colors.append(new_color)

        self.node_instance_renderer.add_instance(new_gl_x, new_gl_y, z, None, self.shapes.node_default_scale, new_color)

        # splice edges in instance_data—and keep starts/ends in sync:
        if next_id is not None:
            # 1) remove the old prev→next edge and its start/end
            if (prev_id, next_id) in self.instance_data.edge_pairs:
                idx = self.instance_data.edge_pairs.index((prev_id, next_id))
                self.instance_data.edge_pairs.pop(idx)
                self.instance_data.starts.pop(idx)
                self.instance_data.ends.pop(idx)
            # 2) add prev→new
            self.instance_data.edge_pairs.append((prev_id, new_id))
            #   start at prev’s current GL pos, end at the new node
            x0, y0 = self.instance_data.positions[prev_idx][:2]
            self.instance_data.starts.append((x0, y0))
            self.instance_data.ends.append((new_gl_x, new_gl_y))
            # 3) add new→next
            next_idx = self.instance_data.ids.index(next_id)
            x1, y1 = self.instance_data.positions[next_idx][:2]
            self.instance_data.edge_pairs.append((new_id, next_id))
            self.instance_data.starts.append((new_gl_x, new_gl_y))
            self.instance_data.ends.append((x1, y1))
        else:
            # only prev→new
            self.instance_data.edge_pairs.append((prev_id, new_id))
            x0, y0 = self.instance_data.positions[prev_idx][:2]
            self.instance_data.starts.append((x0, y0))
            self.instance_data.ends.append((new_gl_x, new_gl_y))

        self.arrow_instance_renderer.reset(self.instance_data.starts, self.instance_data.ends, self.instance_data.edge_pairs)
        self.fix_edges()

    # Return false if we want parent behavior
    def mousePressEvent(self, event) -> bool:
        if event.button() == Qt.LeftButton and self.prev_hovered is not None:
            self._dragging = True
            self._drag_index = self.prev_hovered
        if event.button() == Qt.RightButton and self.prev_hovered is not None:
            NodeFormWidget(
                on_accept=self.handleNodeFormAccept,
                on_delete=self.handleNodeFormDelete,
                node_index=self.prev_hovered,
                node_id=self.instance_data.ids[self.prev_hovered],
                attr=self.instance_data.attributes[self.prev_hovered],
                node_color=self.instance_data.colors[self.prev_hovered],
                x=self.instance_data.real_positions[self.prev_hovered][0],
                y=self.instance_data.real_positions[self.prev_hovered][1]
            ).exec()
        return False

    # Return false if we want parent behavior
    def mouseMoveEvent(self, event) -> bool:
        if self._dragging:
            # map widget gives real‐world coords, then GL coords
            xw, yw = self.map_widget.get_real_world_coords(event.x(), event.y())
            self.node_instance_renderer.scale_instance(self._drag_index, self.shapes.node_default_scale * 1.5)
            self.translate_node(self._drag_index, xw, yw)
            return True
        return False

    def mouseReleaseEventNonDrag(self, event):
        if event.button() == Qt.LeftButton:
            if self._drag_index is not None:
                self.node_instance_renderer.scale_instance(self._drag_index, self.shapes.node_default_scale)
            self._dragging = False
            self._drag_index = None

    def handleNodeFormAccept(self, node_index, x, y, attr):
        self.update_instance(node_index, x, y, attr)
        self.translate_node(node_index, x, y)

    def handleNodeFormDelete(self, node_index: int):
        node_id = self.instance_data.ids[node_index]

        prev_edges = [(u, v) for u, v in self.instance_data.edge_pairs if v == node_id]
        next_edges = [(u, v) for u, v in self.instance_data.edge_pairs if u == node_id]
        prev_ids = list({u for u, _ in prev_edges})
        next_ids = list({v for _, v in next_edges})

        to_remove = [
            i for i, (u, v) in enumerate(self.instance_data.edge_pairs)
            if u == node_id or v == node_id
        ]
        for i in sorted(to_remove, reverse=True):
            self.instance_data.edge_pairs.pop(i)
            self.instance_data.starts.pop(i)
            self.instance_data.ends.pop(i)

        for pu in prev_ids:
            for nv in next_ids:
                self.instance_data.edge_pairs.append((pu, nv))
                # lookup GL positions
                pi = self.instance_data.ids.index(pu)
                ni = self.instance_data.ids.index(nv)
                x0, y0 = self.instance_data.positions[pi][:2]
                x1, y1 = self.instance_data.positions[ni][:2]
                self.instance_data.starts.append((x0, y0))
                self.instance_data.ends.append((x1, y1))

        for lst in (
            self.instance_data.ids,
            self.instance_data.keys,
            self.instance_data.positions,
            self.instance_data.real_positions,
            self.instance_data.attributes,
            self.instance_data.colors,
        ):
            lst.pop(node_index)

        self.node_instance_renderer.remove_instance(node_index)

        self.arrow_instance_renderer.reset(
            self.instance_data.starts,
            self.instance_data.ends,
            self.instance_data.edge_pairs
        )

        if self.prev_hovered == node_index:
            self.prev_hovered = None
        elif self.prev_hovered is not None and self.prev_hovered > node_index:
            self.prev_hovered -= 1

        if self._drag_index == node_index:
            self._drag_index = None
        elif self._drag_index is not None and self._drag_index > node_index:
            self._drag_index -= 1
        self.fix_edges()
