from ..opengl.shader import ShaderRenderer
from ..opengl.renderer import InstanceRenderer
from ..opengl.utils import create_shader_program, shader_path
from ..enums import OpenGLContextName
from ..opengl.instance.arrows import ArrowInstanceRenderer
from ..forms.node_form import NodeFormWidget
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QCursor
from PyQt5.QtWidgets import QMenu

import numpy as np
import xml.etree.ElementTree as ET


class InstanceData:
    def __init__(self):
        self.idx = []
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

        self.edge_mode = 0
        self.edge_candidates = []
        self.prev_edge_node_hovered = None

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

    def draw(self, proj_mat, view_mat, ortho_proj_mat):
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

        if self.edge_mode == 1:
            self.shader_renderer.text_renderer.render_text("ADD EDGE MODE", mouse_pos.x(), mouse_pos.y() - 50, 1.0, (0, 1, 0), ortho_proj_mat)
            self.handle_edge_mode(mouse_x, mouse_y)
            self.add_edge()
        elif self.edge_mode == 2:
            self.shader_renderer.text_renderer.render_text("REMOVE EDGE MODE", mouse_pos.x(), mouse_pos.y() - 50, 1.0, (1, 0, 0), ortho_proj_mat)
            self.handle_edge_mode(mouse_x, mouse_y)
            self.remove_edge()
        else:
            self.highlight_selected_instance(mouse_x, mouse_y)

    def add_edge(self):
        if len(self.edge_candidates) == 1:
            self.node_instance_renderer.scale_instance(self.edge_candidates[0], self.shapes.node_default_scale * 1.5)
        elif len(self.edge_candidates) == 2:
            u_idx, v_idx = self.edge_candidates
            self.node_instance_renderer.scale_instance(u_idx, self.shapes.node_default_scale)
            u_id = self.instance_data.idx[u_idx]
            v_id = self.instance_data.idx[v_idx]

            if (u_id, v_id) in self.instance_data.edge_pairs:
                self.edge_mode = 0
                self.edge_candidates.clear()
                return

            x0, y0, _ = self.instance_data.positions[u_idx]
            x1, y1, _ = self.instance_data.positions[v_idx]

            self.instance_data.edge_pairs.append((u_id, v_id))
            self.instance_data.starts.append((x0, y0))
            self.instance_data.ends.append((x1, y1))

            self.arrow_instance_renderer.reset(
                self.instance_data.starts,
                self.instance_data.ends,
                self.instance_data.edge_pairs
            )

            self.edge_mode = False
            self.edge_candidates.clear()
            self.fix_edges()

    def remove_edge(self):
        if len(self.edge_candidates) == 1:
            self.node_instance_renderer.scale_instance(self.edge_candidates[0], self.shapes.node_default_scale * 1.5)
        elif len(self.edge_candidates) == 2:
            u_idx, v_idx = self.edge_candidates
            self.node_instance_renderer.scale_instance(u_idx, self.shapes.node_default_scale)
            u_id = self.instance_data.idx[u_idx]
            v_id = self.instance_data.idx[v_idx]
            pair = (u_id, v_id)

            if pair not in self.instance_data.edge_pairs:
                self.edge_mode = 0
                self.edge_candidates.clear()
                return

            ep_index = self.instance_data.edge_pairs.index(pair)
            self.instance_data.edge_pairs.pop(ep_index)
            self.instance_data.starts.pop(ep_index)
            self.instance_data.ends.pop(ep_index)

            self.arrow_instance_renderer.reset(
                self.instance_data.starts,
                self.instance_data.ends,
                self.instance_data.edge_pairs
            )

            self.edge_mode = False
            self.edge_candidates.clear()
            self.fix_edges()

    def handle_edge_mode(self, mouse_x, mouse_y):
        if self.node_instance_renderer is None:
            return

        nearest = None
        m_radius = 0.1
        n_radius = 4.0

        for i, pos in enumerate(self.instance_data.positions):
            if self.is_near(mouse_x, mouse_y, pos[0], pos[1], m_radius, n_radius):
                nearest = i
                break

        if nearest != self.prev_edge_node_hovered:
            # restore previous
            if self.prev_edge_node_hovered is not None:
                self.node_instance_renderer.scale_instance(self.prev_edge_node_hovered, self.shapes.node_default_scale)
            # grow selected
            if nearest is not None:
                self.node_instance_renderer.scale_instance(nearest, self.shapes.node_default_scale * 1.5)
            self.prev_edge_node_hovered = nearest

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
            real_id = int(data.get('id', 0))
            x_real = float(data.get('x', 0))
            y_real = float(data.get('y', 0))
            attr = int(data.get('attr', 0))
            x, y = self.map_widget.get_gl_coords(x_real, y_real)
            z = 0.0
            self.instance_data.keys.append(f"n{real_id}")
            self.instance_data.ids.append(real_id)
            self.instance_data.idx.append(int_id)
            self.instance_data.positions.append((x, y, z))
            self.instance_data.real_positions.append((x_real, y_real))
            self.instance_data.colors.append(self.ATTRIBUTES_COLORS[attr])
            self.instance_data.attributes.append(attr)

        id2pos = {
            nid: pos
            for nid, pos in zip(self.instance_data.idx, self.instance_data.positions)
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
        # 1) Register GraphML namespaces
        ET.register_namespace('', "http://graphml.graphdrawing.org/xmlns")
        ET.register_namespace('xsi', "http://www.w3.org/2001/XMLSchema-instance")

        schema_loc = (
            "http://graphml.graphdrawing.org/xmlns "
            "http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd"
        )
        # 2) Root <graphml>
        root = ET.Element(
            'graphml',
            {
                'xmlns': "http://graphml.graphdrawing.org/xmlns",
                'xmlns:xsi': "http://www.w3.org/2001/XMLSchema-instance",
                'xsi:schemaLocation': schema_loc
            }
        )

        # 3) Emit your <key> declarations in the original order
        keys = [
            ('d0', 'node', 'x', 'double'),
            ('d1', 'node', 'y', 'double'),
            ('d2', 'node', 'new_attribute', 'long'),
            ('d3', 'edge', 'dotted', 'boolean'),
        ]
        for kid, f, name, typ in keys:
            ET.SubElement(
                root, 'key',
                {'id': kid, 'for': f, 'attr.name': name, 'attr.type': typ}
            )

        # 4) The <graph> container
        graph = ET.SubElement(root, 'graph', edgedefault='directed')
        for key, (x_real, y_real), attr in zip(
                self.instance_data.ids,
                self.instance_data.real_positions,
                self.instance_data.attributes):
            node = ET.SubElement(graph, 'node', id=str(key))
            ET.SubElement(node, 'data', key='d0').text = f"{x_real}"
            ET.SubElement(node, 'data', key='d1').text = f"{y_real}"
            ET.SubElement(node, 'data', key='d2').text = f"{attr}"

        # 6) Emit edges in the exact order of instance_data.edge_pairs:
        #    use id_to_key to map your integer ids → the original key strings
        id_to_key = {i: k for i, k in zip(self.instance_data.idx, self.instance_data.keys)}
        for u_id, v_id in self.instance_data.edge_pairs:
            u_key = id_to_key[u_id]
            v_key = id_to_key[v_id]
            edge = ET.SubElement(graph, 'edge', source=u_key, target=v_key)
            ET.SubElement(edge, 'data', key='d3').text = 'True'

        # 7) Pretty-indent (Python 3.9+ or fallback)
        try:
            ET.indent(root, space="  ")
        except AttributeError:
            def _indent(e, level=0):
                i = "\n" + level * "  "
                if len(e):
                    if not e.text or not e.text.strip():
                        e.text = i + "  "
                    for c in e:
                        _indent(c, level + 1)
                    if not e.tail or not e.tail.strip():
                        e.tail = i
                else:
                    if level and (not e.tail or not e.tail.strip()):
                        e.tail = i
            _indent(root)

        # 8) Write to disk
        tree = ET.ElementTree(root)
        tree.write(path, encoding='utf-8', xml_declaration=True)

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
        node_id = self.instance_data.idx[index]
        self.arrow_instance_renderer.update_for_node(node_id, (x, y))

    def fix_edges(self):
        for index in range(len(self.instance_data.idx)):
            node_id = self.instance_data.idx[index]
            x, y, _ = self.instance_data.positions[index]
            self.arrow_instance_renderer.update_for_node(node_id, (x, y))

    def add_node(self, prev_idx=None, real_x=None, real_y=None):
        # compute new node coords
        epsilon = 0.1
        if prev_idx is not None:
            x_real, y_real = self.instance_data.real_positions[prev_idx]
            new_x_real = x_real + epsilon
            new_y_real = y_real + epsilon
            new_gl_x, new_gl_y = self.map_widget.get_gl_coords(new_x_real, new_y_real)
            z = self.instance_data.positions[prev_idx][2]
        else:
            new_x_real, new_y_real = real_x, real_y
            new_gl_x, new_gl_y = self.map_widget.get_gl_coords(real_x, real_y)
            z = 0.1

        # pick new ID/key, attribute, color
        new_id = max(self.instance_data.idx) + 1
        k = max(self.instance_data.ids) + 1
        new_key = f"n{k}"
        if prev_idx is not None:
            new_attr = self.instance_data.attributes[prev_idx]
        else:
            new_attr = 0
        new_color = self.ATTRIBUTES_COLORS[new_attr]

        # append the new node to instance_data
        self.instance_data.keys.append(new_key)
        self.instance_data.ids.append(k)
        self.instance_data.idx.append(new_id)
        self.instance_data.positions.append((new_gl_x, new_gl_y, z))
        self.instance_data.real_positions.append((new_x_real, new_y_real))
        self.instance_data.attributes.append(new_attr)
        self.instance_data.colors.append(new_color)

        self.node_instance_renderer.add_instance(new_gl_x, new_gl_y, z, None, self.shapes.node_default_scale, new_color)

        return new_id, new_gl_x, new_gl_y

    ##############
    # Events
    ##############

    def mouseDoubleClickEvent(self, event):
        # only handle left-button double-click when we have a hovered node
        if event.button() == Qt.LeftButton and self.prev_hovered is not None:
            prev_idx = self.prev_hovered
            prev_id = self.instance_data.idx[prev_idx]
            # find the “next” in our edge_pairs, if any
            next_pair = next(((u, v) for u, v in self.instance_data.edge_pairs if u == prev_id), None)
            next_id = next_pair[1] if next_pair is not None else None

            new_id, new_gl_x, new_gl_y = self.add_node(prev_idx)

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
                next_idx = self.instance_data.idx.index(next_id)
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
        if event.button() == Qt.LeftButton and self.edge_mode and self.prev_edge_node_hovered is not None:
            if self.prev_edge_node_hovered not in self.edge_candidates:
                self.edge_candidates.append(self.prev_edge_node_hovered)
        elif event.button() == Qt.LeftButton and self.prev_hovered is not None:
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
        elif event.button() == Qt.RightButton and self.prev_hovered is None:
            self.edge_mode = 0
            menu = QMenu(self.map_widget)
            action_0 = menu.addAction("Add Node")
            action_1 = menu.addAction("Add Edge")
            action_2 = menu.addAction("Remove Edge")
            menu.setStyleSheet("""
                QMenu {
                    color: white;
                    font-size: 16px;
                    border: none;
                    background-color: transparent;
                }
                QMenu::item {
                    background-color: rgba(40, 40, 40, 0.5);
                    margin: 1px;
                    padding-left: 30px;
                    padding-right: 30px;
                    padding-top: 4px;
                    padding-bottom: 4px;
                    border-radius: 8px;
                }
                QMenu::item:selected {
                    background-color: purple;
                }
            """)
            chosen = menu.exec(QCursor.pos())
            if chosen == action_0:
                xw, yw = self.map_widget.get_real_world_coords(event.x(), event.y())
                self.add_node(real_x=xw, real_y=yw)
            elif chosen == action_1:
                self.edge_mode = 1
            elif chosen == action_2:
                self.edge_mode = 2
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
        node_id = self.instance_data.idx[node_index]

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
                pi = self.instance_data.idx.index(pu)
                ni = self.instance_data.idx.index(nv)
                x0, y0 = self.instance_data.positions[pi][:2]
                x1, y1 = self.instance_data.positions[ni][:2]
                self.instance_data.starts.append((x0, y0))
                self.instance_data.ends.append((x1, y1))

        for lst in (
            self.instance_data.ids,
            self.instance_data.idx,
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
