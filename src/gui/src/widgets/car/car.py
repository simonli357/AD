from PyQt5 import QtWidgets
from OpenGL import GL as gl
from .hud import HudRenderer
from .stats import HidableOverlay
from ..enums import MapData, NamedColor, OpenGLContextName, RoadObjectsColor
from ..opengl.shader import ShaderRenderer
from ..opengl.instance.gt import GTRenderer
from ..opengl.instance.model import ModelInstanceRenderer
from ..opengl.instance.path import PathRenderer

import numpy as np
import glm


class CarWidget(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.stop_drawing = False
        self.main_window = self.parent()

        self.yaw = 0
        self.x_pos = 11.78
        self.y_pos = MapData.REAL_WORLD_HEIGHT.value - 2.06
        self.z_pos = 0
        self.speed = 0
        self.steer = 0

        self.cam_dist = 30.0
        self.cam_height = self.cam_dist
        self.forward_offset = 5.0
        self.fov = 45.0

        self.visited = set()

        self.road_objects_ids = {
            'Oneway': set(),
            'Stopsign': set(),
            'Highway Entrance': set(),
            'Highway Exit': set(),
            'Roundabout': set(),
            'Parking': set(),
            'Crosswalk': set(),
            'No Entry': set(),
            'Priority': set(),
            'Light': set(),
            'Green Light': set(),
            'Yellow Light': set(),
            'Red Light': set(),
            'Pedestrian': set(),
        }

        self.setup_ui()

    def setup_ui(self):
        self.run_statistics = HidableOverlay(self)

    def set_steer(self, steer: float):
        self.steer = steer
        self.run_statistics.set_car_rotation(self.yaw, steer)

    def update_sw_load(self, load_msg):
        self.hud_renderer.cores_usage = load_msg.cores_usage
        self.hud_renderer.temperature = load_msg.temperature
        self.hud_renderer.ram_usage = load_msg.ram_usage
        self.hud_renderer.heap_usage = load_msg.heap_usage
        self.hud_renderer.stack_usage = load_msg.stack_usage

    def set_car_data(self, yaw: float, speed: float, x: float, y: float, z: float) -> None:
        if self.main_window.cam_buttons_widget.started:
            dx = x - self.x_pos
            dy = y - self.y_pos
            displacement = np.sqrt(dx**2 + dy**2)
            self.run_statistics.dist_traveled += displacement
            self.run_statistics.set_distance_traveled()
            self.run_statistics.update_visited_destinations(x, y)
        self.yaw = yaw
        self.speed = speed * 100
        self.x_pos = x
        self.y_pos = y
        self.z_pos = z
        self.run_statistics.set_car_pose(x, y, z)

    def initializeGL(self):
        gl.glClearColor(0.0, 0.0, 0.0, 1.0)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDepthFunc(gl.GL_LEQUAL)
        gl.glDisable(gl.GL_BLEND)
        gl.glDisable(gl.GL_LINE_SMOOTH)    # Avoid anti-aliasing overhead
        gl.glDisable(gl.GL_POLYGON_SMOOTH)
        gl.glDisable(gl.GL_MULTISAMPLE)    # Disable MSAA if not used
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_FILL)  # Fastest mode
        gl.glShadeModel(gl.GL_FLAT)        # Faster than GL_SMOOTH if applicable

        self.hud_proj_mat = glm.ortho(0.0, self.width(), self.height(), 0.0, -1.0, 1.0)

        self.SCREEN_W = 640
        self.SCREEN_H = 413
        self.CAR_SCALE = (0.86, 0.75, 0.8)

        aspect = self.SCREEN_W / self.SCREEN_H
        self.proj_mat = glm.perspective(
            self.fov,
            aspect,
            0.1,
            1000.0
        )

        self.shader_renderer = ShaderRenderer(ctx_name=OpenGLContextName.CAR)
        self.hud_renderer = HudRenderer(self)

        self.destinations_renderer = ModelInstanceRenderer(self.shader_renderer.destination_model)
        self.visited_destinations_renderer = GTRenderer(self.shader_renderer.green_destination_model, 'Destination')
        self.update_destinations()

        self.renderers = {
            'Oneway': GTRenderer(self.shader_renderer.oneway_sign_model, 'Oneway'),
            'Stopsign': GTRenderer(self.shader_renderer.stop_sign_model, 'Stopsign'),
            'Highway Entrance': GTRenderer(self.shader_renderer.highway_entrance_sign_model, 'Highway Entrance'),
            'Highway Exit': GTRenderer(self.shader_renderer.highway_exit_sign_model, 'Highway Exit'),
            'Roundabout': GTRenderer(self.shader_renderer.roundabout_sign_model, 'Roundabout'),
            'Parking': GTRenderer(self.shader_renderer.parking_sign_model, 'Parking'),
            'Crosswalk': GTRenderer(self.shader_renderer.crosswalk_sign_model, 'Crosswalk'),
            'No Entry': GTRenderer(self.shader_renderer.noentry_sign_model, 'No Entry'),
            'Priority': GTRenderer(self.shader_renderer.prio_sign_model, 'Priority'),
            'Light': GTRenderer(self.shader_renderer.traffic_light_model, 'Light'),
            'Green Light': GTRenderer(self.shader_renderer.green_light_model, 'Green Light'),
            'Yellow Light': GTRenderer(self.shader_renderer.yellow_light_model, 'Yellow Light'),
            'Red Light': GTRenderer(self.shader_renderer.red_light_model, 'Red Light'),
            'Pedestrian': GTRenderer(self.shader_renderer.pedestrian_model, 'Pedestrian'),
        }

        self._cam_quat = glm.angleAxis(glm.radians(self.yaw), glm.vec3(0, 0, 1))
        self.MIN_SLERP = 0.0
        self.MAX_SLERP = 0.75
        self.MAX_ANGLE = np.radians(20.5)
        self.DEAD_ZONE = np.radians(2.0)

    def paintGL(self):
        if self.stop_drawing:
            return

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        x, y = self.get_gl_coords(self.x_pos, self.y_pos)

        raw_quat = glm.angleAxis(glm.radians(self.yaw), glm.vec3(0, 0, 1))
        if glm.dot(self._cam_quat, raw_quat) < 0.0:
            raw_quat = -raw_quat

        q_delta = raw_quat * glm.conjugate(self._cam_quat)
        w_clamped = np.clip(q_delta.w, -1.0, 1.0)
        angle_diff = 2.0 * np.arccos(w_clamped)

        if angle_diff < self.DEAD_ZONE:
            blend = 0.0
        else:
            blend = np.interp(
                angle_diff,
                [0.0, self.MAX_ANGLE],
                [self.MIN_SLERP, self.MAX_SLERP],
                right=self.MAX_SLERP
            )

        self._cam_quat = glm.normalize(glm.slerp(self._cam_quat, raw_quat, blend))

        forward_vec = self._cam_quat * glm.vec3(1, 0, 0)

        cam_pos = glm.vec3(x, y, self.cam_height) - forward_vec * self.cam_dist
        glm_forward = glm.vec3(forward_vec.x, forward_vec.y, 0.0)
        target_pos = glm.vec3(x, y, 0.0) + glm_forward * self.forward_offset
        self.view_mat = glm.lookAt(cam_pos, target_pos, glm.vec3(0, 0, 1))

        self.shader_renderer.draw_texture(
            mat=self.shader_renderer.bfmc_track_model,
            x=0.0,
            y=0.0,
            z=0,
            scale=(self.SCREEN_W, self.SCREEN_H),
            view_matrix=self.view_mat,
            proj_matrix=self.proj_mat
        )

        self.shader_renderer.draw_car(
            x=x,
            y=y,
            yaw=np.radians(self.yaw),
            scale=self.CAR_SCALE,
            color=NamedColor.WHITE,
            view_matrix=self.view_mat,
            proj_matrix=self.proj_mat
        )

        self.destinations_renderer.draw(self.proj_mat, self.view_mat)
        self.visited_destinations_renderer.draw(self.proj_mat, self.view_mat)
        self.draw_path_nodes(self.main_window.map_widget.waypoints)
        self.draw_detected_objects(self.main_window.map_widget.detected_data, self.main_window.map_widget.road_msg_dict, self.main_window.map_widget.object_dict)

        # HUD
        self.hud_renderer.draw_hud(self.hud_proj_mat, self.width(), self.height())

        self.update()

    def update_visited_destination(self, x_visited, y_visited):
        x, y = self.get_gl_coords(x_visited, y_visited)
        self.visited.add((x, y))
        self.visited_destinations_renderer.add_or_update_instance((x, y), x, y, 0, (5.0, 5.0, 5.0), z=-0.5)
        self.visited_destinations_renderer.set_ids(self.visited)

    def clear_road_objects(self):
        for id_set in self.road_objects_ids.values():
            id_set.clear()

    def is_near(self, x1: float, y1: float, x2: float, y2: float, rad1: float, rad2: float):
        return (x2 - x1)**2 + (y2 - y1)**2 <= (rad1 + rad2)**2

    def draw_detected_objects(self, detected_data, road_msg_dict, object_dict):
        if detected_data is None or len(detected_data) == 0:
            return

        self.clear_road_objects()

        for i in range(1, len(detected_data)):
            obj_type = detected_data[i, road_msg_dict['type']]
            x_real = detected_data[i, road_msg_dict['x']]
            y_real = detected_data[i, road_msg_dict['y']]
            orientation = detected_data[i, road_msg_dict['orientation']]
            id = detected_data[i, road_msg_dict['id']]
            confidence = detected_data[i, road_msg_dict['confidence']]
            obj_name = object_dict[obj_type].upper()
            label = "".join(obj_name.split())

            # Convert map coordinates to pixel coordinates
            x, y = self.get_gl_coords(x_real, MapData.REAL_WORLD_HEIGHT.value - y_real)
            # orientation = 2 * np.pi - orientation

            if object_dict[obj_type] == 'Car':
                self.shader_renderer.draw_car(
                    x=x,
                    y=y,
                    yaw=orientation,
                    scale=self.CAR_SCALE,
                    color=NamedColor.RED,
                    view_matrix=self.view_mat,
                    proj_matrix=self.proj_mat
                )
            else:
                self.renderers[object_dict[obj_type]].add_or_update_instance(id, x, y, orientation, (32.0, 32.0, 32.0), extra_rot=True)
                self.road_objects_ids[object_dict[obj_type]].add(id)

            if self.is_near(self.x_pos, self.y_pos, x_real, MapData.REAL_WORLD_HEIGHT.value - y_real, 3.0, 0.2):
                if label == "CAR":
                    speed = detected_data[i, road_msg_dict['speed']]
                    self.shader_renderer.text_renderer.render_text3D(f"{obj_name}: {confidence:.2f}", x, y, 9.5, self.SCREEN_W, self.SCREEN_H, RoadObjectsColor[label].value, self.proj_mat, self.view_mat)
                    self.shader_renderer.tiny_text_renderer.render_text3D(f"{speed * 100:.2f} cm/s", x, y, 7, self.SCREEN_W, self.SCREEN_H, (0.0, 0.7, 0.0), self.proj_mat, self.view_mat)
                elif label == "LIGHT" or label == "GREENLIGHT" or label == "REDLIGHT" or label == "YELLOWLIGHT":
                    self.shader_renderer.text_renderer.render_text3D(f"{obj_name}: {confidence:.2f}", x, y, 10, self.SCREEN_W, self.SCREEN_H, RoadObjectsColor[label].value, self.proj_mat, self.view_mat)
                else:
                    self.shader_renderer.text_renderer.render_text3D(f"{obj_name}: {confidence:.2f}", x, y, 9, self.SCREEN_W, self.SCREEN_H, RoadObjectsColor[label].value, self.proj_mat, self.view_mat)

        self.draw_road_objects()

    def draw_road_objects(self):
        for renderer in self.renderers.values():
            renderer.set_ids(self.road_objects_ids[renderer.obj_type])
            renderer.draw(self.proj_mat, self.view_mat)

    def draw_path_nodes(self, waypoints):
        if waypoints is None or len(waypoints) < 2:
            return

        positions = []
        rotations = []

        x1, y1 = self.get_gl_coords(waypoints[0], MapData.REAL_WORLD_HEIGHT.value - waypoints[1])
        angle = 0
        for i in range(0, len(waypoints) - 1, 4):
            if i + 3 > len(waypoints):
                positions.append((x1, y1, 0.1))
                rotations.append((angle, 0, 0, 1))
            else:
                x2, y2 = self.get_gl_coords(waypoints[i + 2], MapData.REAL_WORLD_HEIGHT.value - waypoints[i + 3])
                dx = x2 - x1
                dy = y2 - y1
                angle = np.arctan2(dy, dx + (1e-5)) - np.pi / 2
                positions.append((x1, y1, 0.1))
                rotations.append((angle, 0, 0, 1))
                x1, y1 = x2, y2

        if hasattr(self, 'path_node_renderer'):
            self.path_node_renderer.transform_all(positions=positions, rotations=rotations)
        else:
            self.path_node_renderer = PathRenderer(positions=positions, rotations=rotations)
        self.path_node_renderer.render(self.proj_mat, self.view_mat)

    def get_gl_coords(self, real_x, real_y):
        # Convert real-world to OpenGL world coordinates
        world_x = real_x / MapData.REAL_WORLD_WIDTH.value * self.SCREEN_W
        world_y = (MapData.REAL_WORLD_HEIGHT.value - real_y) / MapData.REAL_WORLD_HEIGHT.value * self.SCREEN_H
        return world_x, world_y

    def update_destinations(self):
        self.destinations_renderer.clear_instances()
        for _, row in self.main_window.destinations.iterrows():
            real_x = row['X']
            real_y = MapData.REAL_WORLD_HEIGHT.value - row['Y']
            x, y = self.get_gl_coords(real_x, real_y)
            self.destinations_renderer.add_instance(
                x=x,
                y=y,
                orientation=0.0,
                scale=(4.0, 4.0, 4.0)
            )
        self.destinations_renderer.upload_instances()

    def cleanup_gl_resources(self):
        self.stop_drawing = True
        try:
            gl.glFlush()
        except Exception:
            pass

    def __del__(self):
        self.cleanup_gl_resources()

    def deleteLater(self):
        self.cleanup_gl_resources()
        super().deleteLater()

    ###############
    # Events
    ###############

    def resizeGL(self, w, h):
        gl.glViewport(0, 0, w, h)
        self.hud_proj_mat = glm.ortho(0.0, w, h, 0.0, -1.0, 1.0)
        self.update_destinations()
