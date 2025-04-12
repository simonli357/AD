from PyQt5 import QtWidgets, QtCore
from OpenGL import GL as gl
from ..opengl.shader import ShaderRenderer

import glm


class CameraOverlay(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None, cam_widget=None):
        super().__init__(parent)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground, True)
        self.setAttribute(QtCore.Qt.WA_AlwaysStackOnTop, True)
        self.cam_widget = cam_widget

        # Lane
        self.center = None
        self.crosswalk = False
        self.stopline = False
        self.stopline_dist = None

        self.class_names = ["oneway", "highwayentrance", "stopsign", "roundabout", "park", "crosswalk", "noentry", "highwayexit", "priority", "lights", "block", "pedestrian", "car", "green light", "yellow light", "red light"]
        self.confidence_thresholds = [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.65, 0.65, 0.65, 0.65, 0.7, 0.75, 0.65, 0.65, 0.65]

        self.COLOR_LIST = [
            (1, 1, 1), (0.098, 0.325, 0.850), (0.125, 0.694, 0.929), (0.556, 0.184, 0.494), (0.188, 0.674, 0.466),
            (0.933, 0.745, 0.301), (0.184, 0.078, 0.635), (0.300, 0.300, 0.300), (0.600, 0.600, 0.600), (0.000, 0.000, 1.000),
            (0.000, 0.500, 1.000), (0.000, 0.749, 0.749), (0.000, 1.000, 0.000)
        ]

    def update_overlay(self):
        self.update()

    def initializeGL(self):
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glClearColor(0, 0, 0, 0)

        self.shader_renderer = ShaderRenderer()
        self.box_renderer = self.shader_renderer.detection_box_model
        self.lane_renderer = self.shader_renderer.lane_model
        self.proj_mat = glm.ortho(0.0, self.width(), self.height(), 0.0, -1.0, 1.0)
        self.view_mat = glm.vec4(1.0)

    def paintGL(self):
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        if not self.cam_widget.has_frame:
            self.shader_renderer.text_renderer.render_text("NO VIDEO", 320, 240, 1.0, (0.0, 1.0, 0.0), self.proj_mat)
        self.draw_detection_boxes()
        self.draw_lane_indicator()

    def draw_detection_boxes(self):
        for i in range(self.cam_widget.numObj):
            try:
                id = int(self.cam_widget.detected_objects[7 * i + 6])
            except Exception as e:
                print("Error in sign detection")
                print(e)
                return
            if self.cam_widget.detected_objects[7 * i + 5] < self.confidence_thresholds[id]:
                continue

            color_index = id % len(self.COLOR_LIST)
            color = self.COLOR_LIST[color_index]

            confidence = self.cam_widget.detected_objects[7 * i + 5] * 100
            distance = self.cam_widget.detected_objects[7 * i + 4]
            text = f"{self.class_names[id]} {confidence:.1f}% {distance:.2f} m"

            x1 = int(self.cam_widget.detected_objects[7 * i])
            y1 = int(self.cam_widget.detected_objects[7 * i + 1])
            x2 = int(self.cam_widget.detected_objects[7 * i + 2])
            y2 = int(self.cam_widget.detected_objects[7 * i + 3])

            self.box_renderer.draw(x1, y1, x2, y2, text.upper(), 1.0, color, self.proj_mat)

    def draw_lane_indicator(self):
        if self.center is None:
            return

        thickness = 5
        x1 = self.center * self.width() - thickness / 2.0
        x2 = self.center * self.width() + thickness / 2.0
        y1 = 480 * 0.8
        y2 = 480
        self.lane_renderer.draw(x1, y1, x2, y2, 4, (1.0, 1.0, 0.0), self.proj_mat)

        text_x = 0.02 * self.width()
        text_y = 0.04 * self.height()
        y_offset = 30

        if self.stopline_dist:
            text = f"STOPLINE DISTANCE: {self.stopline_dist:.2f}"
            text_w, text_h = self.shader_renderer.text_renderer.compute_text_size(text, 1.0)
            self.shader_renderer.text_renderer.render_text(text, text_x + text_w / 2, text_y + text_h / 2, 1.0, (0.0, 1.0, 0.0), self.proj_mat)
            text_y += y_offset

        if self.stopline:
            text = "STOPLINE DETECTED"
            text_w, text_h = self.shader_renderer.text_renderer.compute_text_size(text, 1.0)
            self.shader_renderer.text_renderer.render_text(text, text_x + text_w / 2, text_y + text_h / 2, 1.0, (0.0, 1.0, 0.0), self.proj_mat)
            text_y += y_offset

        if self.crosswalk:
            text = "CROSSWALK DETECTED"
            text_w, text_h = self.shader_renderer.text_renderer.compute_text_size(text, 1.0)
            self.shader_renderer.text_renderer.render_text(text, text_x + text_w / 2, text_y + text_h / 2, 1.0, (0.0, 1.0, 0.0), self.proj_mat)

    ################
    # Events
    ################

    def resizeGL(self, w, h):
        super().resizeGL(w, h)
        self.proj_mat = glm.ortho(0.0, w, h, 0.0, -1.0, 1.0)
