from PyQt5 import QtWidgets, QtCore
from OpenGL import GL as gl
from ..opengl.shader import ShaderRenderer

import glm
import numpy as np


class CameraOverlay(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None, cam_widget=None):
        super().__init__(parent)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground, True)
        self.setAttribute(QtCore.Qt.WA_AlwaysStackOnTop, True)
        self.cam_widget = cam_widget

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
        self.proj_mat = glm.ortho(0.0, self.width(), self.height(), 0.0, -1.0, 1.0)
        self.view_mat = glm.vec4(1.0)

    def paintGL(self):
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        if not self.cam_widget.has_frame:
            self.shader_renderer.text_renderer.render_text("NO VIDEO", 320, 240, 1.0, (0.0, 1.0, 0.0), self.proj_mat)

        self.box_renderer.draw(100, 100, 200, 200, "TEST", 1.0, (0, 1, 0), self.proj_mat)

    def draw_detection_boxes(self):
        for i in range(self.cam_widget.numObj):
            try:
                id = int(self.cam_widget.detected_objects[7 * i + 6])
            except Exception as e:
                print("Error in sign detection")
                print(e)
                return
            if self.cam_widget.detected_objects[7 * i + 5] < self.cam_widget.confidence_thresholds[id]:
                continue

            color_index = id % len(self.COLOR_LIST)
            color = tuple(int(c * 255) for c in self.COLOR_LIST[color_index])  # Scale color to [0, 255]

            confidence = self.cam_widget.detected_objects[7 * i + 5] * 100
            distance = self.cam_widget.detected_objects[7 * i + 4]
            text = f"{self.cam_widget.class_names[id]} {confidence:.1f}% {distance:.2f}m"

            x1 = int(self.cam_widget.detected_objects[7 * i])
            y1 = int(self.cam_widget.detected_objects[7 * i + 1])
            x2 = int(self.cam_widget.detected_objects[7 * i + 2])
            y2 = int(self.cam_widget.detected_objects[7 * i + 3])

    def draw_lane_indicator(self):
        if self.center is None:
            return
        # Draw the center line
        # cv2.line(image, (int(self.center), image.shape[0]), (int(self.center), int(0.8 * image.shape[0])), (0, 0, 255), 5)
        # cv2.putText(image, f"center: {self.center:.2f}",
        #             (int(image.shape[1] * 0.05), int(image.shape[0] * 0.1)),
        #             cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        # # Add text if stopline or crosswalk is detected
        # if self.stopline:
        #     cv2.putText(image, "Stopline detected!",
        #                 (int(image.shape[1] * 0.05), int(image.shape[0] * 0.3)),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        # if self.crosswalk:
        #     cv2.putText(image, "Crosswalk detected!",
        #                 (int(image.shape[1] * 0.05), int(image.shape[0] * 0.4)),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        # if self.stopline_dist:
        #     cv2.putText(image, f"Stopline distance: {self.stopline_dist:.2f}",
        #                 (int(image.shape[1] * 0.05), int(image.shape[0] * 0.2)),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

    ################
    # Events
    ################

    def resizeGL(self, w, h):
        super().resizeGL(w, h)
        self.proj_mat = glm.ortho(0.0, w, h, 0.0, -1.0, 1.0)
