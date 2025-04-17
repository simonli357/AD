from PyQt5 import QtWidgets, QtCore
from OpenGL import GL as gl
from ..enums import OpenGLContextName
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

        # RealSense intrinsics
        self.CAMERA_PARAMS_REAL = {
            "fx": 607.40564,
            "fy": 607.05829,
            "cx": 322.97223,
            "cy": 244.39398,
        }

        # RealSense extrinsics: x, y, z, roll, pitch, yaw
        self.REALSENSE_TF_REAL = (-0.113, 0.0, 0.2573, 0.0, 0.2617, 0.0)

    def update_overlay(self):
        self.update()

    def update_camera_matrices(self, w, h):
        # 1) Extract intrinsics
        fx = self.CAMERA_PARAMS_REAL["fx"]
        fy = self.CAMERA_PARAMS_REAL["fy"]
        cx = self.CAMERA_PARAMS_REAL["cx"]
        cy = self.CAMERA_PARAMS_REAL["cy"]

        # 2) Choose near/far planes (tweak as needed)
        near = 0.1
        far = 100.0

        # 4) Build the GL projection matrix that encodes your pinhole intrinsics
        #    (column‑major, glm style: mat[col][row])
        P = glm.mat4(0.0)
        P[0][0] = 2.0 * fx / w
        P[1][1] = 2.0 * fy / h
        P[2][0] = 2.0 * cx / w - 1.0
        # note the 1 - 2*cy/h because Qt’s Y=0 is top, OpenGL’s Y=-1 is bottom
        P[2][1] = 1.0 - 2.0 * cy / h
        P[2][2] = -(far + near) / (far - near)
        P[2][3] = -1.0
        P[3][2] = -2.0 * far * near / (far - near)
        # store
        self.proj_mat = P

        # 5) Extract extrinsics
        tx, ty, tz, roll, pitch, yaw = self.REALSENSE_TF_REAL

        swap = glm.rotate(glm.mat4(1.0), -glm.pi() / 2.0, glm.vec3(1, 0, 0))
        flipZ = glm.rotate(glm.mat4(1.0), glm.pi(), glm.vec3(0, 0, 1))

        # 6) Build world‑from‑camera transform: T * R
        T = glm.translate(glm.mat4(1.0), glm.vec3(tx, ty, tz))
        R = glm.mat4(1.0)
        R = glm.rotate(R, yaw, glm.vec3(0.0, 0.0, 1.0))
        R = glm.rotate(R, pitch, glm.vec3(1.0, 0.0, 0.0))
        R = glm.rotate(R, roll, glm.vec3(0.0, 1.0, 0.0))
        self.extrinsic = T * swap * R * flipZ

        # 7) Invert to get view (world → camera)
        self.view_mat = glm.inverse(self.extrinsic)

    def initializeGL(self):
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glClearColor(0, 0, 0, 0)

        self.shader_renderer = ShaderRenderer(ctx_name=OpenGLContextName.CAM)
        self.box_renderer = self.shader_renderer.detection_box_model
        self.lane_renderer = self.shader_renderer.lane_model
        self.hud_proj_mat = glm.ortho(0.0, self.width(), self.height(), 0.0, -1.0, 1.0)
        self.hud_view_mat = glm.vec4(1.0)

        self.update_camera_matrices(self.width(), self.height())

    def paintGL(self):
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        self.draw_detection_boxes()
        self.draw_lane_indicator()
        self.shader_renderer.grid_model.draw(self.proj_mat, self.view_mat, color=(1.0, 1.0, 1.0), cell_size=0.05)
        self.shader_renderer.eye_model.draw(self.proj_mat, self.view_mat, self.extrinsic)

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

            self.box_renderer.draw(x1, y1, x2, y2, text.upper(), 1.0, color, self.hud_proj_mat)

    def draw_lane_indicator(self):
        if self.center is None:
            return

        thickness = 5
        x1 = self.center * self.width() - thickness / 2.0
        x2 = self.center * self.width() + thickness / 2.0
        y1 = self.height() * 0.8
        y2 = self.height()
        self.lane_renderer.draw(x1, y1, x2, y2, 4, (1.0, 1.0, 0.0), self.hud_proj_mat)

        text_x = 0.02 * self.width()
        text_y = 0.04 * self.height()
        y_offset = 30

        text = f"CENTER: {self.center:.2f}"
        text_w, text_h = self.shader_renderer.text_renderer.compute_text_size(text, 1.0)
        self.shader_renderer.text_renderer.render_text(text, text_x + text_w / 2, text_y + text_h / 2, 1.0, (0.0, 1.0, 0.0), self.hud_proj_mat)
        text_y += y_offset

        if self.stopline_dist:
            text = f"STOPLINE DISTANCE: {self.stopline_dist:.2f}"
            text_w, text_h = self.shader_renderer.text_renderer.compute_text_size(text, 1.0)
            self.shader_renderer.text_renderer.render_text(text, text_x + text_w / 2, text_y + text_h / 2, 1.0, (0.0, 1.0, 0.0), self.hud_proj_mat)
            text_y += y_offset

        if self.stopline:
            text = "STOPLINE DETECTED"
            text_w, text_h = self.shader_renderer.text_renderer.compute_text_size(text, 1.0)
            self.shader_renderer.text_renderer.render_text(text, text_x + text_w / 2, text_y + text_h / 2, 1.0, (0.0, 1.0, 0.0), self.hud_proj_mat)
            text_y += y_offset

        if self.crosswalk:
            text = "CROSSWALK DETECTED"
            text_w, text_h = self.shader_renderer.text_renderer.compute_text_size(text, 1.0)
            self.shader_renderer.text_renderer.render_text(text, text_x + text_w / 2, text_y + text_h / 2, 1.0, (0.0, 1.0, 0.0), self.hud_proj_mat)

    def get_gl_coords(self, x_real, y_real, z_real):
        if hasattr(self, 'extrinsic'):
            cam_pt = glm.vec4(x_real, y_real, z_real, 1.0)
            world_pt = self.extrinsic * cam_pt
        return world_pt.x, world_pt.y, world_pt.z

    ################
    # Events
    ################

    def resizeGL(self, w, h):
        super().resizeGL(w, h)
        self.hud_proj_mat = glm.ortho(0.0, w, h, 0.0, -1.0, 1.0)
        self.update_camera_matrices(w, h)
