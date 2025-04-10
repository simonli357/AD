from OpenGL import GL as gl
from ..enums import NamedColor

import glm


class HudRenderer:
    def __init__(self, shader_renderer):
        self.shader_renderer = shader_renderer

    def draw_hud(self):
        viewport = gl.glGetIntegerv(gl.GL_VIEWPORT)
        screen_width = viewport[2]
        screen_height = viewport[3]
        self.proj_mat = glm.ortho(0.0, float(screen_width), float(screen_height), 0.0, -1.0, 1.0)

        self.shader_renderer.progress_bar_model.draw(
            screen_width=screen_width,
            screen_height=screen_height,
            x_norm=0.98,
            y_norm=0.02,
            width_norm=0.25,
            height_norm=0.01,
            fill_color=NamedColor.GREEN.value,
            percentage=1.0,
            proj_mat=self.proj_mat
        )

        self.shader_renderer.speedometer_model.draw(
            screen_width,
            screen_height,
            0.18,
            0.7,
            self.proj_mat
        )
