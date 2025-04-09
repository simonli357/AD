from OpenGL import GL as gl
from ..enums import NamedColor

import glm


class HudRenderer:
    def __init__(self, shader_renderer):
        self.shader_renderer = shader_renderer

    def draw_hud(self):
        gl.glDisable(gl.GL_DEPTH_TEST)

        viewport = gl.glGetIntegerv(gl.GL_VIEWPORT)
        screen_width = viewport[2]
        screen_height = viewport[3]
        self.proj_mat = glm.ortho(0.0, float(screen_width), float(screen_height), 0.0, -1.0, 1.0)

        self.shader_renderer.progress_bar_model.draw_I(
            x=0.98 * screen_width,
            y=0.02 * screen_height,
            width=0.25 * screen_width,
            height=0.01 * screen_height,
            fill_color=NamedColor.GREEN.value,
            percentage=0.5,
            proj_mat=self.proj_mat
        )

        # Re-enable depth testing if subsequent rendering requires it
        gl.glEnable(gl.GL_DEPTH_TEST)
