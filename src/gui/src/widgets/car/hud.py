from ..enums import NamedColor


class HudRenderer:
    def __init__(self, car_widget):
        self.car_widget = car_widget
        self.shader_renderer = car_widget.shader_renderer

    def draw_hud(self, proj_mat, screen_width, screen_height):
        self.shader_renderer.progress_bar_model.draw(
            screen_width=screen_width,
            screen_height=screen_height,
            x_norm=0.98,
            y_norm=0.02,
            width_norm=0.25,
            height_norm=0.01,
            fill_color=NamedColor.GREEN.value,
            percentage=1.0,
            proj_mat=proj_mat
        )

        self.shader_renderer.speedometer_model.draw(
            screen_width,
            screen_height,
            0.19,
            0.7,
            proj_mat,
            self.car_widget.speed,
            self.car_widget.steer
        )
