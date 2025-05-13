from ..enums import NamedColor


class HudRenderer:
    def __init__(self, car_widget):
        self.car_widget = car_widget
        self.shader_renderer = car_widget.shader_renderer

        self.cores_usage = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        self.ram_usage = 0.5
        self.temperature = 20.0
        self.heap_usage = 0.5
        self.stack_usage = 0.5

    def draw_hud(self, proj_mat, screen_width, screen_height):
        # Temperature
        termo_icon_x = 0.47 * screen_width
        termo_icon_y = (0.64 + 0.06 * 2) * screen_height
        termo_text_x = 0.53 * screen_width
        termo_text_y = termo_icon_y
        self.shader_renderer.draw_texture2D(termo_icon_x, termo_icon_y, self.shader_renderer.thermometer_texture, 25, proj_mat)
        self.shader_renderer.text_renderer.render_text(f"{self.temperature:.0f}°C", termo_text_x, termo_text_y, 1.0, (1.0, 0.0, 0.0), proj_mat)

        bar_width = 0.2
        bar_height = 0.01
        x_norm_init = 0.77
        y_norm_init = 0.64 + 0.06 * 3

        # HEAP
        self.shader_renderer.progress_bar_model.draw(
            screen_width=screen_width,
            screen_height=screen_height,
            x_norm=x_norm_init,
            y_norm=y_norm_init,
            width_norm=bar_width,
            height_norm=bar_height,
            fill_color=NamedColor.GREEN.value,
            percentage=self.heap_usage,
            proj_mat=proj_mat,
            icon=self.car_widget.shader_renderer.heap_texture
        )
        y_norm_init += 0.06

        # STACK
        self.shader_renderer.progress_bar_model.draw(
            screen_width=screen_width,
            screen_height=screen_height,
            x_norm=x_norm_init,
            y_norm=y_norm_init,
            width_norm=bar_width,
            height_norm=bar_height,
            fill_color=NamedColor.ORANGE.value,
            percentage=self.stack_usage,
            proj_mat=proj_mat,
            icon=self.car_widget.shader_renderer.stack_texture
        )
        y_norm_init += 0.06

        # RAM
        self.shader_renderer.progress_bar_model.draw(
            screen_width=screen_width,
            screen_height=screen_height,
            x_norm=x_norm_init,
            y_norm=y_norm_init,
            width_norm=bar_width,
            height_norm=bar_height,
            fill_color=NamedColor.INDIGO.value,
            percentage=self.ram_usage,
            proj_mat=proj_mat,
            icon=self.car_widget.shader_renderer.ram_texture
        )
        y_norm_init += 0.06

        x_norm_init = 1.09
        y_norm_init = 0.64
        # CPU Stats
        for use in self.cores_usage[:6]:
            self.shader_renderer.progress_bar_model.draw(
                screen_width=screen_width,
                screen_height=screen_height,
                x_norm=x_norm_init,
                y_norm=y_norm_init,
                width_norm=bar_width,
                height_norm=bar_height,
                fill_color=NamedColor.STEEL_BLUE.value,
                percentage=use,
                proj_mat=proj_mat,
                icon=self.car_widget.shader_renderer.cpu_texture
            )
            y_norm_init += 0.06

        # Speedometer
        self.shader_renderer.speedometer_model.draw(
            screen_width,
            screen_height,
            0.18,
            0.73,
            proj_mat,
            self.car_widget.speed,
            self.car_widget.steer
        )

        self.shader_renderer.compass_model.draw(
            screen_width,
            screen_height,
            0.85,
            0.22,
            proj_mat,
            self.car_widget.yaw
        )
