import numpy as np
from OpenGL import GL as gl
import glm


class ProgressBar:
    """Custom progress bar drawing a sheared parallelogram with a fill fraction."""

    def __init__(self, text_renderer, shader_program, texture_shader):
        self.text_renderer = text_renderer
        self.shader_program = shader_program
        self.texture_shader = texture_shader

        # shear as fraction of full width
        self.shear_fraction = 0.025
        self.backdrop_color = (1.0, 1.0, 1.0, 0.5)

        # ——— 1) Build a unit “parallelogram” VAO/VBO once ———
        # vertices in (u,v) space: (0,0)=top‑left, (1,0)=top‑right, (1,1)=bottom‑right, (0,1)=bottom‑left
        quad_uv = np.array([
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0, 1.0],
            [0.0, 0.0],
            [1.0, 1.0],
            [0.0, 1.0],
        ], dtype=np.float32).flatten()

        self.vao = gl.glGenVertexArrays(1)
        self.vbo = gl.glGenBuffers(1)

        gl.glBindVertexArray(self.vao)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, quad_uv.nbytes, quad_uv, gl.GL_STATIC_DRAW)

        pos_loc = gl.glGetAttribLocation(self.shader_program, "aUV")
        gl.glEnableVertexAttribArray(pos_loc)
        gl.glVertexAttribPointer(pos_loc, 2, gl.GL_FLOAT, gl.GL_FALSE, 2 * 4, gl.ctypes.c_void_p(0))

        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
        gl.glBindVertexArray(0)

        # ——— 2) Cache all uniform locations ———
        self.uProj_loc = gl.glGetUniformLocation(self.shader_program, "uProjection")
        self.uRect_loc = gl.glGetUniformLocation(self.shader_program, "uRect")      # vec4(x0, y0, width, height)
        self.uShear_loc = gl.glGetUniformLocation(self.shader_program, "uShear")     # float
        self.uFillFrac_loc = gl.glGetUniformLocation(self.shader_program, "uFillFrac")  # float
        self.uColor_loc = gl.glGetUniformLocation(self.shader_program, "uColor")

        self.texture_u_model = gl.glGetUniformLocation(self.texture_shader, "model")
        self.texture_u_proj = gl.glGetUniformLocation(self.texture_shader, "projection")

    def draw_texture(self, x, y, icon, scale, proj_matrix):
        gl.glUseProgram(self.texture_shader)

        # Set matrices
        model = glm.mat4(1.0)
        model = glm.translate(model, glm.vec3(x, y, 0))
        model = glm.scale(model, glm.vec3(scale, scale, 1.0))
        model = glm.rotate(model, np.radians(180), glm.vec3(0, 0, 1))

        gl.glUniformMatrix4fv(self.texture_u_model, 1, gl.GL_FALSE, glm.value_ptr(model))
        gl.glUniformMatrix4fv(self.texture_u_proj, 1, gl.GL_FALSE, glm.value_ptr(proj_matrix))

        # Bind texture
        gl.glActiveTexture(gl.GL_TEXTURE0)
        gl.glBindTexture(gl.GL_TEXTURE_2D, icon.texture_id)
        gl.glUniform1i(gl.glGetUniformLocation(self.texture_shader, "texture1"), 0)

        # Draw
        gl.glBindVertexArray(icon.vao)
        gl.glDrawElements(gl.GL_TRIANGLES, 6, gl.GL_UNSIGNED_INT, None)
        gl.glBindVertexArray(0)

    def draw(self, screen_width, screen_height, x_norm, y_norm, width_norm, height_norm, fill_color, percentage, proj_mat, icon):
        # ——— 3) Denormalize and compute constants once ———
        x_px = x_norm * screen_width
        y_px = y_norm * screen_height
        w_px = width_norm * screen_width
        h_px = height_norm * screen_height
        shear = self.shear_fraction * w_px

        # projection matrix
        proj = glm.value_ptr(proj_mat)

        gl.glUseProgram(self.shader_program)

        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)

        gl.glBindVertexArray(self.vao)

        # set projection
        gl.glUniformMatrix4fv(self.uProj_loc, 1, gl.GL_FALSE, proj)

        # — draw backdrop (fillFrac=1.0) —
        gl.glUniform4f(self.uRect_loc, x_px - w_px, y_px, w_px, h_px)
        gl.glUniform1f(self.uShear_loc, shear)
        gl.glUniform1f(self.uFillFrac_loc, 1.0)
        gl.glUniform4f(self.uColor_loc, *self.backdrop_color)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, 6)

        # — draw filled portion —
        gl.glUniform1f(self.uFillFrac_loc, percentage)
        gl.glUniform4f(self.uColor_loc, *fill_color)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, 6)

        gl.glBindVertexArray(0)
        gl.glUseProgram(0)

        # — draw icon + text on top —
        # 1) text string
        text_str = f"{percentage * 100:.0f}%"

        # 2) compute text position at bar’s center
        text_x = x_px - 0.6 * w_px
        text_y = y_px - 0.5 * h_px

        # 3) compute icon position just to the left of the text
        bar_left_x = x_px - w_px
        midpoint_x = (text_x + bar_left_x) * 0.5
        icon_size = h_px * 5
        icon_x = midpoint_x - icon_size * 0.20
        icon_y = y_px + h_px - 0.3 * icon_size

        # 4) draw the icon (texture_shader must expect `model` + `projection` uniforms)
        self.draw_texture(icon_x, icon_y, icon, icon_size, proj_mat)

        # 5) render the text (centered at text_x,text_y)
        self.text_renderer.render_text(
            text_str,
            text_x, text_y,
            1.0,                       # scale
            (1.0, 1.0, 1.0, 1.0),      # white color RGBA
            proj_mat
        )

        gl.glDisable(gl.GL_BLEND)
