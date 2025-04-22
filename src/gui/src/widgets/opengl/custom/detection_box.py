from OpenGL import GL as gl
import numpy as np


class DetectionBox:
    def __init__(self, text_renderer, shader_program):
        self.text_renderer = text_renderer
        self.shader_program = shader_program

        # ——— 1) Create a unit‐quad VAO/VBO once ———
        #    geometry: (0,0), (1,0), (1,1), (0,1)
        unit_quad = np.array([
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0, 1.0],
            [0.0, 1.0],
        ], dtype=np.float32)

        self.vao = gl.glGenVertexArrays(1)
        self.vbo = gl.glGenBuffers(1)

        gl.glBindVertexArray(self.vao)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.vbo)
        gl.glBufferData(
            gl.GL_ARRAY_BUFFER,
            unit_quad.nbytes,
            unit_quad,
            gl.GL_STATIC_DRAW
        )

        pos_loc = gl.glGetAttribLocation(shader_program, "aPos")
        gl.glEnableVertexAttribArray(pos_loc)
        gl.glVertexAttribPointer(pos_loc, 2, gl.GL_FLOAT, gl.GL_FALSE, 0, gl.ctypes.c_void_p(0))

        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
        gl.glBindVertexArray(0)

        # ——— 2) Cache uniform locations ———
        self.u_proj_loc = gl.glGetUniformLocation(shader_program, "uProjection")
        self.u_color_loc = gl.glGetUniformLocation(shader_program, "uColor")
        self.u_rect_loc = gl.glGetUniformLocation(shader_program, "uRect")

    def draw(self, x1, y1, x2, y2, label_text, line_width, color, proj_mat):
        gl.glUseProgram(self.shader_program)
        gl.glBindVertexArray(self.vao)

        # — set projection once —
        proj_array = np.array(proj_mat.to_list(), dtype=np.float32)
        gl.glUniformMatrix4fv(self.u_proj_loc, 1, gl.GL_FALSE, proj_array)

        # — draw the box outline —
        gl.glLineWidth(line_width)
        # rect: (x, y, width, height)
        gl.glUniform4f(self.u_rect_loc, x1, y1, x2 - x1, y2 - y1)
        gl.glUniform4f(self.u_color_loc, *color, 1.0)
        gl.glDrawArrays(gl.GL_LINE_LOOP, 0, 4)

        # --- Draw the background quad ——
        tw, th = self.text_renderer.compute_text_size(label_text, 1.0)
        pad = 5.0

        bw = max((x2 - x1), tw + pad * 2)
        bh = th + pad * 2

        bg_x, bg_y = x1, y1 - bh

        gl.glUniform4f(self.u_rect_loc, bg_x, bg_y, bw, bh)
        gl.glUniform4f(self.u_color_loc, *color, 0.5)
        gl.glDrawArrays(gl.GL_TRIANGLE_FAN, 0, 4)

        # center your text in that quad:
        text_x = bg_x + bw * 0.5
        text_y = bg_y + bh * 0.5
        self.text_renderer.render_text(label_text, text_x, text_y, 1.0, color, proj_mat)
