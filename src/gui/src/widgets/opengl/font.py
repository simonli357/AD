from OpenGL import GL as gl
from .utils import shader_path, create_shader_program

import freetype
import numpy as np
import glm


class TextRenderer:
    def __init__(self, pixel_size):
        self.characters = {}
        self.pixel_size = pixel_size

        font_path = shader_path('text', 'Arial.ttf')
        self._init_freetype(font_path, pixel_size)

        self.text_shader = create_shader_program(shader_path('text', 'text.vert'), shader_path('text', 'text.frag'))

        self.text3d_shader = create_shader_program(shader_path('text', 'text3D.vert'), shader_path('text', 'text.frag'))

        self.loc_proj = gl.glGetUniformLocation(self.text_shader, "projection")
        self.loc_textColor = gl.glGetUniformLocation(self.text_shader, "textColor")
        self.loc_sampler = gl.glGetUniformLocation(self.text_shader, "text")
        self.loc_offset = gl.glGetUniformLocation(self.text_shader, "textOffset")

        self.loc3_proj = gl.glGetUniformLocation(self.text3d_shader, "projection")
        self.loc3_view = gl.glGetUniformLocation(self.text3d_shader, "view")
        self.loc3_model = gl.glGetUniformLocation(self.text3d_shader, "model")
        self.loc3_color = gl.glGetUniformLocation(self.text3d_shader, "textColor")
        self.loc3_samp = gl.glGetUniformLocation(self.text3d_shader, "text")

    def _init_freetype(self, font_path, pixel_size):
        face = freetype.Face(font_path)
        face.set_pixel_sizes(0, pixel_size)
        gl.glPixelStorei(gl.GL_UNPACK_ALIGNMENT, 1)

        for c in range(256):
            char = chr(c)
            try:
                face.load_char(char, freetype.FT_LOAD_RENDER)
            except freetype.FT_Error:
                continue

            bitmap = face.glyph.bitmap
            w, h = bitmap.width, bitmap.rows

            tex = gl.glGenTextures(1)
            gl.glBindTexture(gl.GL_TEXTURE_2D, tex)
            gl.glTexImage2D(
                gl.GL_TEXTURE_2D, 0, gl.GL_RED,
                w, h, 0, gl.GL_RED, gl.GL_UNSIGNED_BYTE,
                bitmap.buffer
            )
            gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_CLAMP_TO_EDGE)
            gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_CLAMP_TO_EDGE)
            gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
            gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)
            gl.glBindTexture(gl.GL_TEXTURE_2D, 0)

            vertices = np.array([
                # x,    y,    u,  v
                0.0, 0.0, 0.0, 0.0,
                0.0, h, 0.0, 1.0,
                w, h, 1.0, 1.0,

                0.0, 0.0, 0.0, 0.0,
                w, h, 1.0, 1.0,
                w, 0.0, 1.0, 0.0,
            ], dtype=np.float32)

            vao = gl.glGenVertexArrays(1)
            vbo = gl.glGenBuffers(1)
            gl.glBindVertexArray(vao)
            gl.glBindBuffer(gl.GL_ARRAY_BUFFER, vbo)
            gl.glBufferData(gl.GL_ARRAY_BUFFER, vertices.nbytes, vertices, gl.GL_STATIC_DRAW)
            gl.glEnableVertexAttribArray(0)
            gl.glVertexAttribPointer(0, 4, gl.GL_FLOAT, gl.GL_FALSE, 4 * 4, gl.ctypes.c_void_p(0))
            gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
            gl.glBindVertexArray(0)

            self.characters[char] = {
                'texture': tex,
                'VAO': vao,
                'VBO': vbo,
                'size': (w, h),
                'bearing': (face.glyph.bitmap_left, face.glyph.bitmap_top),
                'advance': face.glyph.advance.x
            }

    def compute_text_size(self, text, scale):
        total_w, max_h = 0, 0
        for c in text:
            ch = self.characters.get(c)
            if not ch:
                continue
            total_w += (ch['advance'] >> 6) * scale
            max_h = max(max_h, ch['size'][1] * scale)
        return total_w, max_h

    def render_text(self, text, x, y, scale=1.0, color=(1, 1, 1), projection=None):
        text_w, text_h = self.compute_text_size(text, scale)
        x -= text_w / 2.0
        y -= text_h / 2.0

        gl.glUseProgram(self.text_shader)
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)

        gl.glUniformMatrix4fv(self.loc_proj, 1, gl.GL_FALSE, glm.value_ptr(projection))
        gl.glUniform3f(self.loc_textColor, color[0], color[1], color[2])
        gl.glUniform1i(self.loc_sampler, 0)

        for c in text:
            ch = self.characters.get(c)
            if not ch:
                continue

            w, h = ch['size']
            bx, by = ch['bearing']
            xpos = x + bx * scale
            ypos = y - (h - by) * scale

            # tweak special glyphs (° . -) if needed
            if c == '°' and self.pixel_size == 16:
                ypos -= 1.7 * h * scale
            if c == '.' and self.pixel_size == 16:
                ypos += 5 * h * scale
            if c == '.' and self.pixel_size == 48:
                ypos += 4.2 * h * scale
            if c == '-' and self.pixel_size == 16:
                ypos += 0.8 * h * scale

            gl.glUniform2f(self.loc_offset, xpos, ypos)

            gl.glActiveTexture(gl.GL_TEXTURE0)
            gl.glBindTexture(gl.GL_TEXTURE_2D, ch['texture'])
            gl.glBindVertexArray(ch['VAO'])
            gl.glDrawArrays(gl.GL_TRIANGLES, 0, 6)

            x += (ch['advance'] >> 6) * scale

        gl.glBindVertexArray(0)
        gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
        gl.glDisable(gl.GL_BLEND)
        gl.glUseProgram(0)

    def render_text3D(self, text, x, y, z, scale=1.0, color=(1, 1, 1), proj_mat=None, view_mat=None):
        """ Render `text` at world‐space (x,y,z), always facing the camera. """
        gl.glUseProgram(self.text3d_shader)
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)

        gl.glUniformMatrix4fv(self.loc3_proj, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))
        gl.glUniformMatrix4fv(self.loc3_view, 1, gl.GL_FALSE, glm.value_ptr(view_mat))
        gl.glUniform3f(self.loc3_color, *color)
        gl.glUniform1i(self.loc3_samp, 0)

        cam_right = glm.vec3(view_mat[0][0], view_mat[1][0], view_mat[2][0])
        cam_up = glm.vec3(view_mat[0][1], view_mat[1][1], view_mat[2][1])

        cursor = 0.0
        for c in text:
            ch = self.characters.get(c)
            if not ch:
                continue

            w, h = ch['size']
            bx, by = ch['bearing']
            adv = (ch['advance'] >> 6)

            w *= scale
            h *= scale
            bx *= scale
            by *= scale
            adv *= scale

            mat_scale = glm.mat4(1.0)
            mat_scale[0][0], mat_scale[1][0], mat_scale[2][0] = cam_right * w
            mat_scale[0][1], mat_scale[1][1], mat_scale[2][1] = cam_up * h

            world_offset = cam_right * (cursor + bx) + cam_up * by
            world_pos = glm.vec3(x, y, z) + world_offset
            mat_trans = glm.translate(glm.mat4(1.0), world_pos)

            model = mat_trans * mat_scale
            gl.glUniformMatrix4fv(self.loc3_model, 1, gl.GL_FALSE, glm.value_ptr(model))

            gl.glActiveTexture(gl.GL_TEXTURE0)
            gl.glBindTexture(gl.GL_TEXTURE_2D, ch['texture'])
            gl.glBindVertexArray(ch['VAO'])
            gl.glDrawArrays(gl.GL_TRIANGLES, 0, 6)

            cursor += adv

        gl.glBindVertexArray(0)
        gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
        gl.glDisable(gl.GL_BLEND)
        gl.glUseProgram(0)
