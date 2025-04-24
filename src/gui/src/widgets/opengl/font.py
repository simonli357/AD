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
        self._init_render_data()
        self.text_shader = create_shader_program(shader_path('text', 'text.vert'), shader_path('text', 'text.frag'))

        self.loc_proj = gl.glGetUniformLocation(self.text_shader, "projection")
        self.loc_textColor = gl.glGetUniformLocation(self.text_shader, "textColor")
        self.loc_sampler = gl.glGetUniformLocation(self.text_shader, "text")

    def _init_freetype(self, font_path, pixel_size):
        face = freetype.Face(font_path)
        face.set_pixel_sizes(0, pixel_size)
        gl.glPixelStorei(gl.GL_UNPACK_ALIGNMENT, 1)
        for c in range(256):
            char = chr(c)
            try:
                face.load_char(char, freetype.FT_LOAD_RENDER)
            except freetype.FT_Error:
                print(f"Failed to load character: {char}")
                continue
            bitmap = face.glyph.bitmap
            w, h = bitmap.width, bitmap.rows

            texture = gl.glGenTextures(1)
            gl.glBindTexture(gl.GL_TEXTURE_2D, texture)
            gl.glTexImage2D(
                gl.GL_TEXTURE_2D,
                0,
                gl.GL_RED,
                w,
                h,
                0,
                gl.GL_RED,
                gl.GL_UNSIGNED_BYTE,
                bitmap.buffer
            )
            gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_CLAMP_TO_EDGE)
            gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_CLAMP_TO_EDGE)
            gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
            gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)
            gl.glBindTexture(gl.GL_TEXTURE_2D, 0)

            self.characters[char] = {
                'texture': texture,
                'size': (w, h),
                'bearing': (face.glyph.bitmap_left, face.glyph.bitmap_top),
                'advance': face.glyph.advance.x
            }

    def _init_render_data(self):
        self.VAO = gl.glGenVertexArrays(1)
        self.VBO = gl.glGenBuffers(1)
        gl.glBindVertexArray(self.VAO)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.VBO)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, 6 * 4 * 4, None, gl.GL_DYNAMIC_DRAW)
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 4, gl.GL_FLOAT, gl.GL_FALSE, 4 * 4, gl.ctypes.c_void_p(0))
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
        gl.glBindVertexArray(0)

    def compute_text_size(self, text, scale):
        """Compute the width and height of the given text (in pixels) when rendered at the given scale."""
        total_width = 0
        max_height = 0
        for c in text:
            ch = self.characters.get(c)
            if ch is None:
                continue
            # Advance is in 1/64 pixels; convert to pixels and multiply by scale.
            total_width += (ch['advance'] >> 6) * scale
            # Use the glyph's height (in pixels) multiplied by scale.
            h = ch['size'][1] * scale
            if h > max_height:
                max_height = h
        return total_width, max_height

    def render_text(self, text, x, y, scale, color, projection):
        """
        Render a string of text onto the screen, centered at (x,y).

        :param text: The string to render.
        :param x: x position in pixels where the text will be centered.
        :param y: y position in pixels (measured from the bottom) where the text will be centered.
        :param scale: Scale factor.
        :param color: A tuple (r, g, b) with values in [0.0, 1.0].
        :param projection: A glm.mat4 projection matrix.
        """
        # First compute the size of the rendered text.
        text_width, text_height = self.compute_text_size(text, scale)
        # Adjust x and y to get the starting (baseline) position so the full text is centered.
        x = x - text_width / 2.0
        y = y - text_height / 2.0

        gl.glUseProgram(self.text_shader)

        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)

        gl.glUniformMatrix4fv(self.loc_proj, 1, gl.GL_FALSE, glm.value_ptr(projection))
        gl.glUniform3f(self.loc_textColor, color[0], color[1], color[2])
        # Ensure that texture unit 0 is used.
        gl.glUniform1i(self.loc_sampler, 0)
        gl.glActiveTexture(gl.GL_TEXTURE0)
        gl.glBindVertexArray(self.VAO)

        # Render each character.
        for c in text:
            ch = self.characters.get(c)
            if ch is None:
                continue

            # Compute the glyph's quad positions.
            xpos = x + ch['bearing'][0] * scale
            # Adjust y: since y is the baseline (from bottom), we place the quad so that the bottom is at y.
            ypos = y - (ch['size'][1] - ch['bearing'][1]) * scale

            if c == '°' and self.pixel_size == 16:
                # Increase ypos by a fraction of the glyph height (tweak this factor as needed).
                ypos -= 1.7 * ch['size'][1] * scale
            if c == '.' and self.pixel_size == 16:
                # Increase ypos by a fraction of the glyph height (tweak this factor as needed).
                ypos += 5 * ch['size'][1] * scale
            if c == '.' and self.pixel_size == 48:
                # Increase ypos by a fraction of the glyph height (tweak this factor as needed).
                ypos += 4.2 * ch['size'][1] * scale
            if c == '-' and self.pixel_size == 16:
                ypos += 0.8 * ch['size'][1] * scale

            w = ch['size'][0] * scale
            h = ch['size'][1] * scale

            # Build vertices for the glyph quad with bottom-left origin.
            vertices = np.array([
                xpos, ypos, 0.0, 0.0,  # Bottom-left
                xpos, ypos + h, 0.0, 1.0,  # Top-left
                xpos + w, ypos + h, 1.0, 1.0,  # Top-right

                xpos, ypos, 0.0, 0.0,  # Bottom-left
                xpos + w, ypos + h, 1.0, 1.0,  # Top-right
                xpos + w, ypos, 1.0, 0.0   # Bottom-right
            ], dtype=np.float32)

            gl.glBindTexture(gl.GL_TEXTURE_2D, ch['texture'])
            gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.VBO)
            gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 0, vertices.nbytes, vertices)
            gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
            gl.glDrawArrays(gl.GL_TRIANGLES, 0, 6)

            # Advance x for the next character.
            x += (ch['advance'] >> 6) * scale

        gl.glBindVertexArray(0)
        gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
        gl.glDisable(gl.GL_BLEND)
        gl.glUseProgram(0)
