import freetype
import numpy as np
from OpenGL import GL as gl
from OpenGL.GL.shaders import compileProgram, compileShader
import glm
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
shader_dir = os.path.join(current_dir, 'shaders')


def shader_path(dirname: str, filename: str):
    return os.path.join(shader_dir, dirname, filename)


def create_shader_program(vertex_filepath: str, fragment_filepath: str) -> int:
    with open(vertex_filepath, "r") as f:
        vertex_source = f.read()
    with open(fragment_filepath, "r") as f:
        fragment_source = f.read()
    vertex_shader = compileShader(vertex_source, gl.GL_VERTEX_SHADER)
    fragment_shader = compileShader(fragment_source, gl.GL_FRAGMENT_SHADER)
    program = compileProgram(vertex_shader, fragment_shader)
    gl.glDeleteShader(vertex_shader)
    gl.glDeleteShader(fragment_shader)
    return program


class TextRenderer:
    def __init__(self, pixel_size):
        self.characters = {}
        font_path = shader_path('text', 'Arial.ttf')
        self._init_freetype(font_path, pixel_size)
        self._init_render_data()
        self.text_shader = create_shader_program(
            shader_path('text', 'text.vert'),
            shader_path('text', 'text.frag')
        )

    def _init_freetype(self, font_path, pixel_size):
        face = freetype.Face(font_path)
        face.set_pixel_sizes(0, pixel_size)
        gl.glPixelStorei(gl.GL_UNPACK_ALIGNMENT, 1)
        for c in range(128):
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

    def render_text(self, text, x, y, scale, color, projection):
        """
        Render a string of text onto the screen.

        :param text: The string to render.
        :param x: x position in pixels.
        :param y: y position in pixels (measured from the bottom).
        :param scale: Scale factor.
        :param color: A tuple (r, g, b) with values in [0.0, 1.0].
        :param projection: A glm.mat4 projection matrix.
        """
        gl.glUseProgram(self.text_shader)
        loc_proj = gl.glGetUniformLocation(self.text_shader, "projection")
        gl.glUniformMatrix4fv(loc_proj, 1, gl.GL_FALSE, glm.value_ptr(projection))
        loc_textColor = gl.glGetUniformLocation(self.text_shader, "textColor")
        gl.glUniform3f(loc_textColor, color[0], color[1], color[2])
        gl.glActiveTexture(gl.GL_TEXTURE0)
        gl.glBindVertexArray(self.VAO)

        for c in text:
            ch = self.characters.get(c)
            if ch is None:
                continue

            xpos = x + ch['bearing'][0] * scale
            # y is the baseline; adjust to get bottom-left.
            ypos = y - (ch['size'][1] - ch['bearing'][1]) * scale

            w = ch['size'][0] * scale
            h = ch['size'][1] * scale

            # Build vertices with the **bottom-left origin first** (flip the quad vertically):
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
            x += (ch['advance'] >> 6) * scale

        gl.glBindVertexArray(0)
        gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
        gl.glUseProgram(0)
