from OpenGL import GL as gl
from .loaders import load_obj, load_texture

import os


current_dir = os.path.dirname(os.path.abspath(__file__))
asset_dir = os.path.join(current_dir, 'assets')


def get_asset_path(asset_file_name):
    return os.path.join(asset_dir, asset_file_name)


class GlobalRenderer:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(GlobalRenderer, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if hasattr(self, '_initialized'):
            return
        self._initialized = True
        self.load_vbos()
        self.load_textures()

    def load_vbos(self):
        self.car_model = load_obj(get_asset_path('car.obj'))
        self.barca_track = load_obj(get_asset_path('track.obj'))

    def load_textures(self):
        self.bfmc_track_texture = load_texture(get_asset_path('track.png'))

    #####################
    # Draw functions
    #####################

    def draw_2D_texture(self, texture, vbo, x=0, y=0, scale=1.0):
        if texture is None or vbo is None:
            return
        gl.glPushMatrix()

        gl.glEnable(gl.GL_TEXTURE_2D)
        gl.glBindTexture(gl.GL_TEXTURE_2D, texture)

        vbo.bind()

        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glEnableClientState(gl.GL_TEXTURE_COORD_ARRAY)
        # Stride = 20 bytes (5 floats * 4 bytes each)
        gl.glVertexPointer(3, gl.GL_FLOAT, 20, vbo)
        # TexCoord offset = 12 bytes (3 floats into the vertex data)
        gl.glTexCoordPointer(2, gl.GL_FLOAT, 20, vbo + 12)

        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_CLAMP_TO_BORDER)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_CLAMP_TO_BORDER)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_NEAREST)

        gl.glTranslatef(x, y, 0)
        gl.glScalef(scale, scale, scale)
        gl.glDrawArrays(gl.GL_QUADS, 0, 4)

        vbo.unbind()

        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        gl.glDisableClientState(gl.GL_TEXTURE_COORD_ARRAY)
        gl.glDisable(gl.GL_TEXTURE_2D)

        gl.glPopMatrix()

    def draw_car(self, x, y, yaw, scale, color: (float, float, float, float)):
        gl.glPushMatrix()
        gl.glPushAttrib(gl.GL_CURRENT_BIT)
        gl.glTranslatef(x, y, 0)
        gl.glRotatef(yaw, 0, 0, 1)
        gl.glScalef(scale, scale, scale)
        gl.glColor4f(*color)
        self.car_model.vbo.bind()
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, self.car_model.vbo)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.car_model.vertex_count)
        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        self.car_model.vbo.unbind()
        gl.glPopAttrib()
        gl.glPopMatrix()
