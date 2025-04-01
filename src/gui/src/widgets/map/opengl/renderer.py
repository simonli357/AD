from OpenGL import GL as gl


def draw_track(texture, track_vbo, track_vertex_count):
    gl.glPushMatrix()

    gl.glEnable(gl.GL_TEXTURE_2D)
    gl.glBindTexture(gl.GL_TEXTURE_2D, texture)
    gl.glEnable(gl.GL_BLEND)
    gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)

    track_vbo.bind()

    gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
    gl.glEnableClientState(gl.GL_TEXTURE_COORD_ARRAY)
    # Stride = 20 bytes (5 floats * 4 bytes each)
    gl.glVertexPointer(3, gl.GL_FLOAT, 20, track_vbo)
    # TexCoord offset = 12 bytes (3 floats into the vertex data)
    gl.glTexCoordPointer(2, gl.GL_FLOAT, 20, track_vbo + 12)

    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_CLAMP_TO_BORDER)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_CLAMP_TO_BORDER)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_NEAREST)

    gl.glDrawArrays(gl.GL_QUADS, 0, track_vertex_count)

    track_vbo.unbind()

    gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
    gl.glDisableClientState(gl.GL_TEXTURE_COORD_ARRAY)
    gl.glDisable(gl.GL_TEXTURE_2D)

    gl.glPopMatrix()


def draw_grid(grid_vbo, grid_vertex_count):
    gl.glPushMatrix()
    gl.glColor3f(0.3, 0.3, 0.3)

    grid_vbo.bind()
    gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
    gl.glVertexPointer(3, gl.GL_FLOAT, 0, grid_vbo)
    gl.glDrawArrays(gl.GL_LINES, 0, grid_vertex_count)
    gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
    grid_vbo.unbind()

    gl.glPopMatrix()
