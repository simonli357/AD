from OpenGL import GL as gl


def draw_track(track_model):
    gl.glPushMatrix()
    gl.glPushAttrib(gl.GL_CURRENT_BIT)

    gl.glColor4f(0.0, 1.0, 0.0, 0.5)

    track_model.vbo.bind()
    gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
    gl.glVertexPointer(3, gl.GL_FLOAT, 0, track_model.vbo)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, track_model.vertex_count)
    gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
    track_model.vbo.unbind()

    gl.glPopAttrib()
    gl.glPopMatrix()


def draw_grid(grid_model):
    grid_vbo = grid_model[0]
    grid_vertex_count = grid_model[1]
    gl.glPushMatrix()
    gl.glPushAttrib(gl.GL_CURRENT_BIT)

    gl.glColor3f(0.3, 0.3, 0.3)

    grid_vbo.bind()
    gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
    gl.glVertexPointer(3, gl.GL_FLOAT, 0, grid_vbo)
    gl.glDrawArrays(gl.GL_LINES, 0, grid_vertex_count)
    gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
    grid_vbo.unbind()

    gl.glPopAttrib()
    gl.glPopMatrix()
