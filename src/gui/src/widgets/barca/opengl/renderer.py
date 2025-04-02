from OpenGL import GL as gl


def draw_track(track_model):
    gl.glPushMatrix()

    gl.glEnable(gl.GL_BLEND)
    gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
    gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)
    gl.glColor4f(0.0, 1.0, 0.0, 0.5)
    gl.glLineWidth(0.01)

    track_model.vbo.bind()
    gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
    gl.glVertexPointer(3, gl.GL_FLOAT, 0, track_model.vbo)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, track_model.vertex_count)
    gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
    track_model.vbo.unbind()

    gl.glDisable(gl.GL_BLEND)
    gl.glPopMatrix()


def draw_grid(grid_model):
    grid_vbo = grid_model[0]
    grid_vertex_count = grid_model[1]
    gl.glPushMatrix()
    gl.glColor3f(0.3, 0.3, 0.3)

    grid_vbo.bind()
    gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
    gl.glVertexPointer(3, gl.GL_FLOAT, 0, grid_vbo)
    gl.glDrawArrays(gl.GL_LINES, 0, grid_vertex_count)
    gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
    grid_vbo.unbind()

    gl.glPopMatrix()
