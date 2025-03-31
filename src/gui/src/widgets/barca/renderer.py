from OpenGL import GL as gl


def draw_axes():
    gl.glPushAttrib(gl.GL_ENABLE_BIT)
    gl.glPushMatrix()

    viewport = gl.glGetIntegerv(gl.GL_VIEWPORT)
    gl.glMatrixMode(gl.GL_PROJECTION)
    gl.glPushMatrix()
    gl.glLoadIdentity()
    gl.glOrtho(0, viewport[2], viewport[3], 0, -1, 1)

    gl.glMatrixMode(gl.GL_MODELVIEW)
    gl.glLoadIdentity()

    gl.glTranslatef(viewport[2] - 75, 75, 0)
    gl.glScalef(20, 20, 20)

    gl.glDisable(gl.GL_DEPTH_TEST)

    gl.glBegin(gl.GL_LINES)
    # Z-axis (Red)
    gl.glColor3f(1, 0, 0)
    gl.glVertex2f(0, 0)
    gl.glVertex2f(-1, 0.0)
    # Y-axis (Blue)
    gl.glColor3f(0, 0, 1)
    gl.glVertex2f(0, 0)
    gl.glVertex2f(0, -1)
    # X-axis (Green)
    gl.glColor3f(0, 1, 0)
    gl.glVertex2f(0, 0)
    gl.glVertex2f(-0.5, -0.5)
    gl.glEnd()

    gl.glMatrixMode(gl.GL_PROJECTION)
    gl.glPopMatrix()
    gl.glMatrixMode(gl.GL_MODELVIEW)
    gl.glPopMatrix()
    gl.glPopAttrib()
    gl.glEnable(gl.GL_DEPTH_TEST)


def draw_track(track_model):
    gl.glPushMatrix()

    gl.glEnable(gl.GL_BLEND)
    gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
    gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)
    gl.glColor4f(0.3, 0.3, 0.3, 0.3)
    gl.glLineWidth(0.01)

    track_model.vbo.bind()
    gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
    gl.glVertexPointer(3, gl.GL_FLOAT, 0, track_model.vbo)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, track_model.vertex_count)
    gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
    track_model.vbo.unbind()

    gl.glDisable(gl.GL_BLEND)
    gl.glPopMatrix()
