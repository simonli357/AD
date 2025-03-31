from OpenGL import GL as gl
from ..enums import BarcaMapData


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


def draw_waypoint(x, y, wp_vbo):
    gl.glPushMatrix()
    gl.glColor3f(1, 1, 0)
    gl.glRotatef(90, 0.0, 0.0, 1.0)
    gl.glTranslatef(-BarcaMapData.MAP_CENTER_X.value + x, -BarcaMapData.MAP_CENTER_Y.value + y, 0)

    wp_vbo.bind()
    gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
    gl.glVertexPointer(3, gl.GL_FLOAT, 0, wp_vbo)
    gl.glDrawArrays(gl.GL_LINE_LOOP, 0, 100)
    gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
    wp_vbo.unbind()

    gl.glPopMatrix()


def draw_waypoints(state_refs_np, wp_vbo):
    if state_refs_np is not None:
        for i in range(0, state_refs_np.shape[1], 8):
            draw_waypoint(state_refs_np[0, i], state_refs_np[1, i], wp_vbo)
