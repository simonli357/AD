from OpenGL import GL as gl

import numpy as np
import math


def draw_track(texture, track_vbo, track_vertex_count=4):
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


def draw_destination(dest_vbo, x, y):
    vbo = dest_vbo[0]
    v_count = dest_vbo[1]
    gl.glPushMatrix()
    gl.glColor3f(0.0, 0.75, 0.75)
    gl.glTranslatef(x, y, 0)

    vbo.bind()
    gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
    gl.glVertexPointer(3, gl.GL_FLOAT, 0, vbo)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, v_count)
    vbo.unbind()

    gl.glDisableClientState(gl.GL_VERTEX_ARRAY)

    gl.glPopMatrix()


def draw_waypoint(wp_vbo, x, y, color: (int, int, int)):
    vbo = wp_vbo[0]
    v_count = wp_vbo[1]
    gl.glPushMatrix()
    gl.glColor3f(color[0] / 255.0, color[1] / 255.0, color[2] / 255.0)
    gl.glTranslatef(x, y, 0)

    vbo.bind()
    gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
    gl.glVertexPointer(3, gl.GL_FLOAT, 0, vbo)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, v_count)
    vbo.unbind()

    gl.glDisableClientState(gl.GL_VERTEX_ARRAY)

    gl.glPopMatrix()


def draw_path_node(wp_vbo, x, y):
    vbo = wp_vbo[0]
    v_count = wp_vbo[1]
    gl.glPushMatrix()
    gl.glColor3f(1.0, 1.0, 0.0)
    gl.glTranslatef(x, y, 0)

    vbo.bind()
    gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
    gl.glVertexPointer(3, gl.GL_FLOAT, 0, vbo)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, v_count)
    vbo.unbind()

    gl.glDisableClientState(gl.GL_VERTEX_ARRAY)

    gl.glPopMatrix()


def draw_car(x, y, yaw, car_model, color: (float, float, float, float)):
    gl.glPushMatrix()
    gl.glTranslatef(x, y, 0)
    gl.glRotatef(math.degrees(-yaw), 0, 0, 1)
    gl.glScalef(0.55, 0.55, 0.55)

    gl.glEnable(gl.GL_BLEND)
    gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
    gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)
    gl.glColor4f(*color)
    gl.glLineWidth(0.01)

    car_model.vbo.bind()
    gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
    gl.glVertexPointer(3, gl.GL_FLOAT, 0, car_model.vbo)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, car_model.vertex_count)
    gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
    car_model.vbo.unbind()

    gl.glDisable(gl.GL_BLEND)
    gl.glPopMatrix()


def draw_lane(x, y, widget_x, widget_y, orientation):
    """Draw lane markings using OpenGL lines"""
    # Normalize orientation within 0-2π
    orientation %= 2 * np.pi

    # Set line width
    gl.glLineWidth(2.0)

    # Determine lane direction and color
    if abs(orientation) < 0.1 or abs(orientation - 2 * np.pi) < 0.1:
        # Horizontal lane (east-west)
        color = (0.0, 1.0, 0.0)  # Green
        start = (0.0, y)
        end = (widget_x, y)
    elif abs(orientation - np.pi) < 0.1:
        # Horizontal lane (west-east)
        color = (1.0, 0.0, 0.0)  # Red
        start = (0.0, y)
        end = (widget_x, y)
    elif abs(orientation - np.pi / 2) < 0.1:
        # Vertical lane (north-south)
        color = (0.0, 0.0, 1.0)  # Blue
        start = (x, 0.0)
        end = (x, widget_y)
    elif abs(orientation - 3 * np.pi / 2) < 0.1:
        # Vertical lane (south-north)
        color = (1.0, 1.0, 0.0)  # Yellow
        start = (x, 0.0)
        end = (x, widget_y)
    else:
        return

    gl.glPushMatrix()
    gl.glColor3f(*color)

    # Draw the line
    gl.glBegin(gl.GL_LINES)
    gl.glVertex2f(*start)
    gl.glVertex2f(*end)
    gl.glEnd()

    gl.glPopMatrix()


def draw_sign(x, y, texture, sign_vbo, track_vertex_count=4):
    gl.glPushMatrix()

    gl.glEnable(gl.GL_TEXTURE_2D)
    gl.glBindTexture(gl.GL_TEXTURE_2D, texture)
    gl.glEnable(gl.GL_BLEND)
    gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
    gl.glTranslatef(x - 10, y, 0)
    gl.glScalef(0.1, 0.1, 0.1)

    sign_vbo.bind()

    gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
    gl.glEnableClientState(gl.GL_TEXTURE_COORD_ARRAY)
    # Stride = 20 bytes (5 floats * 4 bytes each)
    gl.glVertexPointer(3, gl.GL_FLOAT, 20, sign_vbo)
    # TexCoord offset = 12 bytes (3 floats into the vertex data)
    gl.glTexCoordPointer(2, gl.GL_FLOAT, 20, sign_vbo + 12)

    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_CLAMP_TO_BORDER)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_CLAMP_TO_BORDER)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_NEAREST)

    gl.glDrawArrays(gl.GL_QUADS, 0, track_vertex_count)

    sign_vbo.unbind()

    gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
    gl.glDisableClientState(gl.GL_TEXTURE_COORD_ARRAY)
    gl.glDisable(gl.GL_TEXTURE_2D)

    gl.glPopMatrix()
