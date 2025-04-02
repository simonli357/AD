from OpenGL import GL as gl

import numpy as np


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


def draw_marker(marker_vbo, circle_count, cross_count, x, y):
    gl.glPushMatrix()
    gl.glTranslatef(x, y, 0)

    marker_vbo.bind()
    gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
    gl.glVertexPointer(3, gl.GL_FLOAT, 0, marker_vbo)

    # Draw circle
    gl.glLineWidth(2.0)
    gl.glColor3f(0.0, 1.0, 0.0)
    gl.glDrawArrays(gl.GL_LINE_LOOP, 0, circle_count)

    # Draw cross
    gl.glLineWidth(1.5)
    gl.glColor3f(0.0, 1.0, 0.0)
    gl.glDrawArrays(gl.GL_LINES, circle_count, cross_count)

    marker_vbo.unbind()

    gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
    gl.glPopMatrix()
