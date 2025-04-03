from OpenGL import GL as gl
from ...enums import MapData

import numpy as np


def draw_destination(dest_vbo, x, y):
    vbo = dest_vbo[0]
    v_count = dest_vbo[1]
    gl.glPushMatrix()
    gl.glPushAttrib(gl.GL_CURRENT_BIT)

    gl.glColor3f(0.0, 0.75, 0.75)
    gl.glTranslatef(x, y, 0)

    vbo.bind()
    gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
    gl.glVertexPointer(3, gl.GL_FLOAT, 0, vbo)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, v_count)
    vbo.unbind()

    gl.glDisableClientState(gl.GL_VERTEX_ARRAY)

    gl.glPopAttrib()
    gl.glPopMatrix()


def draw_path_node(waypoints, widget_width, widget_height):
    gl.glPushMatrix()
    gl.glPushAttrib(gl.GL_CURRENT_BIT)

    gl.glColor3f(0.0, 1.0, 0.0)
    gl.glLineWidth(6.0)

    # Convert waypoints to screen coordinates
    screen_points = []
    for i in range(0, len(waypoints), 2):
        if i + 1 >= len(waypoints):
            break
        x = (waypoints[i] / MapData.REAL_WORLD_WIDTH.value) * widget_width
        y = (waypoints[i + 1] / MapData.REAL_WORLD_HEIGHT.value) * widget_height
        screen_points.append((x, y))

    # Draw connected lines
    if len(screen_points) > 1:
        gl.glBegin(gl.GL_LINE_STRIP)
        for x, y in screen_points:
            gl.glVertex2f(x, y)
        gl.glEnd()

    gl.glLineWidth(1.0)
    gl.glPopAttrib()
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
    gl.glPushAttrib(gl.GL_CURRENT_BIT)

    gl.glColor3f(*color)

    # Draw the line
    gl.glBegin(gl.GL_LINES)
    gl.glVertex2f(*start)
    gl.glVertex2f(*end)
    gl.glEnd()

    gl.glLineWidth(1.0)
    gl.glPopAttrib()
    gl.glPopMatrix()


def draw_marker(marker_vbo, circle_count, cross_count, x, y):
    gl.glPushMatrix()
    gl.glPushAttrib(gl.GL_CURRENT_BIT)

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

    gl.glLineWidth(1.0)
    gl.glPopAttrib()
    gl.glPopMatrix()
