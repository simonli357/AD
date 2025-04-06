from OpenGL import GL as gl
from OpenGL.GL.shaders import compileProgram, compileShader
from .loaders import load_mesh, load_map
from .models import line_model, circle_model, crosshair_model, triangle_model
from .obj import load_obj

import os
import glob
import glm
import numpy as np


current_dir = os.path.dirname(os.path.abspath(__file__))
asset_dir = os.path.join(current_dir, 'assets')
shader_dir = os.path.join(current_dir, 'shaders')


def create_shader_module(filepath: str, module_type: int) -> int:
    source_code = ""
    with open(filepath, "r") as file:
        source_code = file.readlines()
    return compileShader(source_code, module_type)


def create_shader_program(vertex_filepath: str, fragment_filepath: str, geometry_filepath=None) -> int:
    vertex_module = create_shader_module(vertex_filepath, gl.GL_VERTEX_SHADER)
    fragment_module = create_shader_module(fragment_filepath, gl.GL_FRAGMENT_SHADER)
    modules = (vertex_module, fragment_module)
    if geometry_filepath is not None:
        modules + (geometry_filepath,)
    shader = compileProgram(*modules)
    gl.glDeleteShader(vertex_module)
    gl.glDeleteShader(fragment_module)
    return shader


def shader_path(dirname: str, filename: str):
    return os.path.join(shader_dir, dirname, filename)


def asset_path(filename: str):
    return os.path.join(asset_dir, filename)


def object_path(dirname: str):
    folder = os.path.join(asset_dir, dirname)
    # Find .obj files
    obj_candidates = glob.glob(os.path.join(folder, "*.obj"))
    obj = obj_candidates[0] if obj_candidates else None

    # Find .mtl files
    mtl_candidates = glob.glob(os.path.join(folder, "*.mtl"))
    mtl = mtl_candidates[0] if mtl_candidates else None

    if obj is None or mtl is None:
        print("Error loading model")
        exit(1)

    return mtl, obj


class ShaderRenderer:
    def __init__(self):
        self.load_models()
        self.load_shaders()

    def load_models(self):
        self.bfmc_track_model = load_map(asset_path('track.png'))
        self.barca_track_model = load_mesh(asset_path('track.obj'))
        self.line_model = line_model()
        self.circle_model = circle_model()
        self.crosshair_model = crosshair_model()
        self.triangle_model = triangle_model()

        self.car_model = load_obj(*object_path('car'))

        # self.prio_sign_model = load_obj(*object_path('priority_sign'))

    def load_shaders(self):
        self.barca_shader = create_shader_program(shader_path('barca', 'barca.vert'), shader_path('barca', 'barca.frag'))
        self.texture_shader = create_shader_program(shader_path('texture', 'texture.vert'), shader_path('texture', 'texture.frag'))
        self.line_shader = create_shader_program(shader_path('line', 'line.vert'), shader_path('line', 'line.frag'))
        self.circle_shader = create_shader_program(shader_path('circle', 'circle.vert'), shader_path('circle', 'circle.frag'))
        self.crosshair_shader = create_shader_program(shader_path('crosshair', 'crosshair.vert'), shader_path('crosshair', 'crosshair.frag'))
        self.triangle_shader = create_shader_program(shader_path('triangle', 'triangle.vert'), shader_path('triangle', 'triangle.frag'))

    ##################
    # Draw Functions
    ##################

    def draw_car(self, x, y, yaw, scale, color: (float, float, float, float), view_matrix, proj_matrix):
        shader_program = self.car_model.shader_program
        gl.glUseProgram(shader_program)

        model = glm.mat4(1.0)
        model = glm.translate(model, glm.vec3(x, y, 0.0))
        model = glm.rotate(model, yaw, glm.vec3(0.0, 0.0, 1.0))
        model = glm.scale(model, glm.vec3(scale, scale, scale))

        model_loc = gl.glGetUniformLocation(shader_program, "model")
        view_loc = gl.glGetUniformLocation(shader_program, "view")
        proj_loc = gl.glGetUniformLocation(shader_program, "projection")

        gl.glUniformMatrix4fv(model_loc, 1, gl.GL_FALSE, glm.value_ptr(model))
        gl.glUniformMatrix4fv(view_loc, 1, gl.GL_FALSE, glm.value_ptr(view_matrix))
        gl.glUniformMatrix4fv(proj_loc, 1, gl.GL_FALSE, glm.value_ptr(proj_matrix))

        if self.car_model.texture is not None:
            gl.glActiveTexture(gl.GL_TEXTURE0)
            gl.glBindTexture(gl.GL_TEXTURE_2D, self.car_model.texture)
            texture_location = gl.glGetUniformLocation(shader_program, "uTexture")
            gl.glUniform1i(texture_location, 0)

        gl.glBindVertexArray(self.car_model.mesh.vao)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.car_model.mesh.vertex_count)
        gl.glBindVertexArray(0)

    def draw_barca_track(self, x, y, z, yaw, scale, color: (float, float, float, float), view_matrix, proj_matrix):
        gl.glUseProgram(self.barca_shader)

        model = glm.mat4(1.0)
        model = glm.translate(model, glm.vec3(x, y, z))
        model = glm.rotate(model, yaw, glm.vec3(0.0, 0.0, 1.0))
        model = glm.scale(model, glm.vec3(scale[0], scale[1], 1.0))

        model_loc = gl.glGetUniformLocation(self.barca_shader, "model")
        view_loc = gl.glGetUniformLocation(self.barca_shader, "view")
        proj_loc = gl.glGetUniformLocation(self.barca_shader, "projection")
        color_loc = gl.glGetUniformLocation(self.barca_shader, "color")

        gl.glUniformMatrix4fv(model_loc, 1, gl.GL_FALSE, glm.value_ptr(model))
        gl.glUniformMatrix4fv(view_loc, 1, gl.GL_FALSE, glm.value_ptr(view_matrix))
        gl.glUniformMatrix4fv(proj_loc, 1, gl.GL_FALSE, glm.value_ptr(proj_matrix))
        gl.glUniform4f(color_loc, color[0], color[1], color[2], color[3])

        gl.glBindVertexArray(self.barca_track_model.vao)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.barca_track_model.vertex_count)
        gl.glBindVertexArray(0)

    def draw_texture(self, mat, x, y, z, scale, view_matrix, proj_matrix):
        if mat is None:
            return

        gl.glUseProgram(self.texture_shader)

        # Set matrices
        model = glm.mat4(1.0)
        model = glm.translate(model, glm.vec3(x, y, z))
        model = glm.scale(model, glm.vec3(scale[0], scale[1], 1.0))

        gl.glUniformMatrix4fv(
            gl.glGetUniformLocation(self.texture_shader, "model"),
            1, gl.GL_FALSE, glm.value_ptr(model)
        )
        gl.glUniformMatrix4fv(
            gl.glGetUniformLocation(self.texture_shader, "view"),
            1, gl.GL_FALSE, glm.value_ptr(view_matrix)
        )
        gl.glUniformMatrix4fv(
            gl.glGetUniformLocation(self.texture_shader, "projection"),
            1, gl.GL_FALSE, glm.value_ptr(proj_matrix)
        )

        # Bind texture
        gl.glActiveTexture(gl.GL_TEXTURE0)
        gl.glBindTexture(gl.GL_TEXTURE_2D, mat.texture_id)
        gl.glUniform1i(gl.glGetUniformLocation(self.texture_shader, "texture1"), 0)

        # Draw
        gl.glBindVertexArray(mat.vao)
        gl.glDrawElements(gl.GL_TRIANGLES, 6, gl.GL_UNSIGNED_INT, None)
        gl.glBindVertexArray(0)

    def draw_line(self, start, end, color, view_matrix, proj_matrix):
        gl.glUseProgram(self.line_shader)

        # Create proper vertex data with x,y coordinates
        vertices = np.array([
            [start[0], start[1]],
            [end[0], end[1]]
        ], dtype=np.float32).flatten()

        # Update VBO data
        self.line_model.vbo.bind()
        gl.glBufferData(
            gl.GL_ARRAY_BUFFER,
            vertices.nbytes,
            vertices,
            gl.GL_DYNAMIC_DRAW
        )

        # Set matrices
        gl.glUniformMatrix4fv(
            gl.glGetUniformLocation(self.line_shader, "projection"),
            1, gl.GL_FALSE, glm.value_ptr(proj_matrix)
        )
        gl.glUniformMatrix4fv(
            gl.glGetUniformLocation(self.line_shader, "view"),
            1, gl.GL_FALSE, glm.value_ptr(view_matrix)
        )
        # Set color
        gl.glUniform4fv(
            gl.glGetUniformLocation(self.line_shader, "color"),
            1, color
        )

        # Draw
        gl.glBindVertexArray(self.line_model.vao)
        gl.glDrawArrays(gl.GL_LINES, 0, 2)
        gl.glBindVertexArray(0)
        self.line_model.vbo.unbind()

    def draw_triangle(self, x, y, z, rot, scale, color, view_matrix, proj_matrix):
        gl.glUseProgram(self.triangle_shader)

        model = glm.mat4(1.0)
        model = glm.translate(model, glm.vec3(x, y, z))
        model = glm.rotate(model, rot, glm.vec3(0.0, 0.0, 1.0))
        model = glm.scale(model, glm.vec3(scale[0], scale[1], 1.0))

        model_loc = gl.glGetUniformLocation(self.triangle_shader, "model")
        view_loc = gl.glGetUniformLocation(self.triangle_shader, "view")
        proj_loc = gl.glGetUniformLocation(self.triangle_shader, "projection")
        color_loc = gl.glGetUniformLocation(self.triangle_shader, "color")

        gl.glUniformMatrix4fv(model_loc, 1, gl.GL_FALSE, glm.value_ptr(model))
        gl.glUniformMatrix4fv(view_loc, 1, gl.GL_FALSE, glm.value_ptr(view_matrix))
        gl.glUniformMatrix4fv(proj_loc, 1, gl.GL_FALSE, glm.value_ptr(proj_matrix))
        gl.glUniform4f(color_loc, color[0], color[1], color[2], color[3])

        gl.glBindVertexArray(self.triangle_model.vao)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.triangle_model.vertex_count)
        gl.glBindVertexArray(0)

    def draw_circle(self, center, radius, color, view_matrix, proj_matrix):
        gl.glUseProgram(self.circle_shader)

        # Set uniforms
        gl.glUniform2f(gl.glGetUniformLocation(self.circle_shader, "center"), center[0], center[1])
        gl.glUniform1f(gl.glGetUniformLocation(self.circle_shader, "radius"), radius)
        gl.glUniform4fv(gl.glGetUniformLocation(self.circle_shader, "color"), 1, color)
        gl.glUniformMatrix4fv(
            gl.glGetUniformLocation(self.circle_shader, "projection"),
            1, gl.GL_FALSE, glm.value_ptr(proj_matrix)
        )
        gl.glUniformMatrix4fv(
            gl.glGetUniformLocation(self.circle_shader, "view"),
            1, gl.GL_FALSE, glm.value_ptr(view_matrix)
        )

        # Draw
        gl.glBindVertexArray(self.circle_model.vao)
        gl.glDrawArrays(gl.GL_TRIANGLE_FAN, 0, self.circle_model.vertex_count)
        gl.glBindVertexArray(0)

    def draw_marker(self, x, y, color, scale, line_width, view_matrix, proj_matrix):
        gl.glUseProgram(self.crosshair_shader)

        # Create model matrix
        model = glm.mat4(1.0)
        model = glm.translate(model, glm.vec3(x, y, 0.0))
        model = glm.scale(model, glm.vec3(scale, scale, 1.0))

        # Set uniforms
        gl.glUniformMatrix4fv(
            gl.glGetUniformLocation(self.crosshair_shader, "projection"),
            1, gl.GL_FALSE, glm.value_ptr(proj_matrix)
        )
        gl.glUniformMatrix4fv(
            gl.glGetUniformLocation(self.crosshair_shader, "view"),
            1, gl.GL_FALSE, glm.value_ptr(view_matrix)
        )
        gl.glUniformMatrix4fv(
            gl.glGetUniformLocation(self.crosshair_shader, "model"),
            1, gl.GL_FALSE, glm.value_ptr(model)
        )
        gl.glUniform4fv(
            gl.glGetUniformLocation(self.crosshair_shader, "color"),
            1, color
        )

        # Draw circle
        gl.glBindVertexArray(self.crosshair_model.vao1)
        gl.glDrawArrays(gl.GL_LINE_LOOP, 0, 64)

        # Draw cross
        gl.glBindVertexArray(self.crosshair_model.vao2)
        gl.glDrawArrays(gl.GL_LINES, 0, 4)

        gl.glBindVertexArray(0)
