from OpenGL import GL as gl
from concurrent.futures import ThreadPoolExecutor

from .utils import asset_path, object_path, create_shader_program, shader_path
from .loaders import load_mesh, load_map, load_2D_texture
from .basic import line_model, circle_model, crosshair_model, triangle_model, arrow_model, quad_model
from .obj import load_obj, create_obj
from ..enums import NamedColor, OpenGLContextName
from .custom.progress_bar import ProgressBar
from .custom.lane import LaneIndicator
from .custom.detection_box import DetectionBox
from .custom.speedometer import Speedometer
from .font import TextRenderer

import glm
import multiprocessing
import numpy as np
import time


class ShaderRenderer:
    def __init__(self, ctx_name):
        self._start_time = time.perf_counter()
        self.text_renderer = TextRenderer(16)
        self.large_text_renderer = TextRenderer(48)
        self.texture_shader = create_shader_program(shader_path('texture', 'texture.vert'), shader_path('texture', 'texture.frag'))
        self.texture2D_shader = create_shader_program(shader_path('texture', 'texture2D.vert'), shader_path('texture', 'texture2D.frag'))
        self.model_shader = create_shader_program(shader_path('model', 'model.vert'), shader_path('model', 'model.frag'))

        if ctx_name == OpenGLContextName.CAM:
            self.load_cam_models()
        elif ctx_name == OpenGLContextName.CAR:
            self.load_car_models()
        elif ctx_name == OpenGLContextName.MAP:
            self.load_map_models()
        elif ctx_name == OpenGLContextName.BARCA:
            self.load_barca_models()
        else:
            print("Invalid context name")
            exit(1)

    def load_cam_models(self):
        self.box_shader = create_shader_program(shader_path('box', 'box.vert'), shader_path('box', 'box.frag'))
        self.lane_shader = create_shader_program(shader_path('lane', 'lane.vert'), shader_path('lane', 'lane.frag'))

        self.detection_box_model = DetectionBox(self.text_renderer, self.box_shader)
        self.lane_model = LaneIndicator(self.text_renderer, self.lane_shader)

    def load_car_models(self):
        self.bfmc_track_model = load_map(asset_path('track.png'))

        model_load_tasks = [
            # (object_path args, attribute name)
            (('car', 'white'), 'white_car_model_data'),
            (('car', 'red'), 'red_car_model_data'),
            (('priority_sign', 'prio'), 'prio_sign_model_data'),
            (('oneway_sign', 'oneway'), 'oneway_sign_model_data'),
            (('stop_sign', 'stopsign'), 'stop_sign_model_data'),
            (('highway_entrance_sign', 'highwayentrance'), 'highway_entrance_sign_model_data'),
            (('highway_exit_sign', 'highwayexit'), 'highway_exit_sign_model_data'),
            (('roundabout_sign', 'roundabout'), 'roundabout_sign_model_data'),
            (('parking_sign', 'parking'), 'parking_sign_model_data'),
            (('crosswalk_sign', 'crosswalk'), 'crosswalk_sign_model_data'),
            (('noentry_sign', 'noentry'), 'noentry_sign_model_data'),
            (('traffic_light', 'lights'), 'traffic_light_model_data'),
            (('traffic_light', 'red'), 'red_light_model_data'),
            (('traffic_light', 'yellow'), 'yellow_light_model_data'),
            (('traffic_light', 'green'), 'green_light_model_data'),
            (('pedestrian', 'pedestrian'), 'pedestrian_model_data'),
            (('destination', 'destination'), 'destination_model_data'),
            (('destination', 'green'), 'green_destination_model_data'),
        ]

        # Parallel loading of all obj models
        with ThreadPoolExecutor(max_workers=multiprocessing.cpu_count()) as executor:
            futures = []
            for args, attr_name in model_load_tasks:
                futures.append((
                    attr_name,
                    executor.submit(
                        lambda a: load_obj(*object_path(*a)),
                        args
                    )
                ))

            # Collect results as they complete
            for attr_name, future in futures:
                try:
                    setattr(self, attr_name, future.result())
                except Exception as e:
                    print(f"Failed to load {attr_name}: {str(e)}")
                    raise

        self.white_car_model = create_obj(self.white_car_model_data, self.model_shader)
        self.red_car_model = create_obj(self.red_car_model_data, self.model_shader)

        self.prio_sign_model = create_obj(self.prio_sign_model_data, self.model_shader)
        self.oneway_sign_model = create_obj(self.oneway_sign_model_data, self.model_shader)
        self.stop_sign_model = create_obj(self.stop_sign_model_data, self.model_shader)
        self.highway_entrance_sign_model = create_obj(self.highway_entrance_sign_model_data, self.model_shader)
        self.highway_exit_sign_model = create_obj(self.highway_exit_sign_model_data, self.model_shader)
        self.roundabout_sign_model = create_obj(self.roundabout_sign_model_data, self.model_shader)
        self.parking_sign_model = create_obj(self.parking_sign_model_data, self.model_shader)
        self.crosswalk_sign_model = create_obj(self.crosswalk_sign_model_data, self.model_shader)
        self.noentry_sign_model = create_obj(self.noentry_sign_model_data, self.model_shader)
        self.traffic_light_model = create_obj(self.traffic_light_model_data, self.model_shader)
        self.red_light_model = create_obj(self.red_light_model_data, self.model_shader)
        self.yellow_light_model = create_obj(self.yellow_light_model_data, self.model_shader)
        self.green_light_model = create_obj(self.green_light_model_data, self.model_shader)
        self.pedestrian_model = create_obj(self.pedestrian_model_data, self.model_shader)
        self.destination_model = create_obj(self.destination_model_data, self.model_shader)
        self.green_destination_model = create_obj(self.green_destination_model_data, self.model_shader)

        self.triangle_shader = create_shader_program(shader_path('triangle', 'triangle.vert'), shader_path('triangle', 'triangle.frag'))
        self.triangle_model = triangle_model()

        self.cpu_texture = load_2D_texture(asset_path('cpu.png'))
        self.ram_texture = load_2D_texture(asset_path('ram.png'))
        self.stack_texture = load_2D_texture(asset_path('stack.png'))
        self.heap_texture = load_2D_texture(asset_path('heap.png'))
        self.thermometer_texture = load_2D_texture(asset_path('thermometer.png'))

        self.progress_bar_shader = create_shader_program(shader_path('progress_bar', 'progress_bar.vert'), shader_path('progress_bar', 'progress_bar.frag'))

        self.speedometer_gauge_shader = create_shader_program(shader_path('speedometer', 'speedometer.vert'), shader_path('speedometer', 'speedometer.frag'))
        self.speedometer_tick_shader = create_shader_program(shader_path('speedometer', 'tick.vert'), shader_path('speedometer', 'tick.frag'))
        self.speedometer_circle_shader = create_shader_program(shader_path('speedometer', 'circle.vert'), shader_path('speedometer', 'circle.frag'))
        self.speedometer_compass_shader = create_shader_program(shader_path('speedometer', 'compass.vert'), shader_path('speedometer', 'compass.frag'))

        self.progress_bar_model = ProgressBar(self.text_renderer, self.progress_bar_shader, self.texture2D_shader)
        self.speedometer_model = Speedometer(self.text_renderer, self.large_text_renderer, self.speedometer_gauge_shader, self.speedometer_tick_shader, self.speedometer_circle_shader, self.speedometer_compass_shader)

    def load_map_models(self):
        self.bfmc_track_model = load_map(asset_path('track.png'))

        model_load_tasks = [
            # (object_path args, attribute name)
            (('car', 'white'), 'white_car_model_data'),
            (('car', 'red'), 'red_car_model_data'),
            (('car', 'orange'), 'orange_car_model_data'),
        ]

        # Parallel loading of all obj models
        with ThreadPoolExecutor(max_workers=multiprocessing.cpu_count()) as executor:
            futures = []
            for args, attr_name in model_load_tasks:
                futures.append((
                    attr_name,
                    executor.submit(
                        lambda a: load_obj(*object_path(*a)),
                        args
                    )
                ))

            # Collect results as they complete
            for attr_name, future in futures:
                try:
                    setattr(self, attr_name, future.result())
                except Exception as e:
                    print(f"Failed to load {attr_name}: {str(e)}")
                    raise

        self.white_car_model = create_obj(self.white_car_model_data, self.model_shader)
        self.red_car_model = create_obj(self.red_car_model_data, self.model_shader)
        self.orange_car_model = create_obj(self.orange_car_model_data, self.model_shader)

        self.line_shader = create_shader_program(shader_path('line', 'line.vert'), shader_path('line', 'line.frag'))
        self.circle_shader = create_shader_program(shader_path('circle', 'circle.vert'), shader_path('circle', 'circle.frag'))
        self.crosshair_shader = create_shader_program(shader_path('crosshair', 'crosshair.vert'), shader_path('crosshair', 'crosshair.frag'))
        self.ripple_shader = create_shader_program(shader_path('ripple', 'ripple.vert'), shader_path('ripple', 'ripple.frag'))
        self.triangle_shader = create_shader_program(shader_path('triangle', 'triangle.vert'), shader_path('triangle', 'triangle.frag'))
        self.arrow_shader = create_shader_program(shader_path('arrow', 'arrow.vert'), shader_path('arrow', 'arrow.frag'))

        self.line_model = line_model()
        self.circle_model = circle_model()
        self.crosshair_model = crosshair_model()
        self.quad_model = quad_model()
        self.triangle_model = triangle_model()
        self.arrow_model = arrow_model()

    def load_barca_models(self):
        self.barca_shader = create_shader_program(shader_path('barca', 'barca.vert'), shader_path('barca', 'barca.frag'))
        self.barca_track_model = load_mesh(asset_path('track.obj'))

        model_load_tasks = [
            # (object_path args, attribute name)
            (('car', 'white'), 'white_car_model_data'),
        ]

        # Parallel loading of all obj models
        with ThreadPoolExecutor(max_workers=multiprocessing.cpu_count()) as executor:
            futures = []
            for args, attr_name in model_load_tasks:
                futures.append((
                    attr_name,
                    executor.submit(
                        lambda a: load_obj(*object_path(*a)),
                        args
                    )
                ))

            # Collect results as they complete
            for attr_name, future in futures:
                try:
                    setattr(self, attr_name, future.result())
                except Exception as e:
                    print(f"Failed to load {attr_name}: {str(e)}")
                    raise

        self.white_car_model = create_obj(self.white_car_model_data, self.model_shader)

    ##################
    # Draw Functions
    ##################

    def draw_car(self, x, y, yaw, color: NamedColor, scale, view_matrix, proj_matrix):
        car_model = None
        if color == NamedColor.WHITE:
            car_model = self.white_car_model
        elif color == NamedColor.RED:
            car_model = self.red_car_model
        elif color == NamedColor.ORANGE:
            car_model = self.orange_car_model
        else:
            return
        shader_program = car_model.shader_program
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

        if car_model.texture is not None:
            gl.glActiveTexture(gl.GL_TEXTURE0)
            gl.glBindTexture(gl.GL_TEXTURE_2D, car_model.texture)
            texture_location = gl.glGetUniformLocation(shader_program, "uTexture")
            gl.glUniform1i(texture_location, 0)

        gl.glBindVertexArray(car_model.mesh.vao)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, car_model.mesh.vertex_count)
        gl.glBindVertexArray(0)
        gl.glUseProgram(0)

    def draw_destination(self, x, y, yaw, scale, view_matrix, proj_matrix):
        obj_model = self.green_destination_model
        shader_program = obj_model.shader_program
        gl.glUseProgram(shader_program)

        model = glm.mat4(1.0)
        model = glm.translate(model, glm.vec3(x, y, -0.5))
        model = glm.rotate(model, yaw, glm.vec3(0.0, 0.0, 1.0))
        # model = glm.rotate(model, np.radians(90), glm.vec3(1.0, 0.0, 0.0))
        model = glm.scale(model, glm.vec3(scale, scale, scale))

        model_loc = gl.glGetUniformLocation(shader_program, "model")
        view_loc = gl.glGetUniformLocation(shader_program, "view")
        proj_loc = gl.glGetUniformLocation(shader_program, "projection")

        gl.glUniformMatrix4fv(model_loc, 1, gl.GL_FALSE, glm.value_ptr(model))
        gl.glUniformMatrix4fv(view_loc, 1, gl.GL_FALSE, glm.value_ptr(view_matrix))
        gl.glUniformMatrix4fv(proj_loc, 1, gl.GL_FALSE, glm.value_ptr(proj_matrix))

        if obj_model.texture is not None:
            has_texture_loc = gl.glGetUniformLocation(shader_program, "hasTexture")
            gl.glUniform1i(has_texture_loc, 1)  # Set to true
            gl.glActiveTexture(gl.GL_TEXTURE0)
            gl.glBindTexture(gl.GL_TEXTURE_2D, obj_model.texture)
            texture_location = gl.glGetUniformLocation(shader_program, "uTexture")
            gl.glUniform1i(texture_location, 0)

        gl.glBindVertexArray(obj_model.mesh.vao)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, obj_model.mesh.vertex_count)
        gl.glBindVertexArray(0)
        gl.glUseProgram(0)

    def draw_road_object(self, obj_type, x, y, yaw, scale, view_matrix, proj_matrix, is_animation=False):
        road_obj_model = None
        is_car = False
        z = 0
        if obj_type == 'Car':
            if not is_animation:
                return
            scale = scale / 300.0
            z = 0.1
            is_car = True
            road_obj_model = self.red_car_model
        elif obj_type == 'Oneway':
            road_obj_model = self.oneway_sign_model
        elif obj_type == 'Stopsign':
            road_obj_model = self.stop_sign_model
        elif obj_type == 'Highway Entrance':
            road_obj_model = self.highway_entrance_sign_model
        elif obj_type == 'Highway Exit':
            road_obj_model = self.highway_exit_sign_model
        elif obj_type == 'Roundabout':
            road_obj_model = self.roundabout_sign_model
        elif obj_type == 'Parking':
            road_obj_model = self.parking_sign_model
        elif obj_type == 'Crosswalk':
            road_obj_model = self.crosswalk_sign_model
        elif obj_type == 'No Entry':
            road_obj_model = self.noentry_sign_model
        elif obj_type == 'Priority':
            road_obj_model = self.prio_sign_model
        elif obj_type == 'Light':
            scale = scale / 1.5
            z = 0.05
            road_obj_model = self.traffic_light_model
        elif obj_type == 'Green Light':
            scale = scale / 1.5
            z = 0.05
            road_obj_model = self.green_light_model
        elif obj_type == 'Yellow Light':
            scale = scale / 1.5
            z = 0.05
            road_obj_model = self.yellow_light_model
        elif obj_type == 'Red Light':
            scale = scale / 1.5
            z = 0.05
            road_obj_model = self.red_light_model
        elif obj_type == 'Pedestrian':
            if is_animation:
                z = 0.1
                scale = scale / 1.5
            road_obj_model = self.pedestrian_model
        else:
            return
        shader_program = road_obj_model.shader_program
        gl.glUseProgram(shader_program)

        model = glm.mat4(1.0)
        model = glm.translate(model, glm.vec3(x, y, z))
        model = glm.rotate(model, yaw, glm.vec3(0.0, 0.0, 1.0))
        if not is_car:
            model = glm.rotate(model, np.radians(90), glm.vec3(1.0, 0.0, 0.0))
        model = glm.scale(model, glm.vec3(scale, scale, scale))

        model_loc = gl.glGetUniformLocation(shader_program, "model")
        view_loc = gl.glGetUniformLocation(shader_program, "view")
        proj_loc = gl.glGetUniformLocation(shader_program, "projection")

        gl.glUniformMatrix4fv(model_loc, 1, gl.GL_FALSE, glm.value_ptr(model))
        gl.glUniformMatrix4fv(view_loc, 1, gl.GL_FALSE, glm.value_ptr(view_matrix))
        gl.glUniformMatrix4fv(proj_loc, 1, gl.GL_FALSE, glm.value_ptr(proj_matrix))

        if road_obj_model.texture is not None:
            has_texture_loc = gl.glGetUniformLocation(shader_program, "hasTexture")
            gl.glUniform1i(has_texture_loc, 1)  # Set to true
            gl.glActiveTexture(gl.GL_TEXTURE0)
            gl.glBindTexture(gl.GL_TEXTURE_2D, road_obj_model.texture)
            texture_location = gl.glGetUniformLocation(shader_program, "uTexture")
            gl.glUniform1i(texture_location, 0)

        gl.glBindVertexArray(road_obj_model.mesh.vao)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, road_obj_model.mesh.vertex_count)
        gl.glBindVertexArray(0)
        gl.glUseProgram(0)

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
        gl.glUseProgram(0)

    def draw_texture(self, mat, x, y, z, scale, view_matrix, proj_matrix):
        if mat is None:
            return

        gl.glUseProgram(self.texture_shader)

        # Set matrices
        model = glm.mat4(1.0)
        model = glm.translate(model, glm.vec3(x, y, z))
        model = glm.scale(model, glm.vec3(scale[0], scale[1], 1.0))

        gl.glUniformMatrix4fv(gl.glGetUniformLocation(self.texture_shader, "model"), 1, gl.GL_FALSE, glm.value_ptr(model))
        gl.glUniformMatrix4fv(gl.glGetUniformLocation(self.texture_shader, "view"), 1, gl.GL_FALSE, glm.value_ptr(view_matrix))
        gl.glUniformMatrix4fv(gl.glGetUniformLocation(self.texture_shader, "projection"), 1, gl.GL_FALSE, glm.value_ptr(proj_matrix))

        # Bind texture
        gl.glActiveTexture(gl.GL_TEXTURE0)
        gl.glBindTexture(gl.GL_TEXTURE_2D, mat.texture_id)
        gl.glUniform1i(gl.glGetUniformLocation(self.texture_shader, "texture1"), 0)

        # Draw
        gl.glBindVertexArray(mat.vao)
        gl.glDrawElements(gl.GL_TRIANGLES, 6, gl.GL_UNSIGNED_INT, None)
        gl.glBindVertexArray(0)
        gl.glUseProgram(0)

    def draw_texture2D(self, x, y, icon, scale, proj_matrix):
        gl.glUseProgram(self.texture2D_shader)

        # Set matrices
        model = glm.mat4(1.0)
        model = glm.translate(model, glm.vec3(x, y, 0))
        model = glm.scale(model, glm.vec3(scale, scale, 1.0))
        model = glm.rotate(model, np.radians(180), glm.vec3(0, 0, 1))

        gl.glUniformMatrix4fv(gl.glGetUniformLocation(self.texture2D_shader, "model"), 1, gl.GL_FALSE, glm.value_ptr(model))
        gl.glUniformMatrix4fv(gl.glGetUniformLocation(self.texture2D_shader, "projection"), 1, gl.GL_FALSE, glm.value_ptr(proj_matrix))

        # Bind texture
        gl.glActiveTexture(gl.GL_TEXTURE0)
        gl.glBindTexture(gl.GL_TEXTURE_2D, icon.texture_id)
        gl.glUniform1i(gl.glGetUniformLocation(self.texture2D_shader, "texture1"), 0)

        # Draw
        gl.glBindVertexArray(icon.vao)
        gl.glDrawElements(gl.GL_TRIANGLES, 6, gl.GL_UNSIGNED_INT, None)
        gl.glBindVertexArray(0)
        gl.glUseProgram(0)

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
        gl.glUniformMatrix4fv(gl.glGetUniformLocation(self.line_shader, "projection"), 1, gl.GL_FALSE, glm.value_ptr(proj_matrix))
        gl.glUniformMatrix4fv(gl.glGetUniformLocation(self.line_shader, "view"), 1, gl.GL_FALSE, glm.value_ptr(view_matrix))
        # Set color
        gl.glUniform4fv(gl.glGetUniformLocation(self.line_shader, "color"), 1, color)

        # Draw
        gl.glBindVertexArray(self.line_model.vao)
        gl.glDrawArrays(gl.GL_LINES, 0, 2)
        gl.glBindVertexArray(0)
        self.line_model.vbo.unbind()
        gl.glUseProgram(0)

    def draw_triangle(self, x, y, z, rot, scale, color, view_matrix, proj_matrix, rot_barca=0):
        gl.glUseProgram(self.triangle_shader)

        model = glm.mat4(1.0)
        model = glm.rotate(model, glm.radians(rot_barca), glm.vec3(0, 0, 1))
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
        gl.glUseProgram(0)

    def draw_circle(self, center, radius, color, view_matrix, proj_matrix):
        gl.glUseProgram(self.circle_shader)

        # Set uniforms
        gl.glUniform2f(gl.glGetUniformLocation(self.circle_shader, "center"), center[0], center[1])
        gl.glUniform1f(gl.glGetUniformLocation(self.circle_shader, "radius"), radius)
        gl.glUniform4fv(gl.glGetUniformLocation(self.circle_shader, "color"), 1, color)
        gl.glUniformMatrix4fv(gl.glGetUniformLocation(self.circle_shader, "projection"), 1, gl.GL_FALSE, glm.value_ptr(proj_matrix))
        gl.glUniformMatrix4fv(gl.glGetUniformLocation(self.circle_shader, "view"), 1, gl.GL_FALSE, glm.value_ptr(view_matrix))

        # Draw
        gl.glBindVertexArray(self.circle_model.vao)
        gl.glDrawArrays(gl.GL_TRIANGLE_FAN, 0, self.circle_model.vertex_count)
        gl.glBindVertexArray(0)
        gl.glUseProgram(0)

    def draw_marker(self, x, y, color, scale, view_matrix, proj_matrix):
        # Create model matrix
        model = glm.mat4(1.0)
        model = glm.translate(model, glm.vec3(x, y, 0.0))
        model = glm.scale(model, glm.vec3(scale, scale, 1.0))

        gl.glUseProgram(self.ripple_shader)

        # Set uniforms
        gl.glUniformMatrix4fv(gl.glGetUniformLocation(self.ripple_shader, "projection"), 1, gl.GL_FALSE, glm.value_ptr(proj_matrix))
        gl.glUniformMatrix4fv(gl.glGetUniformLocation(self.ripple_shader, "view"), 1, gl.GL_FALSE, glm.value_ptr(view_matrix))
        gl.glUniformMatrix4fv(gl.glGetUniformLocation(self.ripple_shader, "model"), 1, gl.GL_FALSE, glm.value_ptr(model))
        gl.glUniform1f(gl.glGetUniformLocation(self.ripple_shader, "time"), time.perf_counter() - self._start_time)

        gl.glBindVertexArray(self.quad_model.vao)
        gl.glDrawArrays(gl.GL_TRIANGLE_STRIP, 0, 4)
        gl.glBindVertexArray(0)
        gl.glUseProgram(0)

        gl.glUseProgram(self.crosshair_shader)

        # Set uniforms
        gl.glUniformMatrix4fv(gl.glGetUniformLocation(self.crosshair_shader, "projection"), 1, gl.GL_FALSE, glm.value_ptr(proj_matrix))
        gl.glUniformMatrix4fv(gl.glGetUniformLocation(self.crosshair_shader, "view"), 1, gl.GL_FALSE, glm.value_ptr(view_matrix))
        gl.glUniformMatrix4fv(gl.glGetUniformLocation(self.crosshair_shader, "model"), 1, gl.GL_FALSE, glm.value_ptr(model))
        gl.glUniform4fv(gl.glGetUniformLocation(self.crosshair_shader, "color"), 1, color)
        gl.glUniform1f(gl.glGetUniformLocation(self.crosshair_shader, "time"), time.perf_counter() - self._start_time)

        # Draw cross
        gl.glBindVertexArray(self.crosshair_model.vao)
        for i in range(4):
            gl.glDrawArrays(gl.GL_TRIANGLE_STRIP, i * 4, 4)
        gl.glBindVertexArray(0)
        gl.glUseProgram(0)

    def draw_axis2D(self, x, y, yaw, scale, view_matrix, proj_matrix):
        gl.glUseProgram(self.arrow_shader)

        model_loc = gl.glGetUniformLocation(self.arrow_shader, "model")
        view_loc = gl.glGetUniformLocation(self.arrow_shader, "view")
        proj_loc = gl.glGetUniformLocation(self.arrow_shader, "projection")
        color_loc = gl.glGetUniformLocation(self.arrow_shader, "color")

        # Draw green arrow (pointing along y)
        model = glm.mat4(1.0)
        model = glm.translate(model, glm.vec3(x, y, 3.0))
        model = glm.rotate(model, yaw, glm.vec3(0, 0, 1))
        model = glm.scale(model, glm.vec3(scale, scale, 1.0))
        gl.glUniformMatrix4fv(model_loc, 1, gl.GL_FALSE, glm.value_ptr(model))
        gl.glUniformMatrix4fv(view_loc, 1, gl.GL_FALSE, glm.value_ptr(view_matrix))
        gl.glUniformMatrix4fv(proj_loc, 1, gl.GL_FALSE, glm.value_ptr(proj_matrix))
        gl.glUniform4f(color_loc, *NamedColor.GREEN.value)

        gl.glBindVertexArray(self.arrow_model.vao)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.arrow_model.vertex_count)
        gl.glBindVertexArray(0)

        # Draw red arrow (pointing along x) using the same global position and yaw, with extra -90° rotation
        model = glm.mat4(1.0)
        # Global transformation: translate and apply global yaw
        model = glm.translate(model, glm.vec3(x, y, 3.0))
        model = glm.rotate(model, yaw, glm.vec3(0, 0, 1))
        # Local adjustment: rotate extra -90° so arrow points to x direction
        model = glm.rotate(model, glm.radians(-90), glm.vec3(0, 0, 1))
        # Apply scale last
        model = glm.scale(model, glm.vec3(scale, scale, 1.0))
        gl.glUniformMatrix4fv(model_loc, 1, gl.GL_FALSE, glm.value_ptr(model))
        gl.glUniform4f(color_loc, *NamedColor.RED.value)

        gl.glBindVertexArray(self.arrow_model.vao)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.arrow_model.vertex_count)
        gl.glBindVertexArray(0)
        gl.glUseProgram(0)
