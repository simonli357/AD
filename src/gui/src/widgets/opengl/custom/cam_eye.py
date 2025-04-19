from OpenGL import GL as gl
import numpy as np
import glm


class CameraEye:
    def __init__(self, shader_program, radius=0.005, sectors=16, stacks=16):
        self.shader_program = shader_program

        # uniforms
        self.loc_uProj = gl.glGetUniformLocation(shader_program, "uProj")
        self.loc_uView = gl.glGetUniformLocation(shader_program, "uView")
        self.loc_uModel = gl.glGetUniformLocation(shader_program, "uModel")
        self.loc_uColor = gl.glGetUniformLocation(shader_program, "uColor")

        # generate sphere mesh (unit radius)
        verts, indices = self._make_sphere(sectors, stacks)
        self.index_count = len(indices)

        # upload to GPU
        self.vao = gl.glGenVertexArrays(1)
        gl.glBindVertexArray(self.vao)

        # VBO
        self.vbo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, verts.nbytes, verts, gl.GL_STATIC_DRAW)

        # EBO
        self.ebo = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ELEMENT_ARRAY_BUFFER, self.ebo)
        gl.glBufferData(gl.GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices, gl.GL_STATIC_DRAW)

        # aPos @ location 0
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, 0, None)

        # unbind
        gl.glBindVertexArray(0)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
        gl.glBindBuffer(gl.GL_ELEMENT_ARRAY_BUFFER, 0)

        # sphere scale factor (turn unit sphere → 5 cm)
        self.scale_mat = glm.scale(glm.mat4(1.0), glm.vec3(radius))

    def _make_sphere(self, sectors, stacks):
        """Returns (verts, indices) for a UV sphere of radius=1."""
        verts = []
        for i in range(stacks + 1):
            phi = glm.pi() / 2 - i * glm.pi() / stacks     # from +π/2 to -π/2
            y = glm.sin(phi)
            r = glm.cos(phi)
            for j in range(sectors + 1):
                theta = j * 2 * glm.pi() / sectors         # 0 → 2π
                x = r * glm.cos(theta)
                z = r * glm.sin(theta)
                verts.extend((x, y, z))
        verts = np.array(verts, dtype=np.float32)

        indices = []
        for i in range(stacks):
            k1 = i * (sectors + 1)
            k2 = k1 + sectors + 1
            for j in range(sectors):
                # two triangles per quad
                indices += [k1 + j, k2 + j, k1 + j + 1]
                indices += [k1 + j + 1, k2 + j, k2 + j + 1]
        indices = np.array(indices, dtype=np.uint32)

        return verts, indices

    def draw(self, proj_mat, view_mat, extrinsic, color=(1.0, 0.0, 0.0)):
        gl.glUseProgram(self.shader_program)
        gl.glUniformMatrix4fv(self.loc_uProj, 1, gl.GL_FALSE, glm.value_ptr(proj_mat))
        gl.glUniformMatrix4fv(self.loc_uView, 1, gl.GL_FALSE, glm.value_ptr(view_mat))
        gl.glUniform3f(self.loc_uColor, *color)

        # --- compute intersection exactly like CameraRay ---
        origin = glm.vec3(extrinsic[3])
        fwd4 = extrinsic * glm.vec4(0, 0, -1, 0)
        dir = glm.normalize(glm.vec3(fwd4))
        if abs(dir.z) < 1e-6:
            gl.glUseProgram(0)
            return
        t = -origin.z / dir.z
        if t < 0:
            gl.glUseProgram(0)
            return
        intersect = origin + dir * t

        # build model matrix = translate(intersect) * scale(0.05)
        M = glm.translate(glm.mat4(1.0), intersect) * self.scale_mat
        gl.glUniformMatrix4fv(self.loc_uModel, 1, gl.GL_FALSE, glm.value_ptr(M))

        # draw sphere
        gl.glBindVertexArray(self.vao)
        gl.glBindBuffer(gl.GL_ELEMENT_ARRAY_BUFFER, self.ebo)
        gl.glDrawElements(gl.GL_TRIANGLES, self.index_count, gl.GL_UNSIGNED_INT, None)
        gl.glBindVertexArray(0)

        gl.glUseProgram(0)
