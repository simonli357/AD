#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aUV;
uniform mat4 projection, view, model;
out vec2 TexCoords;

void main() {
    TexCoords = aUV;
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}
