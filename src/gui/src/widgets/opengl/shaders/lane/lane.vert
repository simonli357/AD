#version 330 core

layout(location = 0) in vec2 vertexPos;
uniform mat4 projection;
out vec2 vUV;

void main() {
    gl_Position = projection * vec4(vertexPos, 0.0, 1.0);
    vUV = vertexPos;
}
