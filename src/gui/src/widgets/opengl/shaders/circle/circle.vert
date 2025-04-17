#version 330 core
layout(location = 0) in vec2 aPos;
uniform mat4 projection;
uniform mat4 view;
uniform vec2 center;
uniform float radius;

out vec2 fragCoord;

void main() {
    vec2 position = center + aPos * radius;
    gl_Position = projection * view * vec4(position, 0.2, 1.0);
    fragCoord = aPos;
}
