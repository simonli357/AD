#version 330 core
layout(location = 0) in vec2 aPos;
uniform mat4 projection;
uniform mat4 view;

void main() {
    gl_Position = projection * view * vec4(aPos, 3.0, 1.0);
}
