#version 330 core
layout(location = 0) in vec2 aPos;

out vec2 vLocalPos;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

void main() {
    vLocalPos = aPos;
    gl_Position = projection * view * model * vec4(aPos, 0.0, 1.0);
}
