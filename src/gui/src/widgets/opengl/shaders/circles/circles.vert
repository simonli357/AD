#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in mat4 instanceMatrix;

uniform mat4 projection;
uniform mat4 view;

void main() {
    gl_Position = projection * view * instanceMatrix * vec4(aPos, 0.0, 1.0);
}
