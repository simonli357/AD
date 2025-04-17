#version 330 core

layout(location = 0) in vec3 aPos;

uniform mat4 uProj;
uniform mat4 uView;

out vec3 vWorldPos;

void main() {
    vWorldPos = aPos;
    gl_Position = uProj * uView * vec4(aPos, 1.0);
}
