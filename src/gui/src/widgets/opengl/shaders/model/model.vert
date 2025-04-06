#version 330 core
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec2 inTexCoord;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
out vec2 TexCoord;

void main() {
    gl_Position = projection * view * model * vec4(inPosition, 1.0);
    TexCoord = inTexCoord;
}
