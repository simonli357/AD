#version 330 core

layout(location = 0) in vec4 vertex;   
out vec2 TexCoords;

uniform mat4 projection;
uniform vec2 textOffset;

void main() {
    vec2 pos = vertex.xy + textOffset;
    gl_Position = projection * vec4(pos, 0.0, 1.0);
    TexCoords = vertex.zw;
}

