#version 330 core
in vec2 TexCoord;
in vec3 VertexColor;
out vec4 FragColor;

uniform int hasTexture;
uniform sampler2D uTexture;
uniform vec4 u_color;

void main() {
    FragColor = u_color;
}
