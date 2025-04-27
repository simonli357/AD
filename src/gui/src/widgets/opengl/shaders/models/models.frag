#version 330 core
in vec2 TexCoord;
in vec3 VertexColor;
out vec4 FragColor;

uniform int hasTexture;
uniform sampler2D uTexture;

void main() {
    vec4 baseColor = vec4(VertexColor, 1.0);
    if (hasTexture == 1) {
        baseColor *= texture(uTexture, TexCoord);
    }
    FragColor = baseColor;
}
