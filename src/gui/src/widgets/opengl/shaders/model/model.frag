#version 330 core

in vec2 TexCoord;
in vec3 VertexColor;
out vec4 FragColor;

uniform sampler2D uTexture;

// If you want to distinguish "no texture" you can pass a uniform or check if the texture is bound. 
// We'll assume the texture is bound if present and is white if not.

void main() {
    vec4 texColor = texture(uTexture, TexCoord);
    // Combine color with texture
    FragColor = vec4(VertexColor, 1.0) * texColor;
}
