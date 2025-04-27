#version 330 core

in vec2 TexCoords;
out vec4 FragColor;

uniform sampler2D text;
uniform vec3 u_TextColor;

void main() {
    float alpha = texture(text, TexCoords).r;
    FragColor = vec4(u_TextColor, alpha);
}
