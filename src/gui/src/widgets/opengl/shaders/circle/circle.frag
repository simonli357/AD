#version 330 core
in vec2 fragCoord;
out vec4 FragColor;
uniform vec4 color;

void main() {
    float dist = length(fragCoord);
    if (dist > 1.0) discard;
    FragColor = color;

    float delta = fwidth(dist);
    FragColor.a *= 1.0 - smoothstep(1.0 - delta, 1.0, dist);
}
