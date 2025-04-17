#version 330 core

in vec3 vWorldPos;

uniform vec3 uColor;
uniform float cellSize;

out vec4 FragColor;

void main() {
    vec2 g = vWorldPos.xy / cellSize; // how many cells across?
    vec2 f = fract(g) - 0.5; // distance to nearest line center in [–0.5, +0.5]
    vec2 d = abs(f); // always positive

    float distInCells = min(d.x, d.y);

    float thicknessInCells = 0.001 / cellSize;

    float w = max(fwidth(g.x), fwidth(g.y));

    float alpha = 1.0 - smoothstep(thicknessInCells - w, thicknessInCells + w, distInCells);

    if (alpha < 0.05)
        discard;

    FragColor = vec4(uColor, alpha);
}
