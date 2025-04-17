#version 330 core

in vec3 vWorldPos;

uniform vec3 uColor; // grid line color
uniform float cellSize; // in meters, e.g. 0.05

out vec4 FragColor;

void main() {
    // Compute how far we are from the nearest grid line along X and Y
    vec2 gridCoord = vWorldPos.xy / cellSize;
    vec2 f = fract(gridCoord) - 0.5;
    vec2 d = abs(f) * cellSize;

    // pick the closest axis
    float lineDist = min(d.x, d.y);

    // line thickness in world units (here 1 mm)
    float thickness = 0.002;

    // smoothly antialias the line
    float alpha = 1.0 - smoothstep(thickness - fwidth(lineDist), thickness + fwidth(lineDist), lineDist);

    if (alpha < 0.05)
        discard;
    FragColor = vec4(uColor, alpha);
}
