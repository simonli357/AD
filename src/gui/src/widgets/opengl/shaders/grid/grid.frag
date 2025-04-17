#version 330 core

in vec3 vWorldPos;

uniform vec3 uColor;
uniform float cellSize; // meters per cell, e.g. 0.05

out vec4 FragColor;

// how far out the grid still shows (in world meters)
const float MAX_DIST = 2.0;

void main() {
    // ————— compute cell coords & signed offset from line center —————
    vec2 g = vWorldPos.xy / cellSize;
    vec2 fc = fract(g) - 0.5; // in [–0.5,+0.5]
    vec2 af = abs(fc);

    // ————— separate 1px antialiasing for vertical (X) and horizontal (Y) lines —————
    float wx = fwidth(g.x); // how many cells = 1 px in X
    float wy = fwidth(g.y); // how many cells = 1 px in Y

    // vertical lines alpha (based on X‑offset)
    float ax = 1.0 - smoothstep(0.5 * wx, wx, af.x);
    // horizontal lines alpha (based on Y‑offset)
    float ay = 1.0 - smoothstep(0.5 * wy, wy, af.y);
    // combine: show whichever line is closer
    float lineA = max(ax, ay);
    if (lineA < 0.01)
        discard;

    // ————— planar fade —————
    float dist = length(vWorldPos.xy);
    float fade = clamp(1.0 - (dist / MAX_DIST), 0.0, 1.0);
    float alpha = lineA * fade;
    if (alpha < 0.01)
        discard;

    FragColor = vec4(uColor, alpha);
}
