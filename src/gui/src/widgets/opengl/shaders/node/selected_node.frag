#version 330 core
layout(location = 0) out vec4 FragColor;

// this must match the out from your vertex shader:
in vec2 vLocalPos;

uniform vec4 color; // interior fill color
const vec4 borderColor = vec4(1.0, 1.0, 0.0, 1.0); // pure yellow
const float halfSize = 0.5; // diamond extends where |x|+|y| <= 0.5
const float borderWidth = 0.05; // thickness of border region

void main() {
    float d = abs(vLocalPos.x) + abs(vLocalPos.y);

    // outside the diamond ‑ discard
    if (d > halfSize) {
        discard;
    }
    // inside the “rim” ‑ yellow border
    else if (d > halfSize - borderWidth) {
        FragColor = borderColor;
    }
    // interior
    else {
        FragColor = color;
    }
}
