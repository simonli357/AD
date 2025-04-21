#version 330 core
layout(location = 0) out vec4 FragColor;

in vec2 vLocalPos;

uniform vec4 color; // interior fill color
const vec4 borderColor = vec4(1.0, 1.0, 0.0, 1.0);
const float halfSize = 0.5;
const float borderWidth = 0.05;

void main() {
    float d = abs(vLocalPos.x) + abs(vLocalPos.y);

    // outside the diamond — discard
    if (d > halfSize) {
        discard;
    }
    // in the border rim — solid yellow
    else if (d > halfSize - borderWidth) {
        FragColor = borderColor;
    }
    // interior — fade alpha from 0 at center to color.a at inner edge
    else {
        float maxD = halfSize - borderWidth;
        // normalized distance [0…1]
        float t = clamp(d / maxD, 0.0, 1.0);
        // alpha ramps from 0 (center) to color.a (edge)
        float a = mix(0.0, color.a, t);
        FragColor = vec4(color.rgb, a);
    }
}
