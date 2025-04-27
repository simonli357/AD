#version 330 core

layout(location = 0) in vec2 aUV; // (u,v) ∈ [0,1]×[0,1]

uniform mat4 uProjection;
uniform vec4 uRect; // x0, y0, width, height
uniform float uShear; // shear amount on bottom edge
uniform float uFillFrac; // how far (0→1) to fill in x

void main() {
    // determine local (u) with fill frac
    float u = aUV.x * uFillFrac;
    float v = aUV.y;

    // compute shear offset at this v
    float dx = uRect.z * u + uRect.x + v * uShear - uRect.z;
    // dx = x0 - width + u*width + v*shear

    float dy = uRect.y - v * uRect.w;
    // dy = y0 - v*height

    gl_Position = uProjection * vec4(dx, dy, 0.0, 1.0);
}
