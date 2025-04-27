#version 330 core

layout(location = 0) in vec3 aBasePos; // your diamond geometry

layout(location = 1) in vec2 iPos; // world → NDC offset
layout(location = 2) in float iAngle; // rotation
layout(location = 3) in float iScale; // uniform scale
layout(location = 4) in vec4 iColor; // RGBA

uniform mat4 projection;
uniform mat4 view;

out vec4 vColor;

void main() {
    // build 2×2 rotation matrix
    float c = cos(iAngle);
    float s = sin(iAngle);
    mat2 R = mat2(c, -s, s, c) * iScale;

    // apply to base vertex
    vec2 pos2d = R * aBasePos.xy + iPos;
    vec4 worldPos = vec4(pos2d, aBasePos.z, 1.0);

    gl_Position = projection * view * worldPos;
    vColor = iColor;
}
