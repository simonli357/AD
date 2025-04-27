#version 330 core
layout(location = 0) in vec3 aPos;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
uniform float time; // seconds elapsed

void main() {
    // build a simple 2×2 rotation matrix
    float angle = time; // 1 rad/sec; use time * speed for different rates
    float c = cos(angle);
    float s = sin(angle);
    mat2 rot = mat2(c, -s, s, c);

    // rotate the XY components of your vertex
    vec2 rotatedXY = rot * aPos.xy;
    vec3 rotatedPos = vec3(rotatedXY, aPos.z);

    // then do your usual MVP transform
    gl_Position = projection * view * model * vec4(rotatedPos, 1.0);
}
