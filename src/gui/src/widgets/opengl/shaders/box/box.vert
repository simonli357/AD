#version 330 core

layout(location = 0) in vec2 aPos; // unit‐quad coords [0→1]

uniform mat4 uProjection;
uniform vec4 uRect; // x, y, width, height

void main() {
    // scale & translate unit quad into [x1,y1]→[x2,y2]
    vec2 p = aPos * uRect.zw + uRect.xy;
    gl_Position = uProjection * vec4(p, 0.0, 1.0);
}
