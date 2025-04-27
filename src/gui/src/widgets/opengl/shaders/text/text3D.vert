#version 330 core

layout(location = 0) in vec4 vertex; // vertex.xy = quad corners; vertex.zw = UVs
out vec2 TexCoords;

uniform sampler2D text;
uniform mat4 u_ViewProj;
uniform vec3 u_Center; // world-space baseline origin
uniform vec2 u_OffsetPx; // pixel offset to THIS glyph’s top-left
uniform vec2 u_SizePx; // glyph size in pixels (optional now)
uniform vec2 u_ScreenSize; // window size in pixels

void main() {
    // 1) Project the world origin:
    vec4 clipBase = u_ViewProj * vec4(u_Center, 1.0);

    // 2) Compute this corner’s pixel position:
    vec2 pixelPos = u_OffsetPx + vertex.xy;

    // 3) Convert pixel offset into clip-space (multiply by clipBase.w to retain perspective):
    vec2 clipOffset = (pixelPos / u_ScreenSize) * 2.0 * clipBase.w;

    // 4) Build the final clip-space position:
    gl_Position = vec4(clipBase.xy + clipOffset, clipBase.z, clipBase.w);

    // 5) Pass UV:
    TexCoords = vec2(vertex.z, 1.0 - vertex.w);
}
