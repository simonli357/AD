#version 330 core
in vec2 TexCoords;
uniform float time; // seconds elapsed
out vec4 FragColor;

const float PI = 3.14159265359;

void main() {
    // normalized pixel‑radius
    vec2 p = TexCoords * 2.0 - 1.0;
    float d = length(p);

    // ── white ripple rings (unchanged) ───────────────────────────────────────
    float speed = 0.25; // rings/sec
    float spacing = 0.2; // distance between rings
    float ringW = 0.03; // ring thickness
    float borderW = 0.05; // outer border
    float limit = 0.7;

    float baseR = fract(time * speed);
    float m = fract((d - baseR) / spacing) * spacing;
    float rings = 1.0 - smoothstep(0.0, ringW, m);
    float ringCenter = d - m;
    rings *= step(ringCenter, limit);

    // ── persistent white contour at r=0.95 ────────────────────────────────────
    float border = smoothstep(0.95 - borderW, 0.95, d) - smoothstep(0.95, 0.95 + borderW, d);

    // ── rotating red quadrant arcs @ r=0.8 ──────────────────────────────────
    float arcR = 0.85; // radius of the red ticks
    float arcW = 0.1; // radial thickness
    float arcAngle = radians(60.0); // angular span (radians)
    float rotSpeed = PI * 0.5; // rad/sec (full rotation in 4 s)

    // compute pixel angle in [0,2π)
    float ang = atan(p.y, p.x);
    if (ang < 0.0) ang += 2.0 * PI;

    // shift by rotation
    float shifted = ang - time * rotSpeed;

    // wrap each quadrant to [0, π/2)
    float localAng = mod(shifted, PI / 2.0);

    // mask where localAng < arcAngle → 1 inside arc span
    float arcMask = step(localAng, arcAngle);

    // radial band at arcR
    float arcRadial = step(d, arcR) * (1.0 - step(d, arcR - arcW));

    float ticks = arcMask * arcRadial;

    // ── composite & output ───────────────────────────────────────────────────
    // draw red ticks on top if present
    if (ticks > 0.01) {
        FragColor = vec4(1.0, 1.0, 1.0, 1.0);
        return;
    }

    // otherwise fall back to white ripple + border
    float alpha = max(rings, border);
    if (alpha < 0.01) discard;
    FragColor = vec4(1.0, 1.0, 1.0, alpha);
}
