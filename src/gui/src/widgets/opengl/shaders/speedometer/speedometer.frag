#version 330 core
in vec2 fragPos;
out vec4 FragColor;

uniform vec2 uCenter; // Gauge center (in pixels)
uniform float uInnerRadius; // Inner radius of the gauge (in pixels)
uniform float uOuterRadius; // Outer radius of the gauge (in pixels)
uniform float uProgress; // Normalized progress [0,1] (0: no fill, 1: full 280° fill)
uniform vec4 uFillColor; // Color for filled portion
uniform vec4 uBgColor; // Background color (unfilled gauge)

const float maxSweep = 290.0; // Total sweep in degrees when uProgress is 1.
const float smoothing = 1.0; // Smoothing (in degrees) for the fill edge.
const float startAngle = 0.0; // In our custom system, fill starts at 180° (i.e. bottom).

void main()
{
    // Compute the vector from the gauge center to this fragment.
    vec2 pos = fragPos - uCenter;
    float r = length(pos);

    // Only process fragments inside the ring.
    if (r < uInnerRadius || r > uOuterRadius)
        discard;

    // Compute the standard polar angle in degrees (0° along +X, CCW).
    float standardAngle = degrees(atan(pos.y, pos.x));
    if (standardAngle < 0.0)
        standardAngle += 360.0;

    // Remap so that the bottom (which is standard 270°) becomes 180°.
    // Simply subtract 90°; then (0,-1): 270 - 90 = 180.
    float customAngle = mod(standardAngle - 90.0, 360.0);

    // Compute the relative angle from our start (180°).
    float relativeAngle = (customAngle >= startAngle)
        ? (customAngle - startAngle) : (customAngle + (360.0 - startAngle));

    // Determine how much of the arc should be filled.
    float threshold = uProgress * maxSweep;

    // Apply smoothing at the fill edge.
    float fillFactor = 0.0;
    if (relativeAngle <= threshold)
        fillFactor = 1.0;
    else if (relativeAngle < threshold + smoothing)
        fillFactor = 1.0 - smoothstep(threshold, threshold + smoothing, relativeAngle);
    else
        fillFactor = 0.0;

    // Mix the background and fill colors.
    vec4 color = mix(uBgColor, uFillColor, fillFactor);
    FragColor = color;
}
