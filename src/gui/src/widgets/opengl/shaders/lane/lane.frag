#version 330 core

in vec2 vUV;
out vec4 fragColor;

uniform int divisions;       // Number of dash segments.
uniform vec4 color;          // Base color.
uniform float amplitude;     // Amplitude for sine modulation.
uniform float frequency;     // Frequency for sine modulation.
uniform float dashRatio;     // Fraction of each segment that is drawn.

void main() {
    // Use vUV.y (or vUV.x depending on desired orientation) for segmenting.
    float waveOffset = amplitude * sin(vUV.x * frequency * 6.2831);
    float adjustedV = vUV.y + waveOffset;
    float segPos = adjustedV * float(divisions);
    int segIndex = int(floor(segPos));
    float local = fract(segPos);
    
    // Determine if this segment should be drawn (alternating segments)
    // If local is within the dashRatio, draw; otherwise, make it gap.
    if (local <= dashRatio) {
        fragColor = color;
    } else {
        discard;
    }
}

