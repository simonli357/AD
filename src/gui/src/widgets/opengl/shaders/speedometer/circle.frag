#version 330 core
in vec2 vLocalPos;
out vec4 FragColor;

uniform vec4 color;
uniform float radius;
uniform float lineWidth;

void main()
{
    // Compute the distance from the center.
    float d = length(vLocalPos);
    
    // Difference from the desired circle edge.
    float diff = abs(d - radius);
    
    // Use fwidth to get an adaptive smoothing factor.
    float smoothing = fwidth(diff);

    // Adjust the threshold around lineWidth using the derivative.
    // Here the 'lineWidth' plays a role as the half-thickness, but we add/subtract smoothing to avoid abrupt transitions.
    float alpha = 1.0 - smoothstep(lineWidth - smoothing, lineWidth + smoothing, diff);
    
    FragColor = vec4(color.rgb, color.a * alpha);
}
