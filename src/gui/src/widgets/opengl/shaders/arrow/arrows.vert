#version 330 core

// base-mesh position
layout(location = 0) in vec3 inPosition;

// per-instance color
layout(location = 1) in vec4 inColor;

// per-instance model matrix as 4 vec4’s
layout(location = 2) in vec4 instanceMatrix0;
layout(location = 3) in vec4 instanceMatrix1;
layout(location = 4) in vec4 instanceMatrix2;
layout(location = 5) in vec4 instanceMatrix3;

uniform mat4 view;
uniform mat4 projection;

out vec4 fragColor;

void main() {
    // reconstruct the model matrix
    mat4 model = mat4(
        instanceMatrix0,
        instanceMatrix1,
        instanceMatrix2,
        instanceMatrix3
    );

    // standard MVP
    gl_Position = projection * view * model * vec4(inPosition, 1.0);

    // pass color straight through
    fragColor = inColor;
}
