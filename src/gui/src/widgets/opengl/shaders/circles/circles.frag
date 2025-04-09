#version 330 core
uniform vec4 color;

void main()
{
    FragColor = vec4(color, 1.0);
}
