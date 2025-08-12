#version 420 core
layout(location = 0) in vec3 in_position;  // Fullscreen-Quad (-1..1)
noperspective out vec2 tex_coords;
void main() {
    gl_Position = vec4(in_position, 1.0);
    tex_coords  = in_position.xy * 0.5 + 0.5;
}
