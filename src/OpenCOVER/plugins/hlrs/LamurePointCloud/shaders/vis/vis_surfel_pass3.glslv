#version 420 core

// Input is a simple quad (e.g., from a VBO)
layout(location = 0) in vec3 in_position;

// Pass-through texture coordinates to the fragment shader
out vec2 tex_coords;

void main()
{
    gl_Position   = vec4(in_position, 1.0);
    // [-1,1] → [0,1]
    tex_coords    = in_position.xy * 0.5 + 0.5;
}