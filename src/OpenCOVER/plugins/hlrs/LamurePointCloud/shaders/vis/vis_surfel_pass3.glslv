#version 420 core

// Vertex attributes.
layout(location = 0) in vec3 in_position;

// VS → FS.
out VsOut {
    vec2 uv;  // 0..1 screen UV
} vs_out;

void main()
{
    // Pass-through to clip space (expects NDC positions in in_position).
    gl_Position = vec4(in_position, 1.0);

    // Map NDC [-1,1] to UV [0,1].
    vs_out.uv = in_position.xy * 0.5 + 0.5;
}
