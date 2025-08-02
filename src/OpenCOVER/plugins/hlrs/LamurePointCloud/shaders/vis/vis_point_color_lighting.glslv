// Vertex Shader for Point-based Rendering with Lighting
#version 420 core

// ATTRIBUTES
layout(location = 0) in vec3  in_position;
layout(location = 1) in float in_r;
layout(location = 2) in float in_g;
layout(location = 3) in float in_b;
layout(location = 4) in float empty;
layout(location = 5) in float in_radius;
layout(location = 6) in vec3  in_normal;

// UNIFORMS
// Matrix-Uniforms from Surfel-Shader for correct lighting calculations
uniform mat4  mvp_matrix;
uniform mat4  view_matrix;
uniform mat3  view_normal_matrix;
uniform float max_radius;
uniform float min_radius;
uniform float scale_radius;
uniform float scale_projection;

// OUTPUT to Fragment Shader
out VertexData {
    vec3  pass_point_color;
    vec3  pass_world_pos;
    vec3  pass_vs_pos;      // Position in View-Space
    vec3  pass_vs_normal;   // Normal in View-Space
    float pass_radius_ws;
    float pass_screen_size;
} VertexOut;

void main() {
    // --- World-Space Calculations ---
    vec3 worldPos = in_position;
    vec3 normalWS = normalize(in_normal);

    // --- View-Space Calculations (from Surfel-Shader) ---
    // Transform position and normal to view space for lighting
    vec4 vsPos4 = view_matrix * vec4(worldPos, 1.0);
    VertexOut.pass_vs_pos = vsPos4.xyz / vsPos4.w; // Perspective divide
    VertexOut.pass_vs_normal = normalize(view_normal_matrix * normalWS);

    // --- Point-Size Calculation (from Point-Shader) ---
    vec4 clipPos = mvp_matrix * vec4(worldPos, 1.0);
    gl_Position = clipPos;

    float r_world = clamp(in_radius * scale_radius, min_radius, max_radius);
    gl_PointSize = r_world * scale_projection / abs(clipPos.w);

    // --- Pass-through data ---
    VertexOut.pass_point_color = vec3(in_r, in_g, in_b);
    VertexOut.pass_world_pos   = worldPos;
    VertexOut.pass_radius_ws   = r_world;
    VertexOut.pass_screen_size = gl_PointSize;
}