// vis_point_prov.glslv (Vertex Shader)
#version 420 core

// --- Inputs ---
layout(location = 0) in vec3  in_position;
layout(location = 1) in float in_r;
layout(location = 2) in float in_g;
layout(location = 3) in float in_b;
layout(location = 4) in float empty;
layout(location = 5) in float in_radius;
layout(location = 6) in vec3  in_normal;

// Provenance-Inputs
layout(location = 7)  in float in_prov1;
layout(location = 8)  in float in_prov2;
layout(location = 9)  in float in_prov3;
layout(location = 10) in float in_prov4;
layout(location = 11) in float in_prov5;
layout(location = 12) in float in_prov6;

// --- Uniforms (gleiche Skalierung wie Original) ---
uniform mat4  mvp_matrix;
uniform float max_radius;
uniform float min_radius;
uniform float scale_radius;
uniform float scale_projection;

// --- Outputs an Fragment Shader ---
out VertexData {
    vec3  pass_point_color;  // entspricht in_r/g/b
    vec3  pass_world_pos;   // Position in Welt
    vec3  pass_normal_ws;   // Normale im World-Space
    float pass_radius_ws;   // Radius nach clamp
    float pass_screen_size;  // finale PointSize
    float pass_prov1;       // Provenance-Kanäle
    float pass_prov2;
    float pass_prov3;
    float pass_prov4;
    float pass_prov5;
    float pass_prov6;
} VertexOut;

void main() {
    vec4 clipPos = mvp_matrix * vec4(in_position, 1.0);
    gl_Position = clipPos;

    float r = clamp(in_radius * scale_radius, min_radius, max_radius);
    gl_PointSize = r * scale_projection / abs(clipPos.w);

    VertexOut.pass_point_color = vec3(in_r, in_g, in_b);
    VertexOut.pass_world_pos  = in_position;
    VertexOut.pass_normal_ws  = normalize(in_normal);
    VertexOut.pass_radius_ws  = r;
    VertexOut.pass_screen_size = gl_PointSize;
    VertexOut.pass_prov1 = in_prov1;
    VertexOut.pass_prov2 = in_prov2;
    VertexOut.pass_prov3 = in_prov3;
    VertexOut.pass_prov4 = in_prov4;
    VertexOut.pass_prov5 = in_prov5;
    VertexOut.pass_prov6 = in_prov6;
}
