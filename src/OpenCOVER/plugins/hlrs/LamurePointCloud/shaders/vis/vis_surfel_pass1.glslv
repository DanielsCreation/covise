#version 420 core

uniform float max_radius;
uniform float min_radius;
uniform float scale_radius;
uniform mat4  model_view_matrix; 
uniform mat4  projection_matrix;  

layout(location = 0) in vec3 in_position;
layout(location = 5) in float in_radius;
layout(location = 6) in vec3 in_normal;

out VsOut {
    vec3 vs_center;
    vec3 vs_half_u;
    vec3 vs_half_v;
} vs_out;

// Orthonormale Basis aus Normalenvektor (Frisvad).
vec3 any_tangent(vec3 n) {
    float s = n.z >= 0.0 ? 1.0 : -1.0;
    float a = -1.0 / (s + n.z);
    float b = n.x * n.y * a;
    return normalize(vec3(1.0 + s * n.x * n.x * a, s * b, -s * n.x));
}
void build_basis(in vec3 n_in, out vec3 u, out vec3 v) {
    vec3 n = normalize(n_in);
    u = any_tangent(n);
    v = normalize(cross(n, u));
}

void main() {
    float radius_ms = clamp(in_radius * scale_radius, min_radius, max_radius);

    vec3 ms_u, ms_v;
    build_basis(in_normal, ms_u, ms_v);

    vec3 vs_center = (model_view_matrix * vec4(in_position, 1.0)).xyz;
    vec3 vs_half_u = (model_view_matrix * vec4(ms_u * radius_ms, 0.0)).xyz; // w=0: nur linearer Anteil
    vec3 vs_half_v = (model_view_matrix * vec4(ms_v * radius_ms, 0.0)).xyz;

    vs_out.vs_center = vs_center;
    vs_out.vs_half_u = vs_half_u;
    vs_out.vs_half_v = vs_half_v;

    gl_Position = projection_matrix * vec4(vs_center, 1.0);
}
