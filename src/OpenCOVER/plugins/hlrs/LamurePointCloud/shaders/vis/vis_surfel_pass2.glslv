#version 420 core

// Frisvad
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

// Uniforms
uniform float max_radius;
uniform float min_radius;
uniform float scale_radius;
uniform mat3  normal_matrix; // transpose(inverse(mat3(V*M)))

// Inputs
layout(location = 0) in vec3 in_position;
layout(location = 1) in float in_r;
layout(location = 2) in float in_g;
layout(location = 3) in float in_b;
layout(location = 5) in float in_radius;
layout(location = 6) in vec3  in_normal;

// To GS
out VS_OUT {
    vec3 ms_u;
    vec3 ms_v;
    vec3 center_ms;
    vec3 vs_normal;      // unit
    vec3 point_color;    // albedo rgb
    float radius_ws;     // unclamped Screen? -> Welt-Radius nach Skalierung
} vsOut;

void main() {
    float r_ws = clamp(in_radius * scale_radius, min_radius, max_radius);

    vec3 u_ms, v_ms; build_basis(in_normal, u_ms, v_ms);
    vsOut.ms_u        = u_ms * r_ws;
    vsOut.ms_v        = v_ms * r_ws;
    vsOut.center_ms   = in_position;
    vsOut.vs_normal   = normalize(normal_matrix * in_normal);
    vsOut.point_color = vec3(in_r, in_g, in_b);
    vsOut.radius_ws   = r_ws;

    gl_Position = vec4(in_position, 1.0);
}
