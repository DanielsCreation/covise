#version 420 core

uniform float max_radius;
uniform float min_radius;
uniform float scale_radius;
uniform mat4  model_view_matrix; 
uniform mat4  projection_matrix;
uniform mat3  normal_matrix;

layout(location = 0) in vec3 in_position;
layout(location = 5) in float in_radius;
layout(location = 6) in vec3 in_normal;

out VsOut {
    vec3 vs_center;
    vec3 vs_half_u;
    vec3 vs_half_v;
} vs_out;

void main() {
    // Radius-Skalierung mit Begrenzung
    float radius_ms = clamp(in_radius * scale_radius, min_radius, max_radius);

    // --- Stabile Basis-Vektor-Berechnung ---
    // Absichern gegen eine (0,0,0) Normale und normalisieren
    vec3 n = normalize((length(in_normal) > 0.0001) ? in_normal : vec3(0.0, 0.0, 1.0));
    
    // Wähle einen Referenz-Vektor, der garantiert nicht parallel zu n ist
    vec3 ref;
    if (abs(n.x) > abs(n.y) && abs(n.x) > abs(n.z)) {
        ref = vec3(0.0, 1.0, 0.0);
    } else if (abs(n.y) > abs(n.z)) {
        ref = vec3(0.0, 0.0, 1.0);
    } else {
        ref = vec3(1.0, 0.0, 0.0);
    }
    
    // Berechne die orthogonalen Basisvektoren im Model-Space
    vec3 ms_u = normalize(cross(ref, n));
    vec3 ms_v = normalize(cross(n, ms_u));
    // --- Ende der Basis-Berechnung ---

    // Transformation der Basis in den View-Space
    vec3 vs_center = (model_view_matrix * vec4(in_position, 1.0)).xyz;
    vec3 vs_half_u = (model_view_matrix * vec4(ms_u * radius_ms, 0.0)).xyz;
    vec3 vs_half_v = (model_view_matrix * vec4(ms_v * radius_ms, 0.0)).xyz;
    vec3 vs_normal = normalize(normal_matrix * n);

    // --- KORREKTUR ---
    //if (dot(cross(vs_half_u, vs_half_v), vs_normal) < 0.0) {
    //    vs_half_v = -vs_half_v;
    //}
    // --- ENDE KORREKTUR ---

    vs_out.vs_center = vs_center;
    vs_out.vs_half_u = vs_half_u;
    vs_out.vs_half_v = vs_half_v;

    gl_Position = projection_matrix * vec4(vs_center, 1.0);
}