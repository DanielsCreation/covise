#version 420 core

// Global parameters.
uniform float max_radius;
uniform float min_radius;
uniform float scale_radius;
uniform mat3  normal_matrix;
uniform mat4  model_matrix;
uniform mat4  model_view_matrix;
uniform mat4  projection_matrix;
uniform vec2  viewport;
uniform bool  show_normals;
uniform bool  show_accuracy;
uniform bool  show_radius_deviation;
uniform bool  show_output_sensitivity;
uniform float accuracy;
uniform float average_radius;

// Vertex attributes.
layout(location = 0) in vec3  in_position;
layout(location = 1) in float in_r;
layout(location = 2) in float in_g;
layout(location = 3) in float in_b;
layout(location = 5) in float in_radius;
layout(location = 6) in vec3  in_normal;

// VS → GS.
out VsOut {
    vec3 vs_center;
    vec3 vs_half_u;
    vec3 vs_half_v;
    vec3 vs_normal;
    vec3 albedo_rgb;
} vs_out;

// Color mapping includes.
INCLUDE ../common/heatmapping/wavelength_to_rainbow.glsl
INCLUDE ../common/heatmapping/colormap.glsl

vec3 get_output_sensitivity_color(float screen_size) {
    float min_screen_size = 0.0;
    float max_screen_size = 10.0;
    float val = clamp(screen_size, min_screen_size, max_screen_size);
    return data_value_to_rainbow(val, min_screen_size, max_screen_size);
}

vec3 get_color(in vec3 position,
               in vec3 normal,
               in vec3 base_color,
               in float radius,
               in float screen_size) {

    vec3 view_color = base_color;

    if (show_normals) {
        vec3 n_vis = normal;
        if (n_vis.z < 0.0) n_vis *= -1.0;
        view_color = n_vis * 0.5 + 0.5;
    }
    else if (show_output_sensitivity) {
        view_color = get_output_sensitivity_color(screen_size);
    }
    else if (show_radius_deviation) {
        float max_fac  = 2.0;
        float safe_avg = max(1e-8, average_radius);
        view_color = vec3(min(max_fac, radius / safe_avg) / max_fac);
    }
    else {
        view_color = base_color;
    }

    if (show_accuracy) {
        view_color += vec3(accuracy, 0.0, 0.0);
    }

    return view_color;
}

float compute_screen_size_px(vec3 vs_center, vec3 vs_half_u, vec3 vs_half_v, vec2 viewport_wh) {
    vec4 ndc_c = projection_matrix * vec4(vs_center,             1.0);
    vec4 ndc_u = projection_matrix * vec4(vs_center + vs_half_u, 1.0);
    vec4 ndc_v = projection_matrix * vec4(vs_center + vs_half_v, 1.0);

    ndc_c /= max(abs(ndc_c.w), 1e-6) * sign(ndc_c.w);
    ndc_u /= max(abs(ndc_u.w), 1e-6) * sign(ndc_u.w);
    ndc_v /= max(abs(ndc_v.w), 1e-6) * sign(ndc_v.w);

    vec2 sc = (ndc_c.xy * 0.5 + 0.5) * viewport_wh;
    vec2 su = (ndc_u.xy * 0.5 + 0.5) * viewport_wh;
    vec2 sv = (ndc_v.xy * 0.5 + 0.5) * viewport_wh;

    return max(length(sc - su), length(sv - sc));
}


void main() {
    float radius_ws = clamp(in_radius * scale_radius, min_radius, max_radius);

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

    vec3 ws_center = (model_matrix      * vec4(in_position, 1.0)).xyz;
    vec3 vs_center = (model_view_matrix * vec4(in_position, 1.0)).xyz;

    vec3 vs_half_u = (model_view_matrix * vec4(ms_u * radius_ws, 0.0)).xyz;
    vec3 vs_half_v = (model_view_matrix * vec4(ms_v * radius_ws, 0.0)).xyz;
    vec3 vs_normal = normalize(normal_matrix * n);

    // --- KORREKTUR ---
    //if (dot(cross(vs_half_u, vs_half_v), vs_normal) < 0.0) {
    //    vs_half_v = -vs_half_v;
    //}
    // --- ENDE KORREKTUR ---

    vec3 base_color = vec3(in_r, in_g, in_b);

    float screen_size = 0.0;
    if (show_output_sensitivity) {
        screen_size = compute_screen_size_px(vs_center, vs_half_u, vs_half_v, viewport);
    }

    vec3 color_vs = get_color(ws_center, vs_normal, base_color, radius_ws, screen_size);

    vs_out.vs_center  = vs_center;
    vs_out.vs_half_u  = vs_half_u;
    vs_out.vs_half_v  = vs_half_v;
    vs_out.vs_normal  = vs_normal;
    vs_out.albedo_rgb = color_vs;

    gl_Position = projection_matrix * vec4(vs_center, 1.0);
}