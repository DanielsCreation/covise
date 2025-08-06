// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#version 420 core

// --- Uniforms ---
// Matrices
uniform mat4 mvp_matrix;
uniform mat4 model_matrix;
uniform mat4 model_view_matrix;
//uniform mat4 inv_mv_matrix;
uniform mat4 model_to_screen_matrix;
uniform mat3 normal_matrix;
uniform float near_plane;
uniform float max_radius;
uniform float min_radius;
uniform float scale_radius;

uniform bool show_normals;
uniform bool show_accuracy;
uniform bool show_radius_deviation;
uniform bool show_output_sensitivity;

uniform float accuracy;
uniform float average_radius;
uniform int channel;
uniform bool heatmap;
uniform float heatmap_min;
uniform float heatmap_max;
uniform vec3 heatmap_min_color;
uniform vec3 heatmap_max_color;

// --- Inputs ---
layout(location = 0) in vec3 in_position;
layout(location = 1) in float in_r;
layout(location = 2) in float in_g;
layout(location = 3) in float in_b;
layout(location = 4) in float empty;
layout(location = 5) in float in_radius;
layout(location = 6) in vec3 in_normal;
layout(location = 7) in float prov1;
layout(location = 8) in float prov2;
layout(location = 9) in float prov3;
layout(location = 10) in float prov4;
layout(location = 11) in float prov5;
layout(location = 12) in float prov6;

// --- Outputs to Geometry Shader ---
out VertexData {
  vec3 pass_ms_u;
  vec3 pass_ms_v;
  vec3 pass_point_color;
  vec3 pass_vs_normal;
  vec3 mv_vertex_position;
} VertexOut;

//INCLUDE vis_color_prov.glsl

vec3 quick_interp(vec3 color1, vec3 color2, float value) {
  return color1 + (color2 - color1) * clamp(value, 0.0, 1.0);
}

vec3 get_final_color(in vec3 position, in vec3 normal, in vec3 color, in float radius, in vec3 tangent, in vec3 bitangent) {
    vec3 view_color = vec3(0.0);

    if (show_normals) {
        vec4 vis_normal = vec4(normal, 0.0);
        if( vis_normal.z < 0.0 ) {
            vis_normal = vis_normal * -1.0;
        }
        view_color = vec3(vis_normal.xyz * 0.5 + 0.5);
    }
    else if (show_output_sensitivity) {
        float ideal_screen_surfel_size = 2.0;
        float min_screen_surfel_size = 0.0;
        float max_screen_surfel_size = 10.0;
        
        vec4 surfel_pos_screen = model_to_screen_matrix * vec4(position, 1.0);
        surfel_pos_screen /= surfel_pos_screen.w;
        vec4 border_pos_screen_u = model_to_screen_matrix * vec4(position + tangent, 1.0);
        border_pos_screen_u /= border_pos_screen_u.w;
        vec4 border_pos_screen_v = model_to_screen_matrix * vec4(position + bitangent, 1.0);
        border_pos_screen_v /= border_pos_screen_v.w;
        float screen_surfel_size = max(length(surfel_pos_screen.xy - border_pos_screen_u.xy), length(surfel_pos_screen.xy - border_pos_screen_v.xy));
        screen_surfel_size = clamp(screen_surfel_size, min_screen_surfel_size, max_screen_surfel_size);

        // Simplified rainbow heatmap (as original uses another include)
        float t = (screen_surfel_size - min_screen_surfel_size) / (max_screen_surfel_size - min_screen_surfel_size);
        t = clamp(t, 0.0, 1.0);
        float r = mix(0.0, 1.0, t);
        float b = mix(1.0, 0.0, t);
        float g = 1.0 - abs(r - b);
        view_color = vec3(r, g, b);
    }
    else if (show_radius_deviation) {
        float max_fac = 2.0;
        view_color = vec3(min(max_fac, radius / average_radius) / max_fac);
    }
    else if (channel == 0) {
        view_color = color;
    }
    else {
        float prov_value = 0.0;
        if (channel == 1) prov_value = prov1;
        else if (channel == 2) prov_value = prov2;
        else if (channel == 3) prov_value = prov3;
        else if (channel == 4) prov_value = prov4;
        else if (channel == 5) prov_value = prov5;
        else if (channel == 6) prov_value = prov6;

        float value = (prov_value - heatmap_min) / (heatmap_max - heatmap_min);
        if (heatmap) {
            view_color = quick_interp(heatmap_min_color, heatmap_max_color, value);
        } else {
            view_color = vec3(clamp(value, 0.0, 1.0)); 
        }
    }

    if (show_accuracy) {
        view_color += vec3(accuracy, 0.0, 0.0);
    }

    return view_color;
}


void main() {
    float world_radius = clamp(in_radius * scale_radius, min_radius, max_radius);

    vec3 normal = in_normal;

    vec3 tangent, bitangent;
    vec3 ms_n = normalize(normal.xyz);
    vec3 tmp_ms_u;
    if(ms_n.z != 0.0) { tmp_ms_u = vec3(1, 1, (-ms_n.x - ms_n.y) / ms_n.z); } 
    else if (ms_n.y != 0.0) { tmp_ms_u = vec3(1, (-ms_n.x - ms_n.z) / ms_n.y, 1); } 
    else { tmp_ms_u = vec3((-ms_n.y - ms_n.z) / ms_n.x, 1, 1); }
    
    tangent   = normalize(tmp_ms_u) * world_radius;
    bitangent = normalize(cross(ms_n, tmp_ms_u)) * world_radius;

    VertexOut.pass_ms_u = tangent;
    VertexOut.pass_ms_v = bitangent;
    VertexOut.pass_vs_normal = normalize(normal_matrix * normal);

    VertexOut.pass_point_color = vec3(in_r, in_g, in_b);

    gl_Position = vec4(in_position, 1.0);

    vec4 pos_es = model_view_matrix * vec4(in_position, 1.0f);

    VertexOut.mv_vertex_position = pos_es.xyz;
}
