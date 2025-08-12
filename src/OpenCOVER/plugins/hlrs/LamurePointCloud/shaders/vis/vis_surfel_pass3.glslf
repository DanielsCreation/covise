#version 420 core

layout(binding = 0) uniform sampler2D in_color_texture;        // rgba: accumulated color (a = weight sum)
layout(binding = 1) uniform sampler2D in_normal_texture;       // rgb:  accumulated normals
layout(binding = 2) uniform sampler2D in_vs_position_texture;  // rgb:  accumulated view-space positions

layout(location = 0) out vec4 out_color;

uniform vec3 background_color;
uniform bool  show_normals;
uniform bool  show_accuracy;
uniform bool  show_radius_deviation;
uniform bool  show_output_sensitivity;

// VS → FS.
in VsOut {
    vec2 uv;  // 0..1
} fs_in;

INCLUDE ../common/shading/lighting.glsl

void main()
{
    vec2 t = fs_in.uv;

    // Sample accumulated G-Buffer.
    vec4 accumulated_color  = texture(in_color_texture,       t);
    vec3 accumulated_normal = texture(in_normal_texture,      t).rgb;
    vec3 accumulated_pos_vs = texture(in_vs_position_texture, t).rgb;

    // Resolve if any contribution exists (alpha stores total weight).
    if (accumulated_color.a > 0.0) {
        float total_weight = accumulated_color.a;

        vec3 albedo    = accumulated_color.rgb  / total_weight;
        vec3 normal_vs = normalize(accumulated_normal / total_weight);
        vec3 pos_vs    = accumulated_pos_vs     / total_weight;


        if (show_normals || show_accuracy || show_radius_deviation || show_output_sensitivity) {
            out_color = vec4(albedo, 1.0);
        } else {
            out_color = vec4(shade_blinn_phong(pos_vs, normal_vs, albedo), 1.0);
        }
    }
}
