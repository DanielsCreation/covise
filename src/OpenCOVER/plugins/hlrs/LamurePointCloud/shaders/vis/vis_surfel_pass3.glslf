// ---------- vis_surfel_pass3.glslf ----------
#version 420 core

layout(binding = 0) uniform sampler2D in_color_texture;       // rgba: Σ(color*w), a: Σw
layout(binding = 1) uniform sampler2D in_normal_texture;      // rgb:  Σ(normal*w)
layout(binding = 2) uniform sampler2D in_vs_position_texture; // rgb:  Σ(pos_vs*w)

layout(location = 0) out vec4 out_color;

uniform vec3 background_color;
uniform bool show_normals;
uniform bool show_accuracy;
uniform bool show_radius_deviation;
uniform bool show_output_sensitivity;

in VsOut { vec2 uv; } fs_in;

INCLUDE ../common/shading/lighting.glsl

void main() {
    vec2 t = fs_in.uv;

    vec4 accumulated_color  = texture(in_color_texture,       t);
    vec3 accumulated_normal = texture(in_normal_texture,      t).rgb;
    vec3 accumulated_pos_vs = texture(in_vs_position_texture, t).rgb;

    if (accumulated_color.a > 0.0) {
        float total_weight = accumulated_color.a;
        vec3  albedo       = accumulated_color.rgb  / total_weight;
        vec3  normal_vs    = normalize(accumulated_normal / total_weight);
        vec3  pos_vs       = accumulated_pos_vs     / total_weight;

        // Coverage-Alpha aus Summe der Gewichte
        const float kCovScale = 2.0; // Rand-Schärfe (1.5..3.0)
        float alpha = clamp(kCovScale * total_weight, 0.0, 1.0);

        vec3 shaded = (show_normals || show_accuracy || show_radius_deviation || show_output_sensitivity)
                      ? albedo
                      : shade_blinn_phong(pos_vs, normal_vs, albedo);

        out_color = vec4(shaded * alpha, alpha); // premultiplied
    } else {
        // Nichts akkumuliert → Hintergrund bleibt unberührt (Renderer blendet drüber)
        discard;
    }
}
