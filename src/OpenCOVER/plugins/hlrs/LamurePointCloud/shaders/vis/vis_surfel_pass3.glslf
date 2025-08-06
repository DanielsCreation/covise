#version 420 core

layout(binding = 0) uniform sampler2D in_color_texture;
layout(binding = 1) uniform sampler2D in_normal_texture;
layout(binding = 2) uniform sampler2D in_vs_position_texture;

layout(location = 0) out vec4 out_color;

uniform vec3 background_color;

in vec2 tex_coords;

INCLUDE ../common/shading/lighting.glsl

void main() {
    // Sample the G-Buffer textures
    vec4 accumulated_color    = texture(in_color_texture,       tex_coords.xy);
    vec3 accumulated_normal   = texture(in_normal_texture,      tex_coords.xy).rgb;
    vec3 accumulated_pos_vs   = texture(in_vs_position_texture, tex_coords.xy).rgb;

    // Check if a surfel contributed to this pixel
    if (accumulated_color.a > 0.0) {
        // Decode the G-Buffer values by dividing by the accumulated weight (alpha)
        float total_weight = accumulated_color.a;
        vec3 albedo        = accumulated_color.rgb / total_weight;
        vec3 normal_vs     = normalize(accumulated_normal / total_weight);
        vec3 pos_vs        = accumulated_pos_vs / total_weight;

        // Calculate the final color using the Blinn-Phong shading function
        out_color = vec4( shade_blinn_phong(pos_vs, normal_vs, albedo) , 1.0);

    } else {
        discard; // Or out_color = vec4(background_color, 1.0f);
    }
}