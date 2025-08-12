#version 420 core

layout(binding = 0) uniform sampler2D in_color_texture;        // rgba: Σ(color*w), a=Σw
layout(binding = 1) uniform sampler2D in_normal_texture;       // rgb:  Σ(normal*w)
layout(binding = 2) uniform sampler2D in_vs_position_texture;  // rgba: Σ(pos_vs*w), a=Σ(radius_ws*w)

layout(location = 0) out vec4 out_color;

noperspective in vec2 tex_coords;

// ---- deine Includes ----
INCLUDE ../common/shading/lighting.glsl
INCLUDE vis_color_no_prov.glsl

// Zusatzuniforms
uniform mat4 inv_view_matrix;   // Welt = inv(V) * vec4(pos_vs,1)
uniform float fy_pixels;        // P[1][1] * (viewport_height*0.5)
uniform vec3 background_color;  // für leere Pixel

void main() {
    vec4 acc_col = texture(in_color_texture, tex_coords);
    float w = acc_col.a;
    if (w <= 1e-6) {
        out_color = vec4(background_color, 1.0);
        return;
    }

    vec3 albedo   = acc_col.rgb / w;
    vec3 acc_nrm  = texture(in_normal_texture,      tex_coords).rgb;
    vec4 acc_pos4 = texture(in_vs_position_texture, tex_coords);

    vec3 pos_vs   = acc_pos4.rgb / w;
    float radius_ws = (acc_pos4.a > 0.0) ? (acc_pos4.a / w) : average_radius;

    vec3 n = acc_nrm / w;
    float nl2 = dot(n, n);
    vec3 normal_vs = (nl2 > 1e-12) ? n * inversesqrt(nl2) : vec3(0.0, 0.0, 1.0);

    vec3 world_pos = (inv_view_matrix * vec4(pos_vs, 1.0)).xyz;

    // Pixelgröße: r_screen ≈ fy * r_ws / |z_vs|
    float z = abs(pos_vs.z) + 1e-6;
    float screen_size_px = (fy_pixels * radius_ws) / z;

    // Mode-Farbe von deiner Funktion
    vec3 modeColor = get_color(
        world_pos,
        normal_vs,
        albedo,          // base_color
        radius_ws,
        screen_size_px
    );

    // Debug-Flags kommen aus vis_color_no_prov.glsl-Uniforms.
    vec3 finalColor = (show_normals || show_radius_deviation || show_output_sensitivity || show_accuracy)
        ? modeColor
        : shade_blinn_phong(pos_vs, normal_vs, modeColor);

    out_color = vec4(shade_blinn_phong(pos_vs, normal_vs, modeColor), 1.0);
}
