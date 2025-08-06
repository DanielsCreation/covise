// Fragment Shader for Point-based Rendering with Lighting
#version 420 core

in VertexData {
    vec3  pass_point_color;
    vec3  pass_world_pos;
    vec3  pass_vs_pos;      // Position in View-Space
    vec3  pass_vs_normal;   // Normal in View-Space
    float pass_radius_ws;
    float pass_screen_size;
} fsIn;

layout(location = 0) out vec4 out_color;

// UNIFORMS (from Surfel-Shader)
uniform int   use_material_color;
uniform vec3  material_diffuse;
uniform vec4  material_specular;
uniform vec3  ambient_light_color;
uniform vec4  point_light_color;
uniform vec3  point_light_pos;


INCLUDE vis_color_no_prov.glsl
INCLUDE ../common/shading/lighting.glsl



void main() {
    // Get the base color from the visualization mode (e.g., pure color, normals, etc.)
    vec3 modeColor = get_color(
        fsIn.pass_world_pos,
        fsIn.pass_vs_normal,
        fsIn.pass_point_color,
        fsIn.pass_radius_ws,
        fsIn.pass_screen_size
    );

    vec3 finalColor;

    // If a debug/visualization mode is active, show its color directly.
    // Otherwise, apply full lighting.
    if (show_normals || show_radius_deviation || show_output_sensitivity || show_accuracy) {
        finalColor = modeColor;
    } else {
        // All calculations are done in View-Space
        finalColor = shade_blinn_phong(
            fsIn.pass_vs_pos,
            fsIn.pass_vs_normal,
            modeColor
        );
    }

    out_color = vec4(finalColor, 1.0);
}
