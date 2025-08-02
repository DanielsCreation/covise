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

// Include for debug/visualization modes
INCLUDE vis_color_no_prov.glsl

// Blinn-Phong lighting function (from Surfel-Shader)
vec3 shade_blinn_phong(
    in vec3 vs_pos,
    in vec3 vs_normal,
    in vec3 vs_light_pos,
    in vec3 albedo_base) 
{
    vec3 normal     = normalize(vs_normal);
    vec3 light_dir  = normalize(vs_light_pos - vs_pos);
    vec3 view_dir   = normalize(-vs_pos);
    vec3 half_dir   = normalize(light_dir + view_dir);

    float NdotL   = max(dot(normal, light_dir), 0.0);
    float NdotH   = max(dot(normal, half_dir), 0.0);

    vec3 albedo = (use_material_color == 1) ? material_diffuse : albedo_base;
    float shininess = material_specular.a;
    float light_intensity = point_light_color.a;

    vec3 ambient = ambient_light_color * albedo;
    vec3 diffuse = point_light_color.rgb * albedo * NdotL * light_intensity;
    vec3 specular = point_light_color.rgb * material_specular.rgb * pow(NdotH, shininess) * light_intensity;

    return ambient + diffuse + specular;
}


void main() {


    // Get the base color from the visualization mode (e.g., pure color, normals, etc.)
    vec3 modeColor = get_color(
        fsIn.pass_world_pos,
        fsIn.pass_vs_normal, // Pass view-space normal
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
            point_light_pos,
            modeColor
        );
    }

    out_color = vec4(finalColor, 1.0);
}
