#version 420 core

layout(binding = 0) uniform sampler2D in_color_texture;
layout(binding = 1) uniform sampler2D in_normal_texture;
layout(binding = 2) uniform sampler2D in_vs_position_texture;

layout(location = 0) out vec4 out_color;

// Uniforms for lighting and material properties
uniform vec3 background_color;
uniform vec3 ambient_light_color;
uniform vec4 point_light_color;
uniform vec3 point_light_pos;
uniform vec4 material_specular;
uniform int use_material_color;
uniform vec3 material_diffuse;

in vec2 tex_coords;

// Blinn-Phong lighting calculation, adapted from your single-pass shader
vec3 shade_blinn_phong(
    in vec3 vs_pos,
    in vec3 vs_normal,
    in vec3 vs_light_pos,
    in vec3 vs_color) 
{
    vec3 normal     = normalize(vs_normal);
    vec3 view_dir   = normalize(-vs_pos);
    vec3 light_dir  = normalize(vs_light_pos - vs_pos);
    vec3 half_dir   = normalize(light_dir + view_dir);

    float NdotL   = max(dot(normal, light_dir), 0.0);
    float NdotH = max(dot(normal, half_dir), 0.0);

    // Use either material color or vertex color based on the uniform
    vec3 albedo = (use_material_color == 1) ? material_diffuse : vs_color;
    
    // Extract shininess and light intensity from the alpha components
    float shininess = material_specular.a * 128.0; // Scale for a more intuitive range
    float light_intensity = point_light_color.a;

    // Calculate lighting components
    vec3 ambient = ambient_light_color * albedo;
    vec3 diffuse = point_light_color.rgb * albedo * NdotL * light_intensity;
    vec3 specular = point_light_color.rgb * material_specular.rgb * pow(NdotH, shininess) * light_intensity;

    return ambient + diffuse + specular;
}


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
        vec3 final_shaded_color = shade_blinn_phong(
            pos_vs,
            normal_vs,
            point_light_pos, // Global uniform
            albedo           // Decoded surface color
        );

        vec3 tone_mapped_color = final_shaded_color / (final_shaded_color + vec3(1.0));
        float gamma = 2.2;
        vec3 gamma_corrected_color = pow(tone_mapped_color, vec3(1.0 / gamma));
        out_color = vec4(tone_mapped_color, 1.0);

    } else {
        // No surfel here, draw the background color
        discard; // Or out_color = vec4(background_color, 1.0f);
    }
}