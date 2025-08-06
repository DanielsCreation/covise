
uniform vec3   point_light_pos_vs;
uniform float  point_light_intensity;
uniform float  ambient_intensity;
uniform float  specular_intensity;
uniform float  shininess;
uniform float  gamma;
uniform bool   use_tone_mapping;

vec3 shade_blinn_phong(
    in vec3 vs_pos,
    in vec3 vs_normal,
    in vec3 vs_color) 
{
    vec3 normal     = normalize(vs_normal);
    vec3 view_dir   = normalize(-vs_pos);
    vec3 light_dir  = normalize(point_light_pos_vs - vs_pos);
    vec3 half_dir   = normalize(light_dir + view_dir);

    float NdotL   = max(dot(normal, light_dir), 0.0);
    float NdotH = max(dot(normal, half_dir), 0.0);

    vec3 ambient = ambient_intensity * vs_color;
    vec3 diffuse = point_light_intensity * vs_color * NdotL;
    vec3 specular = point_light_intensity * vec3(specular_intensity) * pow(NdotH, shininess);
    vec3 shaded_color = ambient + diffuse + specular;
    
    if (use_tone_mapping) {
        shaded_color = shaded_color / (shaded_color + vec3(1.0));
    }

    vec3 corrected_color = pow(shaded_color, vec3(1.0 / gamma));

    return corrected_color;
}
