// ../../common/shading/lighting.glsl

uniform vec3  point_light_pos_vs;
uniform float point_light_intensity;
uniform float ambient_intensity;
uniform float specular_intensity;
uniform float shininess;
uniform float gamma;
uniform bool  use_tone_mapping;

vec3 shade_blinn_phong(in vec3 vs_pos, in vec3 vs_normal, in vec3 vs_color) 
{
    vec3 normal   = normalize(vs_normal);
    vec3 view_dir = normalize(-vs_pos);
    vec3 light_dir = normalize(point_light_pos_vs - vs_pos);

    if (dot(normal, view_dir) < 0.0) { normal *= -1.0; }

    float NdotL = max(dot(normal, light_dir), 0.0);
    //if (NdotL <= 0.0) {
    //    vec3 amb = ambient_intensity * vs_color;
    //    vec3 mapped = use_tone_mapping ? (amb / (amb + vec3(1.0))) : amb;
    //    return pow(mapped, vec3(1.0 / max(gamma, 1e-6)));
    //}

    vec3  half_dir = normalize(light_dir + view_dir);
    float NdotH    = max(dot(normal, half_dir), 0.0);

    vec3 ambient   = ambient_intensity * vs_color;
    vec3 diffuse   = point_light_intensity * vs_color * NdotL;
    // klassisches Blinn-Phong, optional könnte man (shininess+8)/(8*pi) normalisieren
    vec3 specular  = point_light_intensity * vec3(specular_intensity) * pow(NdotH, max(shininess, 0.0));

    vec3 shaded = ambient + diffuse + specular;

    if (use_tone_mapping) {
        shaded = shaded / (shaded + vec3(1.0)); // Reinhard
    }

    // Gamma-Korrektur (linear -> display)
    return pow(shaded, vec3(1.0 / max(gamma, 1e-6)));
}
