#version 420 core

layout(binding = 0) uniform sampler2D in_color_texture;
layout(binding = 1) uniform sampler2D in_normal_texture;
layout(binding = 2) uniform sampler2D in_vs_position_texture;

layout(location = 0) out vec4 out_color;

uniform vec3 background_color;
uniform vec3 ambient_light_color;
uniform vec4 point_light_color;
uniform vec3 point_light_pos;
uniform vec4 material_specular;
uniform int use_material_color;
uniform vec3 material_diffuse;


in vec2 pos;

vec3 shade_blinn_phong(vec3 vs_pos, vec3 vs_normal, vec3 vs_light_pos, vec3 light_color, vec3 ambient_color, vec3 diffuse_color, vec4 specular_color) {
    vec3 n = normalize(vs_normal);
    vec3 l = normalize(vs_light_pos - vs_pos);
    vec3 v = normalize(-vs_pos); // View vector is from fragment to origin in view space
    vec3 h = normalize(l + v); // Blinn-Phong halfway vector

    // Ambient component
    vec3 ambient = ambient_color * diffuse_color;

    // Diffuse component
    float NdotL = max(dot(n, l), 0.0);
    vec3 diffuse = light_color * diffuse_color * NdotL;

    // Specular component
    float NdotH = max(dot(n, h), 0.0);
    vec3 specular = light_color * specular_color.rgb * pow(NdotH, specular_color.a * 128.0);

    return ambient + diffuse + specular;
}

void main()	{
    // Default to background color
    out_color = vec4(background_color, 1.0f);

    // Read the accumulated color and weight from the first texture
    vec4 texColor = texture(in_color_texture, (pos.xy + 1.0) / 2.0);
	
    // Process only if a fragment was actually rendered here (weight is not zero)
    if(texColor.w > 0.0) {
      // Normalize the color by dividing by the accumulated weight
      vec3 diffuse_color = use_material_color > 0 ? material_diffuse : texColor.xyz / texColor.w;
      
      // Read and normalize accumulated normals and positions
      vec3 texNormal = texture(in_normal_texture, (pos.xy + 1.0) / 2.0).xyz;
      vec3 texVSPosition = texture(in_vs_position_texture, (pos.xy + 1.0) / 2.0).xyz;
      texNormal = normalize(texNormal / texColor.w);
      texVSPosition = texVSPosition / texColor.w;

      // Calculate final color using Blinn-Phong shading
      vec3 shaded_color = shade_blinn_phong(texVSPosition, texNormal, point_light_pos, point_light_color.rgb, ambient_light_color, diffuse_color, material_specular);

      out_color = vec4(shaded_color, 1.0);
    }
}