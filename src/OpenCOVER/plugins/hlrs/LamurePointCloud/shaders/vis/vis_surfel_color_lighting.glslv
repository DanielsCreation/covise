#version 420 core // vis_surfel_color.glslv (Vertex Shader)

layout(location = 0)  in vec3  in_position;
layout(location = 1)  in float in_r;
layout(location = 2)  in float in_g;
layout(location = 3)  in float in_b;
layout(location = 4)  in float empty;
layout(location = 5)  in float in_radius;
layout(location = 6)  in vec3  in_normal;

uniform mat4  mvp_matrix;
uniform mat4  view_matrix;
uniform mat3  view_normal_matrix;
uniform float max_radius;
uniform float min_radius;
uniform float scale_radius;

out VertexData {
    vec3  pass_ms_u;
    vec3  pass_ms_v;
    vec3  pass_point_color;
    vec3  pass_world_pos;
    vec3  pass_vs_pos;
    vec3  pass_vs_normal;
    float pass_radius_ws;
} VertexOut;

void main() {
    // Welt-Position und Normalen
    vec3 worldPos = in_position;
    vec3 normalWS = normalize(in_normal);

    // Position und Normale im View-Space
    vec4 vsPos4 = view_matrix * vec4(worldPos, 1.0);
    vec3 vsPos = vsPos4.xyz;
    vec3 normalVS = normalize(view_normal_matrix * normalWS);

    // Berechne lokale Tangenten-Halbachsen für Quad-Generierung
    vec3 ref = (abs(normalWS.x) > abs(normalWS.y) && abs(normalWS.x) > abs(normalWS.z))
        ? vec3(0.0,1.0,0.0) : (abs(normalWS.y) > abs(normalWS.z)
        ? vec3(0.0,0.0,1.0) : vec3(1.0,0.0,0.0));
    vec3 u = normalize(cross(ref, normalWS));
    vec3 v = normalize(cross(normalWS, u));

    float r_world = clamp(in_radius * scale_radius, min_radius, max_radius);

    // Daten an Geometry Shader weitergeben
    VertexOut.pass_ms_u        = u * r_world;
    VertexOut.pass_ms_v        = v * r_world;
    VertexOut.pass_point_color = vec3(in_r, in_g, in_b);
    VertexOut.pass_world_pos   = worldPos;
    VertexOut.pass_vs_pos      = vsPos;
    VertexOut.pass_vs_normal   = normalVS;
    VertexOut.pass_radius_ws   = r_world;

    gl_Position = mvp_matrix * vec4(worldPos, 1.0);
}
