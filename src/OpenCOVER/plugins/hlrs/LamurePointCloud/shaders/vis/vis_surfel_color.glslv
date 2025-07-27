// vis_surfel_color.glslv (Vertex Shader)
#version 420 core

layout(location = 0)  in vec3  in_position;
layout(location = 1)  in float in_r;
layout(location = 2)  in float in_g;
layout(location = 3)  in float in_b;
layout(location = 4)  in float empty;
layout(location = 5)  in float in_radius;
layout(location = 6)  in vec3  in_normal;

uniform mat4  mvp_matrix;
uniform float max_radius;
uniform float min_radius;
uniform float scale_radius;

out VertexData {
    vec3  pass_ms_u;         // Tangenten-Halbachse U
    vec3  pass_ms_v;         // Tangenten-Halbachse V
    vec3  pass_point_color;  // Basis-Punktfarbe
    vec3  pass_world_pos;    // Surfel-Mittelpunkt (Welt)
    vec3  pass_normal_ws;    // Normale (Welt)
    float pass_radius_ws;    // Radius (Welt, geclamped)
} VertexOut;

void main() {

    vec3 n = normalize(in_normal);

    vec3 ref = (abs(n.x) > abs(n.y) && abs(n.x) > abs(n.z))
        ? vec3(0.0,1.0,0.0)
        : (abs(n.y) > abs(n.z)
           ? vec3(0.0,0.0,1.0)
           : vec3(1.0,0.0,0.0));

    vec3 u = normalize(cross(ref, n));
    vec3 v = normalize(cross(n, u));

    float r_world = clamp(in_radius * scale_radius, min_radius, max_radius);

    VertexOut.pass_ms_u        = u * (r_world * 0.5);
    VertexOut.pass_ms_v        = v * (r_world * 0.5);
    VertexOut.pass_point_color = vec3(in_r, in_g, in_b);
    VertexOut.pass_world_pos   = in_position;
    VertexOut.pass_normal_ws   = n;
    VertexOut.pass_radius_ws   = r_world;

    gl_Position = mvp_matrix * vec4(in_position, 1.0);
}
