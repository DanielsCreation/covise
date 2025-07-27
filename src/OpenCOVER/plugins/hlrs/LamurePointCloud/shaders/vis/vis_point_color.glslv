// Vertex Shader
#version 420 core

layout(location = 0) in vec3  in_position;
layout(location = 1) in float in_r;
layout(location = 2) in float in_g;
layout(location = 3) in float in_b;
layout(location = 4) in float empty;
layout(location = 5) in float in_radius;
layout(location = 6) in vec3  in_normal;

uniform mat4  mvp_matrix;
uniform float max_radius;
uniform float min_radius;
uniform float scale_radius;
uniform float scale_projection;

out VertexData {
    vec3  pass_point_color;  // entspricht in_r/g/b
    vec3  pass_world_pos;   // Position in Welt
    vec3  pass_normal_ws;   // Normale im World-Space
    float pass_radius_ws;   // Radius nach clamp
    float pass_screen_size;  // finale PointSize
} VertexOut;

void main() {
    vec4 clipPos = mvp_matrix * vec4(in_position, 1.0);
    gl_Position = clipPos;

    float r = clamp(in_radius * scale_radius, min_radius, max_radius);
    gl_PointSize = r * scale_projection / abs(clipPos.w);

    VertexOut.pass_point_color = vec3(in_r, in_g, in_b);
    VertexOut.pass_world_pos  = in_position;
    VertexOut.pass_normal_ws  = normalize(in_normal);
    VertexOut.pass_radius_ws  = r;
    VertexOut.pass_screen_size = gl_PointSize;
}
