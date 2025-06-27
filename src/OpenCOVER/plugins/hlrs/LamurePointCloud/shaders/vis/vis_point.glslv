#version 420 core

out VertexData {
    vec3 color;
} VertexOut;

layout(location = 0) in vec3  in_position;
layout(location = 1) in float in_r;
layout(location = 2) in float in_g;
layout(location = 3) in float in_b;
layout(location = 4) in float empty;
layout(location = 5) in float in_radius;
layout(location = 6) in vec3  in_normal;

uniform mat4  mvp_matrix;
uniform float max_radius;
uniform float scale_radius;
uniform float point_size_factor;
uniform float proj_scale;

void main() {

    // Position und Punktgröße berechnen
    vec4 clipPos = mvp_matrix * vec4(in_position, 1.0);
    gl_Position  = clipPos;
    float radius = min(in_radius * scale_radius, max_radius);
    gl_PointSize = radius * point_size_factor * proj_scale / abs(clipPos.w);

    // Farbe weiterreichen
    VertexOut.color = vec3(in_r, in_g, in_b);
}