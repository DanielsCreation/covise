#version 420 core

layout(location = 0) in vec3  in_position;
layout(location = 1) in float in_r;
layout(location = 2) in float in_g;
layout(location = 3) in float in_b;
layout(location = 4) in float empty;      // (Placeholder, wird hier nicht verwendet)
layout(location = 5) in float in_radius;  // Welt-Radius für jeden Punkt
layout(location = 6) in vec3  in_normal;

layout(location = 7)  in float prov1;
layout(location = 8)  in float prov2;
layout(location = 9)  in float prov3;
layout(location = 10) in float prov4;
layout(location = 11) in float prov5;
layout(location = 12) in float prov6;

out vec3 point_color;

uniform mat4 model_matrix;        
uniform mat4 model_view_matrix;   // (wird hier aktuell nicht direkt verwendet)
uniform mat4 projection_matrix;   // **nur** reine Projektionsmatrix (ohne View/Model!)
uniform mat4 mvp_matrix;          // ganzes Model-View-Proj
uniform mat4 inv_mv_matrix;
uniform mat4 model_to_screen_matrix;

uniform vec2 viewport;            // = vec2(windowWidth, windowHeight)

uniform bool face_eye;            // falls du „Facing‐Camera“ machst
uniform vec3 eye;                 // Blickrichtung der Kamera, falls benötigt

uniform float max_radius;         
uniform float point_size_factor;  
uniform float model_radius_scale; 

INCLUDE vis_color.glsl

void main()
{
    float radius_clamped = min(in_radius, max_radius);
    vec3 normal = in_normal;
    if (face_eye) {
        normal = normalize(eye - (model_matrix * vec4(in_position,1.0)).xyz);
    }

    point_color = get_color(in_position, normal, vec3(in_r, in_g, in_b), radius_clamped);
    vec4 clipPos = mvp_matrix * vec4(in_position, 1.0);
    gl_Position = clipPos;
    float rw = model_radius_scale * radius_clamped * point_size_factor;
    float f = projection_matrix[1][1];
    float pixelRadius = rw * (viewport.y * 0.5) * f / abs(clipPos.w);
    float minSize = 1.0;   // z. B. 1 Pixel Mindestgröße
    float maxSize = 256.0; // z. B. Maximal 256 Pixel
    gl_PointSize = clamp(pixelRadius, minSize, maxSize);
}


