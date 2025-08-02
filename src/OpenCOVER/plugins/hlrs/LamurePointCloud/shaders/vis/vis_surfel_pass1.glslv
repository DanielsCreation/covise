#version 420 core

out VertexData {
  vec3 pass_ms_u;
  vec3 pass_ms_v;

} VertexOut;

// Custom scaling uniforms
uniform float max_radius;
uniform float min_radius;
uniform float scale_radius;
uniform mat4 mvp_matrix;


layout(location = 0) in vec3 in_position;
layout(location = 1) in float in_r;
layout(location = 2) in float in_g;
layout(location = 3) in float in_b;
layout(location = 4) in float empty;
layout(location = 5) in float in_radius;
layout(location = 6) in vec3 in_normal;


void main() {

  vec3 n = normalize(in_normal);
  vec3 ref;
  if (abs(n.x) > abs(n.y) && abs(n.x) > abs(n.z)) { ref = vec3(0.0, 1.0, 0.0); } 
  else if (abs(n.y) > abs(n.z)) { ref = vec3(0.0, 0.0, 1.0); } 
  else { ref = vec3(1.0, 0.0, 0.0); }
  
  vec3 u = normalize(cross(ref, n));
  vec3 v = normalize(cross(n, u));

  float r_world = clamp(in_radius * scale_radius, min_radius, max_radius);

  VertexOut.pass_ms_u        = u * (r_world * 0.5);
  VertexOut.pass_ms_v        = v * (r_world * 0.5);

  gl_Position = vec4(in_position, 1.0);
}