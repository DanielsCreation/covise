#version 420 core  

uniform float near_plane;
uniform float far_plane;

in VertexData {
    vec2 pass_uv_coords;
    vec3 pass_vs_pos;
} VertexIn;

void main() {
  if( dot(VertexIn.pass_uv_coords, VertexIn.pass_uv_coords) >  1.0 ) { 
    discard; 
  }
  float d = (-VertexIn.pass_vs_pos.z - near_plane) / (far_plane / 3 - near_plane);
  gl_FragDepth = clamp(d, 0.0, 1.0);
}