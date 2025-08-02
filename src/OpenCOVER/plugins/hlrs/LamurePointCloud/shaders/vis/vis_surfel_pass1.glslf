#version 420 core  

out vec4 FragColor;

in VertexData {
    vec2 pass_uv_coords;
} VertexIn;

void main() {
  // Re-enable discard to render circular surfels instead of quads.
  if( dot(VertexIn.pass_uv_coords, VertexIn.pass_uv_coords) >  1.0 ) {
    discard;
  }

  // WORKAROUND: Write depth to the color buffer, as direct depth-only rendering fails.
  float depth = gl_FragCoord.z;
  FragColor = vec4(depth, depth, depth, 1.0);
}
