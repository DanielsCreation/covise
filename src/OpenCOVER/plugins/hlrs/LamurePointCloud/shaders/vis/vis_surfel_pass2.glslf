#version 420 core

layout(early_fragment_tests) in;

// Gaussian falloff for soft surfel edges
const float gaussian[32] = float[](
  1.000000, 1.000000, 0.988235, 0.968627, 0.956862, 0.917647, 0.894117, 0.870588, 0.915686, 0.788235,
  0.749020, 0.690196, 0.654902, 0.619608, 0.552941, 0.513725, 0.490196, 0.458824, 0.392157, 0.356863,
  0.341176, 0.278431, 0.254902, 0.227451, 0.188235, 0.164706, 0.152941, 0.125490, 0.109804, 0.098039,
  0.074510, 0.062745
);

in VertexData {
  vec3 pass_point_color;
  vec3 pass_normal;
  vec2 pass_uv_coords;
  vec3 mv_vertex_position;
} VertexIn;

// Multiple Render Targets for accumulation
layout(location = 0) out vec4 accumulated_colors;
layout(location = 1) out vec3 accumulated_normals;
layout(location = 2) out vec3 accumulated_vs_positions;

uniform vec2 win_size;

void main() {
  // Discard fragments outside the circular surfel area
  if ( dot(VertexIn.pass_uv_coords, VertexIn.pass_uv_coords) > 1.0 )
    discard;

  // Calculate weight based on distance from surfel center using Gaussian table
  float weight = gaussian[int(round(length(VertexIn.pass_uv_coords) * 31.0 ))];

  // Accumulate weighted color and the weight itself in the alpha channel
  accumulated_colors = vec4(VertexIn.pass_point_color * weight, weight);

  // Accumulate weighted normals and positions for deferred shading
  vec3 adjustedNormal = VertexIn.pass_normal;
  if (adjustedNormal.z < 0.0) {
    adjustedNormal *= -1.0;
  }
  accumulated_normals = adjustedNormal * weight;
  accumulated_vs_positions = VertexIn.mv_vertex_position * weight;
}