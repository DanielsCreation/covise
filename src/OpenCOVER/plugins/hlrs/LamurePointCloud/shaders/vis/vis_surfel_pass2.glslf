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
  vec3 pass_vs_normal;
  vec2 pass_uv_coords;
  vec3 mv_vertex_position;
} VertexIn;

// Multiple Render Targets for accumulation
layout(location = 0) out vec4 accumulated_colors;
layout(location = 1) out vec3 accumulated_normals;
layout(location = 2) out vec3 accumulated_vs_positions;

uniform vec2 win_size;
uniform sampler2D depth_texture;

void main() {

  if ( dot(VertexIn.pass_uv_coords, VertexIn.pass_uv_coords) > 1.0 )
    discard;

  int idx = int(clamp(length(VertexIn.pass_uv_coords) * 31.0, 0.0, 31.0));
  float weight = gaussian[idx];

  vec3 surfelColor = VertexIn.pass_point_color;
  accumulated_colors = vec4(surfelColor * weight, weight);

  accumulated_normals       = VertexIn.pass_vs_normal * weight;
  accumulated_vs_positions  = VertexIn.mv_vertex_position * weight;
}