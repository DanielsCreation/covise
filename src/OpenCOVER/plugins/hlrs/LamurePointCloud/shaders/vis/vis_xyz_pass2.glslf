
OPTIONAL_BEGIN
  layout(location = 1) out vec3 accumulated_normals;
  layout(location = 2) out vec3 accumulated_vs_positions;
OPTIONAL_END

uniform vec2 win_size;

void main() {
  vec2 uv_coords = VertexIn.pass_uv_coords;

  if ( dot(uv_coords, uv_coords) > 1 )
    discard;

  vec3 normal = VertexIn.pass_normal;

  if( normal.z < 0 )
    normal = normal * -1; 

  normal = (normal + vec3(1.0, 1.0, 1.0)) / 2.0;
  float weight = gaussian[int(round(length(uv_coords) * 31.0 ))];

  accumulated_colors = vec4(VertexIn.pass_point_color * weight, weight);


  OPTIONAL_BEGIN
    vec3 adjustedNormal = vec3(0.0,0.0,0.0);
    if (VertexIn.pass_normal.z < 0) {
      adjustedNormal = VertexIn.pass_normal.xyz * -1;
    }
    else {
      adjustedNormal = VertexIn.pass_normal.xyz;
    }
    accumulated_normals = vec3(adjustedNormal.xyz * weight);
    accumulated_vs_positions = vec3(VertexIn.mv_vertex_position.xyz * weight);
  OPTIONAL_END


