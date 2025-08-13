#version 420 core
//layout(early_fragment_tests) in;

uniform sampler2D depth_texture;
uniform vec2      viewport;
uniform float     near_plane;
uniform float     far_plane;

// Input from GS.
in GsOut {
    noperspective vec2 uv;   // -1..1
    flat vec3 vs_center;
    flat vec3 vs_half_u;
    flat vec3 vs_half_v;
    flat vec3 vs_normal;
    flat vec3 albedo_rgb;
} fs_in;

// Accumulation targets.
layout(location = 0) out vec4 accumulated_colors;      // rgb = Σ(color*w), a = Σw
layout(location = 1) out vec3 accumulated_normals;     // Σ(normal*w)
layout(location = 2) out vec3 accumulated_vs_positions;// Σ(pos_vs*w)

// Gaussian LUT.
const float gaussian[32] = float[](
  1.000000, 1.000000, 0.988235, 0.968627, 0.956862, 0.917647, 0.894117, 0.870588, 0.915686, 0.788235,
  0.749020, 0.690196, 0.654902, 0.619608, 0.552941, 0.513725, 0.490196, 0.458824, 0.392157, 0.356863,
  0.341176, 0.278431, 0.254902, 0.227451, 0.188235, 0.164706, 0.152941, 0.125490, 0.109804, 0.098039,
  0.074510, 0.062745
);

float linearize_depth(float d) {
     if (d == 1.0) return far_plane; // Hintergrund-Pixel nicht umrechnen
     float z_ndc = d * 2.0 - 1.0; // Konvertiert den Bereich [0,1] zu [-1,1]
     return (2.0 * near_plane * far_plane) / (far_plane + near_plane - z_ndc * (far_plane - near_plane));
}

void main() {
    vec2 uv = fs_in.uv;
    if (dot(uv, uv) > 1.0) discard;

    float idx = clamp(length(uv) * 31.0, 0.0, 31.0);
    float w   = gaussian[int(idx)];

    vec3 pos_vs = fs_in.vs_center + fs_in.vs_half_u * uv.x + fs_in.vs_half_v * uv.y;
    float current_linear_z = abs(pos_vs.z);
    vec2 screen_uv = gl_FragCoord.xy / viewport;
    float stored_depth_ndc = texture(depth_texture, screen_uv).r;
    float stored_linear_z = linearize_depth(stored_depth_ndc);

    if (current_linear_z > stored_linear_z + length(fs_in.vs_half_u + fs_in.vs_half_v)) {
      discard;
    }

    accumulated_colors       = vec4(fs_in.albedo_rgb * w, w);
    accumulated_normals      = normalize(fs_in.vs_normal) * w;
    accumulated_vs_positions = pos_vs * w;
}
