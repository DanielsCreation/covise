#version 420 core
layout(early_fragment_tests) in;

in GS_OUT {
    noperspective vec2 uv;
    flat vec3  vs_u;
    flat vec3  vs_v;
    flat vec3  center_vs;
    flat vec3  vs_normal;
    flat vec3  point_color;
    flat float radius_ws;
} fin;

// MRTs
layout(location = 0) out vec4 accumulated_colors;        // rgb=Σ(color*w), a=Σw
layout(location = 1) out vec3 accumulated_normals;       // Σ(normal*w)
layout(location = 2) out vec4 accumulated_vs_position;   // rgb=Σ(pos_vs*w), a=Σ(radius_ws*w)

// Gaussian LUT (32)
const float gaussian[32] = float[](
  1.000000, 1.000000, 0.988235, 0.968627, 0.956862, 0.917647, 0.894117, 0.870588, 0.915686, 0.788235,
  0.749020, 0.690196, 0.654902, 0.619608, 0.552941, 0.513725, 0.490196, 0.458824, 0.392157, 0.356863,
  0.341176, 0.278431, 0.254902, 0.227451, 0.188235, 0.164706, 0.152941, 0.125490, 0.109804, 0.098039,
  0.074510, 0.062745
);

// Depth-Band Test (gegen vorderste Schicht aus Pass 1)
layout(binding = 3) uniform sampler2D u_depth_tex; // aus Pass 1
uniform float u_near_plane;
uniform float u_far_plane;
uniform float u_depth_epsilon_vs; // Breite der Schicht in View-Space

float linearize_depth(float d, float n, float f) {
    // d in [0,1] -> NDC z in [-1,1]
    float z_ndc = d * 2.0 - 1.0;
    // Rückgabe: positive lineare Distanz (vor Kamera)
    return (2.0 * n * f) / (f + n - z_ndc * (f - n));
}

void main() {
    vec2 uv = fin.uv;
    if (dot(uv, uv) > 1.0) discard;

    // Rekonstruierte Position in VS
    vec3 pos_vs = fin.center_vs + fin.vs_u * uv.x + fin.vs_v * uv.y;

    // Tiefenband-Prüfung gegen vorderste Surfel-Schicht (Pass1)
    float scene_depth = texture(u_depth_tex, gl_FragCoord.xy / textureSize(u_depth_tex, 0)).r;
    if (scene_depth >= 1.0) discard; // nichts im Prepass

    float z_front_lin = linearize_depth(scene_depth, u_near_plane, u_far_plane); // >0
    float z_front_vs  = -z_front_lin;
    float dz = abs(pos_vs.z - z_front_vs);
    if (dz > u_depth_epsilon_vs) discard;

    // Gewicht & Accumulation
    float r = clamp(length(uv) * 31.0, 0.0, 31.0);
    float w = gaussian[int(r)];

    accumulated_colors       = vec4(fin.point_color * w, w);
    accumulated_normals      = normalize(fin.vs_normal) * w;
    accumulated_vs_position  = vec4(pos_vs * w, fin.radius_ws * w);
}
