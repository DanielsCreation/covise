#version 420 core

uniform float depth_range;
uniform float flank_lift;

uniform sampler2D depth_texture;      // Prepass depth (GL_NEAREST)
uniform mat4      projection_matrix;  // vorhanden, aber für den Test nicht mehr nötig

// Gaussian LUT
const float gaussian[32] = float[](
  1.000000, 1.000000, 0.988235, 0.968627, 0.956862, 0.917647, 0.894117, 0.870588, 0.915686, 0.788235,
  0.749020, 0.690196, 0.654902, 0.619608, 0.552941, 0.513725, 0.490196, 0.458824, 0.392157, 0.356863,
  0.341176, 0.278431, 0.254902, 0.227451, 0.188235, 0.164706, 0.152941, 0.125490, 0.109804, 0.098039,
  0.074510, 0.062745
);

in GsOut {
    noperspective vec2 uv;   // -1..1
    flat vec3 vs_center;
    flat vec3 vs_half_u;
    flat vec3 vs_half_v;
    flat vec3 vs_normal;
    flat vec3 albedo_rgb;
} fs_in;

layout(location = 0) out vec4 accumulated_colors;       // rgb = Σ(color*w), a = Σw
layout(location = 1) out vec3 accumulated_normals;      // Σ(normal*w)
layout(location = 2) out vec3 accumulated_vs_positions; // Σ(pos_vs*w)

// minimaler Stabilitätswert
const float kEpsMin = 1e-6;

void main()
{
    // runder Footprint
    vec2 uv = fs_in.uv;
    if (dot(uv, uv) > 1.0) discard;

    // VS-Position (für Outputs)
    vec3 pos_vs = fs_in.vs_center + fs_in.vs_half_u * uv.x + fs_in.vs_half_v * uv.y;

    // *** WICHTIG: Tiefe direkt aus HW ***
    float z_this  = gl_FragCoord.z;
    float z_scene = texelFetch(depth_texture, ivec2(gl_FragCoord.xy), 0).r;

    // schmale “Oberflächennah”-Bandbreite (pixelabhängig), aber ohne explizites epsilon:
    // fwidth(gl_FragCoord.z) gibt dir die lokale Tiefe-Variation; depth_range skaliert sie.
    float band = max(kEpsMin, fwidth(z_this)) * depth_range;

    // nur die Frontschicht + sehr nahe Beiträge (z >= z_scene - band)
    // (dein bisheriger Code nutzte z01 - z_scene > eps -> discard; das invertieren wir sauber)
    if (z_this > z_scene + band) discard;

    // Gewicht (Gaussian + flank lift)
    float r = sqrt(dot(uv, uv));
    int idx = int(clamp(round(r * 31.0), 0.0, 31.0));
    float w = mix(gaussian[idx], 1.0, flank_lift);
    w = max(0.0, w);

    accumulated_colors       = vec4(fs_in.albedo_rgb * w, w);
    accumulated_normals      = normalize(fs_in.vs_normal) * w;
    accumulated_vs_positions = pos_vs * w;
}
