#version 420 core

uniform sampler2D depth_texture;      // Prepass depth (GL_NEAREST)
uniform mat4      projection_matrix;  // P für NDC-Z-Vergleich

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

// NDC-Depth-Epsilon (keine Uniforms → Konstanten)
const float kEpsMin   = 1e-6;
const float kEpsScale = 2.0;  // 1.5..3.0

void main()
{
    // Analytische Disc-Maske
    vec2  uv = fs_in.uv;
    float r2 = dot(uv, uv);
    if (r2 > 1.0) discard;

    // View-Space Position des Fragments auf der Surfel-Disc
    vec3 pos_vs = fs_in.vs_center + fs_in.vs_half_u * uv.x + fs_in.vs_half_v * uv.y;

    // Projizierte Z in [0,1]
    vec4  clip = projection_matrix * vec4(pos_vs, 1.0);
    float z01  = (clip.z / clip.w) * 0.5 + 0.5;

    // Szene-Z aus Prepass (texelFetch, keine Filterung)
    float z_scene = texelFetch(depth_texture, ivec2(gl_FragCoord.xy), 0).r;

    // Adaptives Epsilon gegen Z-Quantisierung / Ableitungen
    float eps_ndc = max(kEpsMin, fwidth(z01)) * kEpsScale;

    // Manuelle Tiefenprüfung (LEQUAL + Epsilon)
    if (z01 > z_scene + eps_ndc) discard;

    // Pixelpräziser Rand-Falloff (nur für Überlappungs-Blending zwischen Surfeln)
    float r  = sqrt(r2);
    float dr = fwidth(r);                       // ~1 Pixel in r-Einheiten
    float w  = 1.0 - smoothstep(1.0 - 2.0*dr, 1.0, r);
    // optional etwas schärfer: w *= w;

    // Premultiplied Akkumulation
    accumulated_colors       = vec4(fs_in.albedo_rgb * w, w);
    accumulated_normals      = fs_in.vs_normal * w;
    accumulated_vs_positions = pos_vs * w;
}