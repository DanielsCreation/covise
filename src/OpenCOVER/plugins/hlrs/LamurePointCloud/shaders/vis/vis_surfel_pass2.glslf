// ---------- vis_surfel_pass2.glslf (Variante A) ----------
#version 420 core
//layout(early_fragment_tests) in;

uniform sampler2D depth_texture;      // Prepass-Tiefe
uniform mat4      projection_matrix;  // für NDC-Projektion der Surfel-Frag-Tiefe

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

// ---- Tunables (keine neuen Uniforms) ----
const float kEpsMin    = 1e-6; // Mindest-Epsilon in NDC
const float kEpsScale  = 2.0;  // fwidth-Skalierung (1.5..3.0)
const float kMaskStart = 0.90; // ab 90% Radius weicher Rand
const float kMaskEnd   = 1.00; // bei 100% Radius 0

void main() {
    // Kreis-Coverage in Screen-Space (noperspective → stabil)
    vec2  uv = fs_in.uv;
    float r2 = dot(uv, uv);
    if (r2 > 1.0) discard;

    // View-Space Fragmentposition der Surfel-Disc
    vec3 pos_vs = fs_in.vs_center + fs_in.vs_half_u * uv.x + fs_in.vs_half_v * uv.y;

    // Projizierte Tiefe in NDC [0,1]
    vec4  clip = projection_matrix * vec4(pos_vs, 1.0);
    float z01  = (clip.z / clip.w) * 0.5 + 0.5;

    // Szene-Tiefe aus Pass 1 am aktuellen Pixel (nearest, keine Filterung)
    float z_scene = texelFetch(depth_texture, ivec2(gl_FragCoord.xy), 0).r;

    // Adaptives Epsilon (lokale Ableitungen + Mindestbias)
    float eps_ndc = max(kEpsMin, fwidth(z01)) * kEpsScale;

    // Manueller LEQUAL-Test in NDC
    if (z01 > z_scene + eps_ndc) discard;

    // Analytischer Rand-Falloff (Coverage-Weight)
    float  r    = sqrt(r2);
    float  mask = 1.0 - smoothstep(kMaskStart, kMaskEnd, r);
    float  w    = mask;

    accumulated_colors       = vec4(fs_in.albedo_rgb * w, w);
    accumulated_normals      = fs_in.vs_normal * w; // vs_normal ist normiert
    accumulated_vs_positions = pos_vs * w;
}
