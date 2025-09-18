// ---------- vis_surfel_pass1.glslf ----------
#version 420 core

#version 420 core

uniform mat4 projection_matrix;

in GsOut {
    noperspective vec2 uv;
    flat vec3 vs_center;
    flat vec3 vs_half_u;
    flat vec3 vs_half_v;
} fs_in;

void main() {
    if (dot(fs_in.uv, fs_in.uv) > 1.0) discard;
    gl_FragDepth = gl_FragCoord.z;
}
