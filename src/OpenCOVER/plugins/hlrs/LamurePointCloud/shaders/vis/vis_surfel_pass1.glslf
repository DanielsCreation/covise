#version 420 core

layout(early_fragment_tests) in;
in GsOut {
    noperspective vec2 uv;  // -1..1
} fs_in;

void main() {
    if (dot(fs_in.uv, fs_in.uv) > 1.0)
        discard;
}
