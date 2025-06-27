#version 420 core

// Einheitlicher Interface-Block
in VertexData {
    vec3 color;
} Fragment;

// Finaler Farb-Output
out vec4 out_color;

void main() {
    out_color = vec4(Fragment.color, 1.0);
}