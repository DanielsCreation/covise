#version 420 core  

in VertexData {
    vec3 pass_point_color;
    vec2 pass_uv_coords;
} VertexIn;

layout(location = 0) out vec4 out_color;

void main() {
    vec2 uv_coords = VertexIn.pass_uv_coords;
    
    // Kreisförmige Maske
    float dist = length(uv_coords);
    if (dist > 1.0) {
        discard;
    }

    // Elliptiche Maske
    //float ellipse = (uv_coords.x * uv_coords.x) / (a * a) + (uv_coords.y * uv_coords.y) / (b * b);
    //if (ellipse > 1.0) discard;
    
    out_color = vec4(VertexIn.pass_point_color, 1.0);
}