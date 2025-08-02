#version 420 core

out vec4 FragColor;
in vec2 pos;

// Samplers for all G-Buffer textures
uniform sampler2D texture_depth;
uniform sampler2D texture_color; // Contains color in .rgb and weight in .a
uniform sampler2D texture_normal;
uniform sampler2D texture_position;

// Uniform to select what to debug
// 0: Depth, 1: Color, 2: Normal, 3: Position
uniform int debug_mode;

uniform float near_plane;
uniform float far_plane;

float linearize_depth(float depth) {
    float z = depth * 2.0 - 1.0; // back to NDC 
    return (2.0 * near_plane * far_plane) / (far_plane + near_plane - z * (far_plane - near_plane));
}

void main()
{
    // Sanity check mode: Just draw a solid color
    if (debug_mode == -1) {
        FragColor = vec4(0.0, 1.0, 0.0, 1.0); // Bright Green
        return;
    }
    // Convert from [-1, 1] to [0, 1] texture coordinates
    vec2 tex_coords = (pos.xy + 1.0) / 2.0;

    if (debug_mode == 0) { // Visualize Depth
        float depth = texture(texture_depth, tex_coords).r;
        if (depth > 0.0 && depth < 1.0) {
            FragColor = vec4(1.0, 0.0, 0.0, 1.0); // Red if depth value is valid
        } else {
            FragColor = vec4(0.0, 0.0, 1.0, 1.0); // Blue if not
        }
    }
    else if (debug_mode == 1) { // Visualize Depth (from color texture)
        float depth = texture(texture_color, tex_coords).r; // Read depth from red channel
        float linear_depth = linearize_depth(depth) / far_plane; // normalize
        FragColor = vec4(vec3(linear_depth), 1.0);
    }
    else if (debug_mode == 2) { // Visualize Normals
        vec4 color_data = texture(texture_color, tex_coords); // Need weight from color texture
        if (color_data.a > 0.001) {
            vec3 normal = texture(texture_normal, tex_coords).xyz;
            normal = normalize(normal / color_data.a); // De-accumulate
            FragColor = vec4(normal * 0.5 + 0.5, 1.0); // Remap from [-1,1] to [0,1] for viewing
        } else {
            FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        }
    }
    else if (debug_mode == 3) { // Visualize View-Space Position
        vec4 color_data = texture(texture_color, tex_coords); // Need weight
         if (color_data.a > 0.001) {
            vec3 position = texture(texture_position, tex_coords).xyz;
            position = position / color_data.a; // De-accumulate
            // Visualizing positions is tricky, just showing absolute normalized values
            FragColor = vec4(abs(normalize(position)), 1.0);
        } else {
            FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        }
    }
    else {
        FragColor = vec4(1.0, 0.0, 1.0, 1.0); // Magenta for error/unknown mode
    }
}
