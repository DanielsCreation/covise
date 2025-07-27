#version 420 core

in FS_IN {
    vec3  pass_point_color;
    vec2  pass_uv_coords;
    vec3  pass_world_pos;
    vec3  pass_normal_ws;
    float pass_radius_ws;
    float pass_screen_size;
} fsIn;

layout(location = 0) out vec4 out_color;

INCLUDE vis_color_no_prov.glsl

void main() {
    if (length(fsIn.pass_uv_coords) > 1.0)
        discard;

    vec3 col = get_color(
        fsIn.pass_world_pos,
        fsIn.pass_normal_ws,
        fsIn.pass_point_color,
        fsIn.pass_radius_ws,
        fsIn.pass_screen_size
    );
    out_color = vec4(col, 1.0);
}
