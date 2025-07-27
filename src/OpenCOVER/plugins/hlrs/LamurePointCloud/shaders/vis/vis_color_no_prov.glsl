// vis_color_prov.glsl

INCLUDE ../common/heatmapping/wavelength_to_rainbow.glsl
INCLUDE ../common/heatmapping/colormap.glsl

uniform bool  show_normals;
uniform bool  show_accuracy;
uniform bool  show_radius_deviation;
uniform bool  show_output_sensitivity;

uniform float accuracy;
uniform float average_radius;

vec3 get_output_sensitivity_color(float screen_size) {
    float min_screen_size = 0.0;
    float max_screen_size = 10.0;
    float val = clamp(screen_size, min_screen_size, max_screen_size);
    return data_value_to_rainbow(val, min_screen_size, max_screen_size);
}

vec3 get_color(in vec3 position,
               in vec3 normal,
               in vec3 base_color,
               in float radius,
               in float screen_size) {

    vec3  view_color = vec3(0.0);

    if (show_normals) {
        vec3 n = normal;
        if (n.z < 0.0) n *= -1.0;
        view_color = n * 0.5 + 0.5;
    }
    else if (show_output_sensitivity) {
        view_color = get_output_sensitivity_color(screen_size);
    }
    else if (show_radius_deviation) {
        float max_fac  = 2.0;
        float safe_avg = max(1e-8, average_radius);
        view_color = vec3(min(max_fac, radius / safe_avg) / max_fac);
    }
    else {
        view_color = base_color;
    }

    if (show_accuracy) {
        view_color += vec3(accuracy, 0.0, 0.0);
    }

    return view_color;
}