//local
#include "LamurePointCloud.h"

//std
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <vector>
#include <algorithm>
#include <list>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <boost/regex/v4/regex.hpp>
#include <boost/regex/v4/regex_replace.hpp>

//lamure
#include <lamure/ren/config.h>
#include <lamure/ren/model_database.h>
#include <lamure/ren/cut_database.h>
#include <lamure/ren/dataset.h>
#include <lamure/ren/policy.h>

//scm::gl::program_ptr vis_xyz_shader_;
//scm::gl::program_ptr vis_xyz_pass1_shader_;
//scm::gl::program_ptr vis_xyz_pass2_shader_;
//scm::gl::program_ptr vis_xyz_pass3_shader_;
//scm::gl::program_ptr vis_xyz_lighting_shader_;
//scm::gl::program_ptr vis_xyz_pass2_lighting_shader_;
//scm::gl::program_ptr vis_xyz_pass3_lighting_shader_;
//scm::gl::program_ptr vis_xyz_qz_shader_;
//scm::gl::program_ptr vis_xyz_qz_pass1_shader_;
//scm::gl::program_ptr vis_xyz_qz_pass2_shader_;
//scm::gl::program_ptr vis_quad_shader_;
//scm::gl::program_ptr vis_line_shader_;
//scm::gl::program_ptr vis_triangle_shader_;
//scm::gl::program_ptr vis_vt_shader_;
//scm::gl::frame_buffer_ptr fbo_;
//scm::gl::texture_2d_ptr fbo_color_buffer_;
//scm::gl::texture_2d_ptr fbo_depth_buffer_;
//scm::gl::frame_buffer_ptr pass1_fbo_;
//scm::gl::frame_buffer_ptr pass2_fbo_;
//scm::gl::texture_2d_ptr pass1_depth_buffer_;
//scm::gl::texture_2d_ptr pass2_color_buffer_;
//scm::gl::texture_2d_ptr pass2_normal_buffer_;
//scm::gl::texture_2d_ptr pass2_view_space_pos_buffer_;
//scm::gl::texture_2d_ptr pass2_depth_buffer_;
//scm::gl::depth_stencil_state_ptr depth_state_disable_;
//scm::gl::depth_stencil_state_ptr depth_state_less_;
//scm::gl::depth_stencil_state_ptr depth_state_without_writing_;
//scm::gl::rasterizer_state_ptr no_backface_culling_rasterizer_state_;
//scm::gl::blend_state_ptr color_blending_state_;
//scm::gl::blend_state_ptr color_no_blending_state_;
//scm::gl::sampler_state_ptr filter_linear_;
//scm::gl::sampler_state_ptr filter_nearest_;
//scm::gl::sampler_state_ptr vt_filter_linear_;
//scm::gl::sampler_state_ptr vt_filter_nearest_;
//scm::gl::texture_2d_ptr bg_texture_;
//scm::shared_ptr<scm::gl::quad_geometry> screen_quad_;
//scm::time::accum_timer<scm::time::high_res_timer> frame_time_;

struct resource {
    uint64_t num_primitives_{ 0 };
    scm::gl::buffer_ptr buffer_;
    scm::gl::vertex_array_ptr array_;
};

//resource brush_resource_;
//resource pvs_resource_;
//std::map<uint32_t, resource> bvh_resources_;
//std::map<uint32_t, resource> sparse_resources_;
//std::map<uint32_t, resource> frusta_resources_;
//std::map<uint32_t, resource> octree_resources_;
//std::map<uint32_t, resource> image_plane_resources_;


struct input {
    float trackball_x_ = 0.f;
    float trackball_y_ = 0.f;
    scm::math::vec2i mouse_;
    scm::math::vec2i prev_mouse_;
    bool brush_mode_ = 0;
    bool brush_clear_ = 0;
    bool gui_lock_ = false;
    lamure::ren::camera::mouse_state mouse_state_;
    bool keys_[3] = { 0, 0, 0 };
};
input input_;


struct gui {
    bool selection_settings_{ false };
    bool view_settings_{ false };
    bool visual_settings_{ false };
    bool provenance_settings_{ false };
    scm::math::mat4f ortho_matrix_;
};
gui gui_;


struct xyz {
    scm::math::vec3f pos_;
    uint8_t r_;
    uint8_t g_;
    uint8_t b_;
    uint8_t a_;
    float rad_;
    scm::math::vec3f nml_;
};


struct vertex {
    scm::math::vec3f pos_;
    scm::math::vec2f uv_;
};


struct selection {
    int32_t selected_model_ = -1;
    int32_t selected_view_ = -1;
    std::vector<xyz> brush_;
    std::set<uint32_t> selected_views_;
    int64_t brush_end_{ 0 };
};
selection selection_;


struct provenance {
    uint32_t num_views_{ 0 };
};
std::map<uint32_t, provenance> provenance_;


struct vt_info {
    uint32_t texture_id_;
    uint16_t view_id_;
    uint16_t context_id_;
    uint64_t cut_id_;
    vt::CutUpdate* cut_update_;
    std::vector<scm::gl::texture_2d_ptr> index_texture_hierarchy_;
    scm::gl::texture_2d_ptr physical_texture_;
    scm::math::vec2ui physical_texture_size_;
    scm::math::vec2ui physical_texture_tile_size_;
    size_t size_feedback_;
    int32_t* feedback_lod_cpu_buffer_;
    uint32_t* feedback_count_cpu_buffer_;
    scm::gl::buffer_ptr feedback_lod_storage_;
    scm::gl::buffer_ptr feedback_count_storage_;
    int toggle_visualization_;
    bool enable_hierarchy_;
};
vt_info vt_;


struct settings {
    int32_t width_{ 1920 };
    int32_t height_{ 1080 };
    int32_t frame_div_{ 1 };
    int32_t vram_{ 2048 };
    int32_t ram_{ 4096 };
    int32_t upload_{ 32 };
    bool provenance_{ 0 };
    bool create_aux_resources_{ 1 };
    float near_plane_{ 0.001f };
    float far_plane_{ 1000.0f };
    float fov_{ 30.0f };
    bool splatting_{ 1 };
    bool gamma_correction_{ 1 };
    int32_t gui_{ 1 };
    int32_t travel_{ 2 };
    float travel_speed_{ 20.5f };
    int32_t max_brush_size_{ 4096 };
    bool lod_update_{ 1 };
    bool use_pvs_{ 1 };
    bool pvs_culling_{ 0 };
    float lod_point_scale_{ 1.0f };
    float aux_point_size_{ 1.0f };
    float aux_point_distance_{ 0.5f };
    float aux_point_scale_{ 1.0f };
    float aux_focal_length_{ 1.0f };
    int32_t vis_{ 0 };
    int32_t show_normals_{ 0 };
    bool show_accuracy_{ 0 };
    bool show_radius_deviation_{ 0 };
    bool show_output_sensitivity_{ 0 };
    bool show_sparse_{ 0 };
    bool show_views_{ 0 };
    bool show_photos_{ 0 };
    bool show_octrees_{ 0 };
    bool show_bvhs_{ 0 };
    bool show_pvs_{ 0 };
    int32_t channel_{ 0 };
    float lod_error_{ LAMURE_DEFAULT_THRESHOLD };
    bool enable_lighting_{ 1 };
    bool use_material_color_{ 0 };
    scm::math::vec3f material_diffuse_{ 0.6f, 0.6f, 0.6f };
    scm::math::vec4f material_specular_{ 0.4f, 0.4f, 0.4f, 1000.0f };
    scm::math::vec3f ambient_light_color_{ 0.1f, 0.1f, 0.1f };
    scm::math::vec4f point_light_color_{ 1.0f, 1.0f, 1.0f, 1.2f };
    bool heatmap_{ 0 };
    float heatmap_min_{ 0.0f };
    float heatmap_max_{ 0.05f };
    scm::math::vec3f background_color_{ LAMURE_DEFAULT_COLOR_R, LAMURE_DEFAULT_COLOR_G, LAMURE_DEFAULT_COLOR_B };
    scm::math::vec3f heatmap_color_min_{ 68.0f / 255.0f, 0.0f, 84.0f / 255.0f };
    scm::math::vec3f heatmap_color_max_{ 251.f / 255.f, 231.f / 255.f, 35.f / 255.f };
    std::string atlas_file_{ "" };
    std::string json_{ "" };
    std::string pvs_{ "" };
    std::string background_image_{ "" };
    int32_t use_view_tf_{ 0 };
    scm::math::mat4d view_tf_{ scm::math::mat4d::identity() };
    std::vector<std::string> models_;
    std::map<uint32_t, scm::math::mat4d> transforms_;
    std::map<uint32_t, std::shared_ptr<lamure::prov::octree>> octrees_;
    std::map<uint32_t, std::vector<lamure::prov::aux::view>> views_;
    std::map<uint32_t, std::string> aux_;
    std::string selection_{ "" };
    float max_radius_{ std::numeric_limits<float>::max() };
};
settings settings_;


//double fps_ = 0.0;
//uint64_t rendered_splats_ = 0;
//uint64_t rendered_nodes_ = 0;
//
//lamure::ren::Data_Provenance data_provenance_; 
//lamure::ren::camera* camera_ = nullptr;
//bool rendering_ = false;
//int32_t render_width_ = 1280;
//int32_t render_height_ = 720;
//int32_t num_models_ = 0;
//std::vector<scm::math::mat4d> model_transformations_;
//float height_divided_by_top_minus_bottom_ = 0.f;
//scm::shared_ptr<scm::gl::render_device> device_;
//scm::shared_ptr<scm::gl::render_context> context_;
//coVRShader* pointShader = coVRShaderList::instance()->get("Points");



using namespace osg;
using namespace std;
using covise::coCoviseConfig;
using vrui::coInteraction;

static FileHandler handler = { NULL, LamurePointCloudPlugin::load, LamurePointCloudPlugin::unload, "lmr" };
LamurePointCloudPlugin* LamurePointCloudPlugin::plugin = nullptr;

COVERPLUGIN(LamurePointCloudPlugin)

// Constructor
LamurePointCloudPlugin::LamurePointCloudPlugin()
    : ui::Owner("LamurePointCloud", cover->ui)
{
    printf("LamurePointCloudPlugin::LamurePointCloudPlugin() \n");
}


const LamurePointCloudPlugin* LamurePointCloudPlugin::instance() const
{
    return plugin;
}


bool LamurePointCloudPlugin::init()
{
    printf("init()\n");
    if (plugin != NULL)
        return false;
    plugin = this;
    coVRFileManager::instance()->registerFileHandler(&handler);

    lamureMenu = new ui::Menu("LamureMenu", this);
    lamureMenu->setText("Lamure");
    loadMenu = new ui::Menu(lamureMenu, "Load");

    return 1;
}

std::string const LamurePointCloudPlugin::strip_whitespace(std::string const& in_string) {
    return boost::regex_replace(in_string, boost::regex("^ +| +$|( ) +"), "$1");
}

scm::math::mat4d LamurePointCloudPlugin::load_matrix(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open transformation file: \""
            << filename << "\"\n";
        return scm::math::mat4d::identity();
    }
    scm::math::mat4d mat = scm::math::mat4d::identity();
    std::string matrix_values_string;
    std::getline(file, matrix_values_string);
    std::stringstream sstr(matrix_values_string);
    for (int i = 0; i < 16; ++i)
        sstr >> std::setprecision(16) >> mat[i];
    file.close();
    return scm::math::transpose(mat);
}


//void LamurePointCloudPlugin::load_settings(std::string const& filename, settings& settings) {
//
//    std::ifstream lmr_file(filename.c_str());
//
//    if (!lmr_file.is_open()) {
//        std::cout << "could not open lmr file" << std::endl;
//        exit(-1);
//    }
//    else {
//        lamure::model_t model_id = 0;
//
//        std::string line;
//        while (std::getline(lmr_file, line)) {
//            if (line.length() >= 2) {
//                if (line[0] == '#') {
//                    continue;
//                }
//                auto colon = line.find_first_of(':');
//                if (colon == std::string::npos) {
//                    scm::math::mat4d transform = scm::math::mat4d::identity();
//                    std::string model;
//
//                    std::istringstream line_ss(line);
//                    line_ss >> model;
//
//                    settings.models_.push_back(model);
//                    settings.transforms_[model_id] = scm::math::mat4d::identity();
//                    settings.aux_[model_id] = "";
//                    ++model_id;
//
//                }
//                else {
//
//                    std::string key = line.substr(0, colon);
//
//                    if (key[0] == '@') {
//                        auto ws = line.find_first_of(' ');
//                        uint32_t address = atoi(plugin->strip_whitespace(line.substr(1, ws - 1)).c_str());
//                        key = plugin->strip_whitespace(line.substr(ws + 1, colon - (ws + 1)));
//                        std::string value = plugin->strip_whitespace(line.substr(colon + 1));
//
//                        if (key == "tf") {
//                            settings.transforms_[address] = load_matrix(value);
//                            std::cout << "found transform for model id " << address << std::endl;
//                        }
//                        else if (key == "aux") {
//                            settings.aux_[address] = value;
//                            std::cout << "found aux data for model id " << address << std::endl;
//                        }
//                        else {
//                            std::cout << "unrecognized key: " << key << std::endl;
//                            exit(-1);
//                        }
//                        continue;
//                    }
//
//                    key = plugin->strip_whitespace(key);
//                    std::string value = plugin->strip_whitespace(line.substr(colon + 1));
//
//                    if (key == "width") {
//                        settings.width_ = std::max(atoi(value.c_str()), 64);
//                    }
//                    else if (key == "height") {
//                        settings.height_ = std::max(atoi(value.c_str()), 64);
//                    }
//                    else if (key == "frame_div") {
//                        settings.frame_div_ = std::max(atoi(value.c_str()), 1);
//                    }
//                    else if (key == "vram") {
//                        settings.vram_ = std::max(atoi(value.c_str()), 8);
//                    }
//                    else if (key == "ram") {
//                        settings.ram_ = std::max(atoi(value.c_str()), 8);
//                    }
//                    else if (key == "upload") {
//                        settings.upload_ = std::max(atoi(value.c_str()), 8);
//                    }
//                    else if (key == "near") {
//                        settings.near_plane_ = std::max(atof(value.c_str()), 0.0);
//                    }
//                    else if (key == "far") {
//                        settings.far_plane_ = std::max(atof(value.c_str()), 0.1);
//                    }
//                    else if (key == "fov") {
//                        settings.fov_ = std::max(atof(value.c_str()), 9.0);
//                    }
//                    else if (key == "splatting") {
//                        settings.splatting_ = (bool)std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "gamma_correction") {
//                        settings.gamma_correction_ = (bool)std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "gui") {
//                        settings.gui_ = std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "speed") {
//                        settings.travel_speed_ = std::min(std::max(atof(value.c_str()), 0.0), 400.0);
//                    }
//                    else if (key == "pvs_culling") {
//                        settings.pvs_culling_ = (bool)std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "use_pvs") {
//                        settings.use_pvs_ = (bool)std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "lod_point_scale") {
//                        settings.lod_point_scale_ = std::min(std::max(atof(value.c_str()), 0.0), 10.0);
//                    }
//                    else if (key == "aux_point_size") {
//                        settings.aux_point_size_ = std::min(std::max(atof(value.c_str()), 0.00001), 1.0);
//                    }
//                    else if (key == "aux_point_distance") {
//                        settings.aux_point_distance_ = std::min(std::max(atof(value.c_str()), 0.00001), 1.0);
//                    }
//                    else if (key == "aux_focal_length") {
//                        settings.aux_focal_length_ = std::min(std::max(atof(value.c_str()), 0.001), 10.0);
//                    }
//                    else if (key == "max_brush_size") {
//                        settings.max_brush_size_ = std::min(std::max(atoi(value.c_str()), 64), 1024 * 1024);
//                    }
//                    else if (key == "lod_error") {
//                        settings.lod_error_ = std::min(std::max(atof(value.c_str()), 0.0), 10.0);
//                    }
//                    else if (key == "provenance") {
//                        settings.provenance_ = (bool)std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "create_aux_resources") {
//                        settings.create_aux_resources_ = (bool)std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "show_normals") {
//                        settings.show_normals_ = std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "show_accuracy") {
//                        settings.show_accuracy_ = (bool)std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "show_radius_deviation") {
//                        settings.show_radius_deviation_ = (bool)std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "show_output_sensitivity") {
//                        settings.show_output_sensitivity_ = (bool)std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "show_sparse") {
//                        settings.show_sparse_ = (bool)std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "show_views") {
//                        settings.show_views_ = (bool)std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "show_photos") {
//                        settings.show_photos_ = (bool)std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "show_octrees") {
//                        settings.show_octrees_ = (bool)std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "show_bvhs") {
//                        settings.show_bvhs_ = (bool)std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "show_pvs") {
//                        settings.show_pvs_ = (bool)std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "channel") {
//                        settings.channel_ = std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "enable_lighting") {
//                        settings.enable_lighting_ = (bool)std::min(std::max(atoi(value.c_str()), 0), 1);
//                    }
//                    else if (key == "use_material_color") {
//                        settings.use_material_color_ = (bool)std::min(std::max(atoi(value.c_str()), 0), 1);
//                    }
//                    else if (key == "material_diffuse_r") {
//                        settings.material_diffuse_.x = std::max(atof(value.c_str()), 0.0);
//                    }
//                    else if (key == "material_diffuse_g") {
//                        settings.material_diffuse_.y = std::max(atof(value.c_str()), 0.0);
//                    }
//                    else if (key == "material_diffuse_b") {
//                        settings.material_diffuse_.z = std::max(atof(value.c_str()), 0.0);
//                    }
//                    else if (key == "material_specular_r") {
//                        settings.material_specular_.x = std::max(atof(value.c_str()), 0.0);
//                    }
//                    else if (key == "material_specular_g") {
//                        settings.material_specular_.y = std::max(atof(value.c_str()), 0.0);
//                    }
//                    else if (key == "material_specular_b") {
//                        settings.material_specular_.z = std::max(atof(value.c_str()), 0.0);
//                    }
//                    else if (key == "material_specular_exponent") {
//                        settings.material_specular_.w = std::min(std::max(atof(value.c_str()), 0.0), 10000.0);
//                    }
//                    else if (key == "ambient_light_color_r") {
//                        settings.ambient_light_color_.r = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
//                    }
//                    else if (key == "ambient_light_color_g") {
//                        settings.ambient_light_color_.g = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
//                    }
//                    else if (key == "ambient_light_color_b") {
//                        settings.ambient_light_color_.b = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
//                    }
//                    else if (key == "point_light_color_r") {
//                        settings.point_light_color_.r = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
//                    }
//                    else if (key == "point_light_color_g") {
//                        settings.point_light_color_.g = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
//                    }
//                    else if (key == "point_light_color_b") {
//                        settings.point_light_color_.b = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
//                    }
//                    else if (key == "point_light_intensity") {
//                        settings.point_light_color_.w = std::min(std::max(atof(value.c_str()), 0.0), 10000.0);
//                    }
//                    else if (key == "background_color_r") {
//                        settings.background_color_.x = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
//                    }
//                    else if (key == "background_color_g") {
//                        settings.background_color_.y = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
//                    }
//                    else if (key == "background_color_b") {
//                        settings.background_color_.z = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
//                    }
//                    else if (key == "heatmap") {
//                        settings.heatmap_ = (bool)std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "heatmap_min") {
//                        settings.heatmap_min_ = std::max(atof(value.c_str()), 0.0);
//                    }
//                    else if (key == "heatmap_max") {
//                        settings.heatmap_max_ = std::max(atof(value.c_str()), 0.0);
//                    }
//                    else if (key == "heatmap_min_r") {
//                        settings.heatmap_color_min_.x = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
//                    }
//                    else if (key == "heatmap_min_g") {
//                        settings.heatmap_color_min_.y = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
//                    }
//                    else if (key == "heatmap_min_b") {
//                        settings.heatmap_color_min_.z = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
//                    }
//                    else if (key == "heatmap_max_r") {
//                        settings.heatmap_color_max_.x = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
//                    }
//                    else if (key == "heatmap_max_g") {
//                        settings.heatmap_color_max_.y = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
//                    }
//                    else if (key == "heatmap_max_b") {
//                        settings.heatmap_color_max_.z = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
//                    }
//                    else if (key == "atlas_file") {
//                        settings.atlas_file_ = value;
//                    }
//                    else if (key == "json") {
//                        settings.json_ = value;
//                    }
//                    else if (key == "pvs") {
//                        settings.pvs_ = value;
//                    }
//                    else if (key == "selection") {
//                        settings.selection_ = value;
//                    }
//                    else if (key == "background_image") {
//                        settings.background_image_ = value;
//                    }
//                    else if (key == "use_view_tf") {
//                        settings.use_view_tf_ = std::max(atoi(value.c_str()), 0);
//                    }
//                    else if (key == "view_tf") {
//                        settings.view_tf_ = load_matrix(value);
//                    }
//                    else if (key == "max_radius") {
//                        settings.max_radius_ = std::max(atof(value.c_str()), 0.0);
//                    }
//                    else {
//                        std::cout << "unrecognized key: " << key << std::endl;
//                        exit(-1);
//                    }
//
//                    std::cout << key << " : " << value << std::endl;
//                }
//
//            }
//        }
//        lmr_file.close();
//    }
//}
//
//
//void LamurePointCloudPlugin::set_uniforms(scm::gl::program_ptr shader) {
//    shader->uniform("win_size", scm::math::vec2f(render_width_, render_height_));
//
//    shader->uniform("near_plane", settings_.near_plane_);
//    shader->uniform("far_plane", settings_.far_plane_);
//    shader->uniform("point_size_factor", settings_.lod_point_scale_);
//
//    shader->uniform("show_normals", (bool)settings_.show_normals_);
//    shader->uniform("show_accuracy", (bool)settings_.show_accuracy_);
//    shader->uniform("show_radius_deviation", (bool)settings_.show_radius_deviation_);
//    shader->uniform("show_output_sensitivity", (bool)settings_.show_output_sensitivity_);
//
//    shader->uniform("channel", settings_.channel_);
//    shader->uniform("heatmap", (bool)settings_.heatmap_);
//
//    shader->uniform("face_eye", false);
//    shader->uniform("max_radius", settings_.max_radius_);
//
//    shader->uniform("heatmap_min", settings_.heatmap_min_);
//    shader->uniform("heatmap_max", settings_.heatmap_max_);
//    shader->uniform("heatmap_min_color", settings_.heatmap_color_min_);
//    shader->uniform("heatmap_max_color", settings_.heatmap_color_max_);
//
//    if (settings_.enable_lighting_) {
//        shader->uniform("use_material_color", settings_.use_material_color_);
//        shader->uniform("material_diffuse", settings_.material_diffuse_);
//        shader->uniform("material_specular", settings_.material_specular_);
//
//        shader->uniform("ambient_light_color", settings_.ambient_light_color_);
//        shader->uniform("point_light_color", settings_.point_light_color_);
//    }
//}
//
//
//void LamurePointCloudPlugin::draw_all_models(const lamure::context_t context_id, const lamure::view_t view_id, scm::gl::program_ptr shader) {
//
//    lamure::ren::controller* controller = lamure::ren::controller::get_instance();
//    lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
//    lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
//    //lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();
//
//    if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
//        context_->bind_vertex_array(
//            controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, device_, data_provenance_));
//    }
//    else {
//        context_->bind_vertex_array(
//            controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, device_));
//    }
//    context_->apply();
//
//    for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
//        if (selection_.selected_model_ != -1) {
//            model_id = selection_.selected_model_;
//        }
//        bool draw = true;
//        if (settings_.show_sparse_ && sparse_resources_[model_id].num_primitives_ > 0) {
//            if (selection_.selected_model_ != -1) break;
//            //else continue; //don't show lod when sparse is already shown
//            else draw = false;
//        }
//        lamure::model_t m_id = controller->deduce_model_id(std::to_string(model_id));
//        lamure::ren::cut& cut = cuts->get_cut(context_id, view_id, m_id);
//        std::vector<lamure::ren::cut::node_slot_aggregate> renderable = cut.complete_set();
//        const lamure::ren::bvh* bvh = database->get_model(m_id)->get_bvh();
//        if (bvh->get_primitive() != lamure::ren::bvh::primitive_type::POINTCLOUD) {
//            if (selection_.selected_model_ != -1) break;
//            //else continue;
//            else draw = false;
//        }
//
//        //uniforms per model
//        scm::math::mat4d model_matrix = model_transformations_[model_id];
//        scm::math::mat4d projection_matrix = scm::math::mat4d(camera_->get_projection_matrix());
//        scm::math::mat4d view_matrix = camera_->get_high_precision_view_matrix();
//        scm::math::mat4d model_view_matrix = view_matrix * model_matrix;
//        scm::math::mat4d model_view_projection_matrix = projection_matrix * model_view_matrix;
//
//        shader->uniform("mvp_matrix", scm::math::mat4f(model_view_projection_matrix));
//        shader->uniform("model_matrix", scm::math::mat4f(model_matrix));
//        shader->uniform("model_view_matrix", scm::math::mat4f(model_view_matrix));
//        shader->uniform("inv_mv_matrix", scm::math::mat4f(scm::math::transpose(scm::math::inverse(model_view_matrix))));
//
//        const scm::math::mat4d viewport_scale = scm::math::make_scale(render_width_ * 0.5, render_width_ * 0.5, 0.5);
//        const scm::math::mat4d viewport_translate = scm::math::make_translation(1.0, 1.0, 1.0);
//        const scm::math::mat4d model_to_screen = viewport_scale * viewport_translate * model_view_projection_matrix;
//        shader->uniform("model_to_screen_matrix", scm::math::mat4f(model_to_screen));
//
//        //scm::math::vec4d x_unit_vec = scm::math::vec4d(1.0,0.0,0.0,0.0);
//        //float model_radius_scale = scm::math::length(scm::math::vec3d(model_matrix * x_unit_vec));
//        //shader->uniform("model_radius_scale", model_radius_scale);
//        shader->uniform("model_radius_scale", 1.f);
//
//        size_t surfels_per_node = database->get_primitives_per_node();
//        std::vector<scm::gl::boxf>const& bounding_box_vector = bvh->get_bounding_boxes();
//
//        scm::gl::frustum frustum_by_model = camera_->get_frustum_by_model(scm::math::mat4f(model_matrix));
//
//        for (auto const& node_slot_aggregate : renderable) {
//            uint32_t node_culling_result = camera_->cull_against_frustum(
//                frustum_by_model,
//                bounding_box_vector[node_slot_aggregate.node_id_]);
//
//            if (node_culling_result != 1) {
//                /*
//                if (settings_.use_pvs_ && pvs->is_activated() && settings_.pvs_culling_
//                    && !lamure::pvs::pvs_database::get_instance()->get_viewer_visibility(model_id, node_slot_aggregate.node_id_)) {
//                    continue;
//                }
//                */
//                if (settings_.show_accuracy_) {
//                    const float accuracy = 1.0 - (bvh->get_depth_of_node(node_slot_aggregate.node_id_) * 1.0) / (bvh->get_depth() - 1);
//                    shader->uniform("accuracy", accuracy);
//                }
//                if (settings_.show_radius_deviation_) {
//                    shader->uniform("average_radius", bvh->get_avg_primitive_extent(node_slot_aggregate.node_id_));
//                }
//
//                context_->apply();
//
//                if (draw) {
//                    context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST,
//                        (node_slot_aggregate.slot_id_) * (GLsizei)surfels_per_node, surfels_per_node);
//                    rendered_splats_ += surfels_per_node;
//                    ++rendered_nodes_;
//                }
//
//
//
//            }
//        }
//        if (selection_.selected_model_ != -1) {
//            break;
//        }
//    }
//}
//
//
//void LamurePointCloudPlugin::draw_brush(scm::gl::program_ptr shader) {
//
//    if (selection_.brush_end_ > 0) {
//        set_uniforms(shader);
//
//        scm::math::mat4d model_matrix = scm::math::mat4d::identity();
//        scm::math::mat4d projection_matrix = scm::math::mat4d(camera_->get_projection_matrix());
//        scm::math::mat4d view_matrix = camera_->get_high_precision_view_matrix();
//        scm::math::mat4d model_view_matrix = view_matrix * model_matrix;
//        scm::math::mat4d model_view_projection_matrix = projection_matrix * model_view_matrix;
//
//        shader->uniform("mvp_matrix", scm::math::mat4f(model_view_projection_matrix));
//        shader->uniform("model_matrix", scm::math::mat4f(model_matrix));
//        shader->uniform("model_view_matrix", scm::math::mat4f(model_view_matrix));
//        shader->uniform("inv_mv_matrix", scm::math::mat4f(scm::math::transpose(scm::math::inverse(model_view_matrix))));
//
//        shader->uniform("point_size_factor", settings_.aux_point_scale_);
//
//        shader->uniform("model_to_screen_matrix", scm::math::mat4f::identity());
//        shader->uniform("model_radius_scale", 1.f);
//
//        shader->uniform("show_normals", false);
//        shader->uniform("show_accuracy", false);
//        shader->uniform("show_radius_deviation", false);
//        shader->uniform("show_output_sensitivity", false);
//        shader->uniform("channel", 0);
//
//        shader->uniform("face_eye", false);
//
//        context_->bind_vertex_array(brush_resource_.array_);
//        context_->apply();
//        context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 0, selection_.brush_end_);
//
//    }
//}
//
//
//void LamurePointCloudPlugin::draw_resources(const lamure::context_t context_id, const lamure::view_t view_id) {
//
//    if (sparse_resources_.size() > 0) {
//        if ((settings_.show_sparse_ || settings_.show_views_) && sparse_resources_.size() > 0) {
//
//            context_->bind_program(vis_xyz_shader_);
//            context_->set_blend_state(color_no_blending_state_);
//            context_->set_depth_stencil_state(depth_state_less_);
//
//            set_uniforms(vis_xyz_shader_);
//
//            scm::math::mat4d model_matrix = scm::math::mat4d::identity();
//            scm::math::mat4d projection_matrix = scm::math::mat4d(camera_->get_projection_matrix());
//            scm::math::mat4d view_matrix = camera_->get_high_precision_view_matrix();
//            scm::math::mat4d model_view_matrix = view_matrix * model_matrix;
//            scm::math::mat4d model_view_projection_matrix = projection_matrix * model_view_matrix;
//
//            vis_xyz_shader_->uniform("mvp_matrix", scm::math::mat4f(model_view_projection_matrix));
//            vis_xyz_shader_->uniform("model_matrix", scm::math::mat4f(model_matrix));
//            vis_xyz_shader_->uniform("model_view_matrix", scm::math::mat4f(model_view_matrix));
//            vis_xyz_shader_->uniform("inv_mv_matrix", scm::math::mat4f(scm::math::transpose(scm::math::inverse(model_view_matrix))));
//
//            vis_xyz_shader_->uniform("point_size_factor", settings_.aux_point_scale_);
//
//            vis_xyz_shader_->uniform("model_to_screen_matrix", scm::math::mat4f::identity());
//            vis_xyz_shader_->uniform("model_radius_scale", 1.f);
//
//            scm::math::mat4f inv_view = scm::math::inverse(scm::math::mat4f(view_matrix));
//            scm::math::vec3f eye = scm::math::vec3f(inv_view[12], inv_view[13], inv_view[14]);
//
//            vis_xyz_shader_->uniform("eye", eye);
//            vis_xyz_shader_->uniform("face_eye", true);
//
//            vis_xyz_shader_->uniform("show_normals", false);
//            vis_xyz_shader_->uniform("show_accuracy", false);
//            vis_xyz_shader_->uniform("show_radius_deviation", false);
//            vis_xyz_shader_->uniform("show_output_sensitivity", false);
//            vis_xyz_shader_->uniform("channel", 0);
//
//            for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
//                if (selection_.selected_model_ != -1) {
//                    model_id = selection_.selected_model_;
//                }
//
//                auto s_res = sparse_resources_[model_id];
//                if (s_res.num_primitives_ > 0) {
//                    context_->bind_vertex_array(s_res.array_);
//                    context_->apply();
//
//                    uint32_t num_views = provenance_[model_id].num_views_;
//
//                    if (settings_.show_views_) {
//                        if (selection_.selected_views_.empty()) {
//                            context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 0, num_views);
//                        }
//                        else {
//                            for (const auto view : selection_.selected_views_) {
//                                context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, view, 1);
//                            }
//                        }
//                    }
//                    if (settings_.show_sparse_) {
//                        context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, num_views,
//                            s_res.num_primitives_ - num_views);
//                    }
//
//                }
//
//                if (selection_.selected_model_ != -1) {
//                    break;
//                }
//            }
//
//        }
//
//        // draw image_plane resources with vt system
//        if (settings_.show_photos_ && !settings_.atlas_file_.empty()) {
//            context_->bind_program(vis_vt_shader_);
//
//            uint64_t color_cut_id =
//                (((uint64_t)vt_.texture_id_) << 32) | ((uint64_t)vt_.view_id_ << 16) | ((uint64_t)vt_.context_id_);
//            //uint32_t max_depth_level_color = (*vt::CutDatabase::get_instance().get_cut_map())[color_cut_id]->get_atlas()->getDepth() - 1;
//
//            scm::math::mat4f view_matrix = camera_->get_view_matrix();
//            scm::math::mat4f projection_matrix = scm::math::mat4f(camera_->get_projection_matrix());
//
//            vis_vt_shader_->uniform("model_view_matrix", view_matrix);
//            vis_vt_shader_->uniform("projection_matrix", projection_matrix);
//
//            vis_vt_shader_->uniform("physical_texture_dim", vt_.physical_texture_size_);
//            //vis_vt_shader_->uniform("max_level", max_depth_level_color);
//            vis_vt_shader_->uniform("tile_size", scm::math::vec2((uint32_t)vt::VTConfig::get_instance().get_size_tile()));
//            vis_vt_shader_->uniform("tile_padding", scm::math::vec2((uint32_t)vt::VTConfig::get_instance().get_size_padding()));
//
//            vis_vt_shader_->uniform("enable_hierarchy", vt_.enable_hierarchy_);
//            vis_vt_shader_->uniform("toggle_visualization", vt_.toggle_visualization_);
//
//            for (uint32_t i = 0; i < vt_.index_texture_hierarchy_.size(); ++i) {
//                std::string texture_string = "hierarchical_idx_textures";
//                vis_vt_shader_->uniform(texture_string, i, int((i)));
//            }
//
//            vis_vt_shader_->uniform("physical_texture_array", 17);
//
//            context_->set_viewport(
//                scm::gl::viewport(scm::math::vec2ui(0, 0),
//                    scm::math::vec2ui(render_width_, render_height_)));
//
//            context_->set_depth_stencil_state(depth_state_less_);
//            context_->set_rasterizer_state(no_backface_culling_rasterizer_state_);
//            context_->set_blend_state(color_no_blending_state_);
//
//            context_->sync();
//
//            //apply_vt_cut_update();
//
//            for (uint16_t i = 0; i < vt_.index_texture_hierarchy_.size(); ++i) {
//                context_->bind_texture(vt_.index_texture_hierarchy_.at(i), vt_filter_nearest_, i);
//            }
//
//            context_->bind_texture(vt_.physical_texture_, vt_filter_linear_, 17);
//
//            context_->bind_storage_buffer(vt_.feedback_lod_storage_, 0);
//            context_->bind_storage_buffer(vt_.feedback_count_storage_, 1);
//
//            context_->apply();
//
//            for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
//                if (selection_.selected_model_ != -1) {
//                    model_id = selection_.selected_model_;
//                }
//
//                auto t_res = image_plane_resources_[model_id];
//
//                if (t_res.num_primitives_ > 0) {
//                    context_->bind_vertex_array(t_res.array_);
//                    context_->apply();
//                    if (selection_.selected_views_.empty()) {
//                        context_->draw_arrays(scm::gl::PRIMITIVE_TRIANGLE_LIST, 0, t_res.num_primitives_);
//                    }
//                    else {
//                        for (const auto view : selection_.selected_views_) {
//                            context_->draw_arrays(scm::gl::PRIMITIVE_TRIANGLE_LIST, view * 6, 6);
//                        }
//                    }
//                }
//
//                if (selection_.selected_model_ != -1) {
//                    break;
//                }
//            }
//            context_->sync();
//
//            //collect_vt_feedback();
//
//        }
//
//        if (settings_.show_views_ || settings_.show_octrees_) {
//            context_->bind_program(vis_line_shader_);
//
//            scm::math::mat4f projection_matrix = scm::math::mat4f(camera_->get_projection_matrix());
//            scm::math::mat4f view_matrix = camera_->get_view_matrix();
//            vis_line_shader_->uniform("model_matrix", scm::math::mat4f::identity());
//            vis_line_shader_->uniform("view_matrix", view_matrix);
//            vis_line_shader_->uniform("projection_matrix", projection_matrix);
//
//            for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
//                if (selection_.selected_model_ != -1) {
//                    model_id = selection_.selected_model_;
//                }
//
//                if (settings_.show_views_) {
//                    uint32_t num_views = provenance_[model_id].num_views_;
//                    auto f_res = frusta_resources_[model_id];
//                    if (f_res.num_primitives_ > 0) {
//                        context_->bind_vertex_array(f_res.array_);
//                        context_->apply();
//                        if (selection_.selected_views_.empty()) {
//                            context_->draw_arrays(scm::gl::PRIMITIVE_LINE_LIST, 0, f_res.num_primitives_);
//                        }
//                        else {
//                            for (const auto view : selection_.selected_views_) {
//                                context_->draw_arrays(scm::gl::PRIMITIVE_LINE_LIST, view * 16, 16);
//                            }
//                        }
//                    }
//                }
//
//                if (settings_.show_octrees_) {
//                    auto o_res = octree_resources_[model_id];
//                    if (o_res.num_primitives_ > 0) {
//                        context_->bind_vertex_array(o_res.array_);
//                        context_->apply();
//                        context_->draw_arrays(scm::gl::PRIMITIVE_LINE_LIST, 0, o_res.num_primitives_);
//                    }
//                }
//
//                if (selection_.selected_model_ != -1) {
//                    break;
//                }
//            }
//        }
//    }
//
//    if (settings_.show_bvhs_) {
//
//        lamure::ren::controller* controller = lamure::ren::controller::get_instance();
//        lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
//        lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
//        //lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();
//
//        context_->bind_program(vis_line_shader_);
//
//        scm::math::mat4f projection_matrix = scm::math::mat4f(camera_->get_projection_matrix());
//        scm::math::mat4f view_matrix = camera_->get_view_matrix();
//
//        vis_line_shader_->uniform("view_matrix", view_matrix);
//        vis_line_shader_->uniform("projection_matrix", projection_matrix);
//
//        for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
//            if (selection_.selected_model_ != -1) {
//                model_id = selection_.selected_model_;
//            }
//
//            bool draw = true;
//            lamure::model_t m_id = controller->deduce_model_id(std::to_string(model_id));
//            lamure::ren::cut& cut = cuts->get_cut(context_id, view_id, m_id);
//            std::vector<lamure::ren::cut::node_slot_aggregate> renderable = cut.complete_set();
//            const lamure::ren::bvh* bvh = database->get_model(m_id)->get_bvh();
//            if (bvh->get_primitive() != lamure::ren::bvh::primitive_type::POINTCLOUD) {
//                if (selection_.selected_model_ != -1) break;
//                else draw = false;
//            }
//
//            if (draw) {
//
//                //uniforms per model
//                scm::math::mat4d model_matrix = model_transformations_[model_id];
//                vis_line_shader_->uniform("model_matrix", scm::math::mat4f(model_matrix));
//
//                std::vector<scm::gl::boxf>const& bounding_box_vector = bvh->get_bounding_boxes();
//                scm::gl::frustum frustum_by_model = camera_->get_frustum_by_model(scm::math::mat4f(model_matrix));
//
//
//                auto bvh_res = bvh_resources_[model_id];
//                if (bvh_res.num_primitives_ > 0) {
//                    context_->bind_vertex_array(bvh_res.array_);
//                    context_->apply();
//
//                    for (auto const& node_slot_aggregate : renderable) {
//                        uint32_t node_culling_result = camera_->cull_against_frustum(
//                            frustum_by_model,
//                            bounding_box_vector[node_slot_aggregate.node_id_]);
//
//                        if (node_culling_result != 1) {
//                            /*
//                            if (settings_.use_pvs_ && pvs->is_activated() && settings_.pvs_culling_ && !lamure::pvs::pvs_database::get_instance()->get_viewer_visibility(model_id, node_slot_aggregate.node_id_)) {
//                                continue;
//                            }
//                            */
//                            context_->draw_arrays(scm::gl::PRIMITIVE_LINE_LIST, node_slot_aggregate.node_id_ * 24, 24);
//
//                        }
//                    }
//                }
//            }
//
//            if (selection_.selected_model_ != -1) {
//                break;
//            }
//        }
//
//    }
//
//    if (settings_.pvs_ != "" && settings_.show_pvs_) {
//        if (pvs_resource_.num_primitives_ > 0) {
//            context_->bind_program(vis_line_shader_);
//
//            scm::math::mat4f projection_matrix = scm::math::mat4f(camera_->get_projection_matrix());
//            scm::math::mat4f view_matrix = camera_->get_view_matrix();
//            vis_line_shader_->uniform("model_matrix", scm::math::mat4f::identity());
//            vis_line_shader_->uniform("view_matrix", view_matrix);
//            vis_line_shader_->uniform("projection_matrix", projection_matrix);
//
//            context_->bind_vertex_array(pvs_resource_.array_);
//            context_->apply();
//            context_->draw_arrays(scm::gl::PRIMITIVE_LINE_LIST, 0, pvs_resource_.num_primitives_);
//        }
//    }
//}



int LamurePointCloudPlugin::load(const char* filename, osg::Group* loadParent, const char* covise_key)
{
    printf("load()\n");
    assert(plugin);
    std::string lmr_file = std::string(filename);

    // aus main()
    settings_.vis_ = settings_.show_normals_ ? 1
        : settings_.show_accuracy_ ? 2
        : settings_.show_output_sensitivity_ ? 3
        : settings_.channel_ > 0 ? 3 + settings_.channel_
        : 0;

    if (settings_.provenance_ && settings_.json_ != "") {
        std::cout << "json: " << settings_.json_ << std::endl;
        data_provenance_ = lamure::ren::Data_Provenance::parse_json(settings_.json_);
        std::cout << "size of provenance: " << data_provenance_.get_size_in_bytes() << std::endl;
    }

    lamure::ren::policy* policy = lamure::ren::policy::get_instance();
    policy->set_max_upload_budget_in_mb(settings_.upload_);
    policy->set_render_budget_in_mb(settings_.vram_);
    policy->set_out_of_core_budget_in_mb(settings_.ram_);
    render_width_ = settings_.width_ / settings_.frame_div_;
    render_height_ = settings_.height_ / settings_.frame_div_;
    policy->set_window_width(settings_.width_);
    policy->set_window_height(settings_.height_);

    int num_models_ = 0;
    lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
    for (const auto& input_file : settings_.models_) {
        lamure::model_t model_id = database->add_model(input_file, std::to_string(num_models_));
        model_transformations_.push_back(settings_.transforms_[num_models_] * scm::math::mat4d(scm::math::make_translation(database->get_model(num_models_)->get_bvh()->get_translation())));
        ++num_models_;
    }

    // ab hier aus init()
    // setup device (lamure)
    //device_.reset(new scm::gl::render_device());
    //if (!device_) {
    //    std::cout << "error creating device" << std::endl;
    //}

    //// setup context (lamure)
    //context_ = device_->main_context();
    //if (!context_) {
    //    std::cout << "error creating context" << std::endl;
    //}

    //// setup camera (lamure)
    //// Kamera Position und Frustum unabhängig vom anzuzeigenden Datensatz -> gilt nicht umgekehrt (lamure)
    //auto root_bb = lamure::ren::model_database::get_instance()->get_model(0)->get_bvh()->get_bounding_boxes()[0];
    //auto root_bb_min = scm::math::mat4f(model_transformations_[0]) * root_bb.min_vertex();
    //auto root_bb_max = scm::math::mat4f(model_transformations_[0]) * root_bb.max_vertex();
    //scm::math::vec3f center = (root_bb_min + root_bb_max) / 2.f;

    //camera_ = new lamure::ren::camera(0,
    //    make_look_at_matrix(center + scm::math::vec3f(0.f, 0.1f, -0.01f), center, scm::math::vec3f(0.f, 1.f, 0.f)),
    //    length(root_bb_max - root_bb_min), false, false);
    //camera_->set_dolly_sens_(settings_.travel_speed_);

    //// nicht sicher, was diese Einstellung bezweckt. Vermutung: Verschiebung der PointCloud (Zentrierung) und dementsprechend Anpassung der Kameraview erforderlich.
    //if (settings_.use_view_tf_) {
    //    camera_->set_view_matrix(settings_.view_tf_);
    //    std::cout << "view_tf:" << std::endl;
    //    std::cout << camera_->get_high_precision_view_matrix() << std::endl;
    //    camera_->set_dolly_sens_(settings_.travel_speed_);
    //}

    //camera_->set_projection_matrix(settings_.fov_, float(settings_.width_) / float(settings_.height_), settings_.near_plane_, settings_.far_plane_);

    ////start glut_display()
    //if (rendering_) {
    //    return 1;
    //}
    //rendering_ = true;

    //lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
    //lamure::ren::controller* controller = lamure::ren::controller::get_instance();

    //// Frage: Wie können die Daten aus dem Controller geholt werden und in Vertex-Buffer_Objekte geladen werden?
    //if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
    //    controller->reset_system(data_provenance_);
    //}
    //else {
    //    controller->reset_system();
    //}
    //lamure::context_t context_id = controller->deduce_context_id(0);

    //for (lamure::model_t model_id = 0; model_id < num_models_; ++model_id) {
    //    lamure::model_t m_id = controller->deduce_model_id(std::to_string(model_id));

    //    cuts->send_transform(context_id, m_id, scm::math::mat4f(model_transformations_[m_id]));
    //    cuts->send_threshold(context_id, m_id, settings_.lod_error_);
    //    cuts->send_rendered(context_id, m_id);

    //    database->get_model(m_id)->set_transform(scm::math::mat4f(model_transformations_[m_id]));
    //}

    //lamure::view_t cam_id = controller->deduce_view_id(context_id, camera_->view_id());
    //cuts->send_camera(context_id, cam_id, *camera_);

    //std::vector<scm::math::vec3d> corner_values = camera_->get_frustum_corners();
    //double top_minus_bottom = scm::math::length((corner_values[2]) - (corner_values[0]));
    //height_divided_by_top_minus_bottom_ = lamure::ren::policy::get_instance()->window_height() / top_minus_bottom;

    //cuts->send_height_divided_by_top_minus_bottom(context_id, cam_id, height_divided_by_top_minus_bottom_);

    //if (settings_.use_pvs_) {
    //    scm::math::mat4f cm = scm::math::inverse(scm::math::mat4f(camera_->trackball_matrix()));
    //    scm::math::vec3d cam_pos = scm::math::vec3d(cm[12], cm[13], cm[14]);
    //    // pvs->set_viewer_position(cam_pos);
    //}

    //if (settings_.lod_update_) {
    //    if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
    //        controller->dispatch(context_id, device_, data_provenance_);
    //    }
    //    else {
    //        controller->dispatch(context_id, device_);
    //    }
    //}
    //lamure::view_t view_id = controller->deduce_view_id(context_id, camera_->view_id());

    //context_->set_rasterizer_state(no_backface_culling_rasterizer_state_);

    //// glut_display() -> only singlepass, no splatting
    //context_->clear_color_buffer(fbo_, 0,
    //    scm::math::vec4f(settings_.background_color_.x, settings_.background_color_.y, settings_.background_color_.z, 1.0f));
    //context_->clear_depth_stencil_buffer(fbo_);
    //context_->set_frame_buffer(fbo_);

    //auto selected_single_pass_shading_program = vis_xyz_shader_;

    //context_->bind_program(selected_single_pass_shading_program);
    //context_->set_blend_state(color_no_blending_state_);
    //context_->set_depth_stencil_state(depth_state_less_);

    //plugin->set_uniforms(selected_single_pass_shading_program);

    //context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(render_width_, render_height_)));
    //context_->apply();

    ////draw_all_models(context_id, view_id, selected_single_pass_shading_program);

    //if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
    //    context_->bind_vertex_array(
    //        controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, device_, data_provenance_));
    //}
    //else {
    //    context_->bind_vertex_array(
    //        controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, device_));
    //}
    //context_->apply();

    //rendered_splats_ = 0;
    //rendered_nodes_ = 0;

    //for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
    //    if (selection_.selected_model_ != -1) {
    //        model_id = selection_.selected_model_;
    //    }
    //    bool draw = true;
    //    if (settings_.show_sparse_ && sparse_resources_[model_id].num_primitives_ > 0) {
    //        if (selection_.selected_model_ != -1) break;
    //        //else continue; //don't show lod when sparse is already shown
    //        else draw = false;
    //    }
    //    lamure::model_t m_id = controller->deduce_model_id(std::to_string(model_id));
    //    lamure::ren::cut& cut = cuts->get_cut(context_id, view_id, m_id);
    //    std::vector<lamure::ren::cut::node_slot_aggregate> renderable = cut.complete_set();
    //    const lamure::ren::bvh* bvh = database->get_model(m_id)->get_bvh();
    //    if (bvh->get_primitive() != lamure::ren::bvh::primitive_type::POINTCLOUD) {
    //        if (selection_.selected_model_ != -1) break;
    //        //else continue;
    //        else draw = false;
    //    }

    //    //uniforms per model
    //    scm::math::mat4d model_matrix = model_transformations_[model_id];
    //    scm::math::mat4d projection_matrix = scm::math::mat4d(camera_->get_projection_matrix());
    //    scm::math::mat4d view_matrix = camera_->get_high_precision_view_matrix();
    //    scm::math::mat4d model_view_matrix = view_matrix * model_matrix;
    //    scm::math::mat4d model_view_projection_matrix = projection_matrix * model_view_matrix;

    //    selected_single_pass_shading_program->uniform("mvp_matrix", scm::math::mat4f(model_view_projection_matrix));
    //    selected_single_pass_shading_program->uniform("model_matrix", scm::math::mat4f(model_matrix));
    //    selected_single_pass_shading_program->uniform("model_view_matrix", scm::math::mat4f(model_view_matrix));
    //    selected_single_pass_shading_program->uniform("inv_mv_matrix", scm::math::mat4f(scm::math::transpose(scm::math::inverse(model_view_matrix))));

    //    const scm::math::mat4d viewport_scale = scm::math::make_scale(render_width_ * 0.5, render_width_ * 0.5, 0.5);
    //    const scm::math::mat4d viewport_translate = scm::math::make_translation(1.0, 1.0, 1.0);
    //    const scm::math::mat4d model_to_screen = viewport_scale * viewport_translate * model_view_projection_matrix;
    //    selected_single_pass_shading_program->uniform("model_to_screen_matrix", scm::math::mat4f(model_to_screen));

    //    //scm::math::vec4d x_unit_vec = scm::math::vec4d(1.0,0.0,0.0,0.0);
    //    //float model_radius_scale = scm::math::length(scm::math::vec3d(model_matrix * x_unit_vec));
    //    //shader->uniform("model_radius_scale", model_radius_scale);
    //    selected_single_pass_shading_program->uniform("model_radius_scale", 1.f);

    //    size_t surfels_per_node = database->get_primitives_per_node();
    //    std::vector<scm::gl::boxf>const& bounding_box_vector = bvh->get_bounding_boxes();

    //    scm::gl::frustum frustum_by_model = camera_->get_frustum_by_model(scm::math::mat4f(model_matrix));

        // covise-spezifisch
        plugin->LamureGroup = new osg::Group();
        plugin->LamureGroup->setName("Lamure");
        osg::Geode* geode = new osg::Geode();
        geode->setName("LamureGeode");
        plugin->LamureGroup->addChild(geode);
        cover->getObjectsRoot()->addChild(plugin->LamureGroup);
        coVRShader* pointShader = coVRShaderList::instance()->get("Points");
        osg::ref_ptr<osg::MatrixTransform> matTra = new osg::MatrixTransform();

        plugin->pointSet = new PointSet[renderable.size()];

        for (auto const& node_slot_aggregate : renderable) {
            uint32_t node_culling_result = camera_->cull_against_frustum(frustum_by_model, bounding_box_vector[node_slot_aggregate.node_id_]);

            if (node_culling_result != 1) {

                plugin->pointSet[rendered_nodes_].points = new ::Point[surfels_per_node];
                plugin->pointSet[rendered_nodes_].colors = new Color[surfels_per_node];

                context_->apply();

                if (draw) {

                    // context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, (node_slot_aggregate.slot_id_) * (GLsizei)surfels_per_node, surfels_per_node);
                    // 
                    //create drawable and geode and add to the scene (make sure the cube is not empty)
                    PointCloudGeometry* drawable = new PointCloudGeometry(&plugin->pointSet[rendered_nodes_]);
                    Geode* currentGeode = new Geode();
                    geode->addDrawable(drawable);
                    matTra->addChild(currentGeode);
                    if (pointShader != nullptr)
                    {
                        pointShader->apply(currentGeode, drawable);
                    }
                    rendered_splats_ += surfels_per_node;
                    ++rendered_nodes_;
                }
            }
        }
        if (selection_.selected_model_ != -1) {
            break;
        }
    }

    //context_->bind_program(selected_single_pass_shading_program);
    //draw_brush(selected_single_pass_shading_program);
    //draw_resources(context_id, view_id);
    // end single pass
    // 
    // quad_screen not included
    //rendering_ = false;

    //frame_time_.stop();
    //frame_time_.start();
    ////schism bug ? time::to_seconds yields milliseconds
    //if (scm::time::to_seconds(frame_time_.accumulated_duration()) > 100.0) {
    //    fps_ = 1000.0f / scm::time::to_seconds(frame_time_.average_duration());
    //    frame_time_.reset();
    //}

    //// get viewer position
    //osg::Vec3 vecBase = (cover->getViewerMat()).getTrans();

    //std::vector<xyz> points_to_upload;
    
    return 1;
}


bool update() {

    // chose context


    // get viewer position
    osg::Vec3 vecBase = (cover->getViewerMat()).getTrans();


    // fetch data for View


    // create drawable 
    PointSet* pointSet = NULL;
    LamureGeometry* drawable = new LamureGeometry(&pointSet[0]);

    // create shader

    // controll framerate

    // controll number of rendered primitives

    if (rendering_) {
        return 1;
    }
    return 1;
}

void LamurePointCloudPlugin::createGeodes(Group* parent, const std::string& filename)
{
}



void LamurePointCloudPlugin::preFrame()
{
}


int LamurePointCloudPlugin::unload(const char* filename, const char* covise_key)
{
    return 1;
}


void LamurePointCloudPlugin::readMenuConfigData(const char* menu, vector<ImageFileEntry>& menulist, ui::Group* subMenu)
{
    coCoviseConfig::ScopeEntries entries = coCoviseConfig::getScopeEntries(menu);
}

void LamurePointCloudPlugin::selectedMenuButton(ui::Element* menuItem)
{
    string filename;

    // check structures vector for pointer (if found exit)
    vector<ImageFileEntry>::iterator itEntry = pointVec.begin();
    for (; itEntry < pointVec.end(); itEntry++)
    {
        if (itEntry->fileMenuItem == menuItem)
        {
            // call the load method passing in the file name
            filename = itEntry->fileName;

            return; //exit
        }
    }
}

LamurePointCloudPlugin::~LamurePointCloudPlugin()
{
    printf("~LamurePointCloudPlugin()\n");
}

