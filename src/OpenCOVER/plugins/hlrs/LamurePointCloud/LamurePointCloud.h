//gl
#include <GL/glew.h>

//boost
#include <boost/assign/list_of.hpp>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

//lamure
#include <lamure/types.h>
#include <lamure/ren/config.h>
#include <lamure/ren/model_database.h>
#include <lamure/ren/cut_database.h>
#include <lamure/ren/dataset.h>
#include <lamure/ren/policy.h>
#include <lamure/ren/controller.h>
#include <lamure/pvs/pvs_database.h>
#include <lamure/ren/ray.h>
#include <lamure/prov/prov_aux.h>
#include <lamure/prov/octree.h>
#include <lamure/vt/VTConfig.h>
#include <lamure/vt/ren/CutDatabase.h>
#include <lamure/vt/ren/CutUpdate.h>
#include <lamure/vt/pre/AtlasFile.h>

//schism
#include <scm/core.h>
#include <scm/core/math.h>
#include <scm/core/io/tools.h>
#include <scm/core/pointer_types.h>
#include <scm/gl_core/gl_core_fwd.h>
#include <scm/gl_util/primitives/primitives_fwd.h>
#include <scm/core/platform/platform.h>
#include <scm/core/utilities/platform_warning_disable.h>
#include <scm/gl_util/primitives/quad.h>
#include <scm/gl_util/font/font_face.h>
#include <scm/gl_util/font/text.h>
#include <scm/gl_util/font/text_renderer.h>
#include <scm/core/time/accum_timer.h>
#include <scm/core/time/high_res_timer.h>
#include <scm/gl_util/data/imaging/texture_loader.h>
#include <scm/gl_util/primitives/geometry.h>
#include <scm/gl_util/primitives/box.h>

//std
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <vector>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <algorithm>
#include <list>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <ctype.h>
#include <math.h>


#include <cover/coVRConfig.h>
#include <cover/coVRPlugin.h>
#include <cover/coVRPluginSupport.h>
#include <cover/coVRFileManager.h>
#include <cover/coVRMSController.h>
#include <cover/coVRTui.h>

#include <util/coTypes.h>


class LamurePointCloudPlugin : public coVRPlugin
{
private:
    static LamurePointCloudPlugin* plugin;

protected:
    bool rendering_ = false;
    int32_t render_width_ = 1280;
    int32_t render_height_ = 720;

    int32_t num_models_ = 0;
    std::vector<scm::math::mat4d> model_transformations_;

    float height_divided_by_top_minus_bottom_ = 0.f;

    lamure::ren::camera* camera_ = nullptr;

    scm::shared_ptr<scm::gl::render_device> device_;
    scm::shared_ptr<scm::gl::render_context> context_;

    scm::gl::program_ptr vis_xyz_shader_;
    scm::gl::program_ptr vis_xyz_pass1_shader_;
    scm::gl::program_ptr vis_xyz_pass2_shader_;
    scm::gl::program_ptr vis_xyz_pass3_shader_;

    scm::gl::program_ptr vis_xyz_lighting_shader_;
    scm::gl::program_ptr vis_xyz_pass2_lighting_shader_;
    scm::gl::program_ptr vis_xyz_pass3_lighting_shader_;

    scm::gl::program_ptr vis_xyz_qz_shader_;
    scm::gl::program_ptr vis_xyz_qz_pass1_shader_;
    scm::gl::program_ptr vis_xyz_qz_pass2_shader_;

    scm::gl::program_ptr vis_quad_shader_;
    scm::gl::program_ptr vis_line_shader_;
    scm::gl::program_ptr vis_triangle_shader_;
    scm::gl::program_ptr vis_vt_shader_;

    scm::gl::frame_buffer_ptr fbo_;
    scm::gl::texture_2d_ptr fbo_color_buffer_;
    scm::gl::texture_2d_ptr fbo_depth_buffer_;

    scm::gl::frame_buffer_ptr pass1_fbo_;
    scm::gl::texture_2d_ptr pass1_depth_buffer_;
    scm::gl::frame_buffer_ptr pass2_fbo_;
    scm::gl::texture_2d_ptr pass2_color_buffer_;
    scm::gl::texture_2d_ptr pass2_normal_buffer_;
    scm::gl::texture_2d_ptr pass2_view_space_pos_buffer_;
    scm::gl::texture_2d_ptr pass2_depth_buffer_;

    scm::gl::depth_stencil_state_ptr depth_state_disable_;
    scm::gl::depth_stencil_state_ptr depth_state_less_;
    scm::gl::depth_stencil_state_ptr depth_state_without_writing_;
    scm::gl::rasterizer_state_ptr no_backface_culling_rasterizer_state_;

    scm::gl::blend_state_ptr color_blending_state_;
    scm::gl::blend_state_ptr color_no_blending_state_;

    scm::gl::sampler_state_ptr filter_linear_;
    scm::gl::sampler_state_ptr filter_nearest_;

    scm::gl::sampler_state_ptr vt_filter_linear_;
    scm::gl::sampler_state_ptr vt_filter_nearest_;

    scm::gl::texture_2d_ptr bg_texture_;

    struct resource {
        uint64_t num_primitives_{ 0 };
        scm::gl::buffer_ptr buffer_;
        scm::gl::vertex_array_ptr array_;
    };

    resource brush_resource_;
    resource pvs_resource_;
    std::map<uint32_t, resource> bvh_resources_;
    std::map<uint32_t, resource> sparse_resources_;
    std::map<uint32_t, resource> frusta_resources_;
    std::map<uint32_t, resource> octree_resources_;
    std::map<uint32_t, resource> image_plane_resources_;

    scm::shared_ptr<scm::gl::quad_geometry> screen_quad_;
    scm::time::accum_timer<scm::time::high_res_timer> frame_time_;

    double fps_ = 0.0;
    uint64_t rendered_splats_ = 0;
    uint64_t rendered_nodes_ = 0;

    lamure::ren::Data_Provenance data_provenance_;

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

public:
    LamurePointCloudPlugin();
    ~LamurePointCloudPlugin();
    bool init();

    static LamurePointCloudPlugin* instance() { return plugin; };

    static int loadVIS(const char* filename, osg::Group* parent, const char* ck = "");
    static int unloadVIS(const char* filename, const char* ck = "");

    void load_settings(std::string const& vis_file_name, settings& settings);




};