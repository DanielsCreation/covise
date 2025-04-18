
#define GLFW_EXPOSE_NATIVE_WIN32
//local
#include "LamurePointCloud.h"
#include "gl_util.h"
#include "osg_util.h"
#include <osg/StateSet>

#include <lamure/imgui.h>
#include <lamure/imgui_internal.h>
#include <lamure/imgui_impl_glfw_gl3.h>

// std
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <vector>
#include <algorithm>
#include <list>
#include <iosfwd>
#include <sstream>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <winbase.h>
#include <mutex>
#include <filesystem>

//boost
#include <boost/assign/list_of.hpp>
#include <boost/regex.hpp>
#include <boost/thread.hpp>

//schism
#include <scm/time.h>
//#include <scm/core.h>
#include <scm/core/math.h>
#include <scm/core/io/tools.h>
#include <scm/core/pointer_types.h>
//#include <scm/core/platform/platform.h>
//#include <scm/core/utilities/platform_warning_disable.h>

//lamure
#include <lamure/pvs/pvs_database.h>
#include <lamure/prov/prov_aux.h>
#include <lamure/vt/pre/AtlasFile.h>
#include <lamure/prov/octree.h>
#include <lamure/vt/VTConfig.h>
#include <lamure/vt/ren/CutDatabase.h>
#include <lamure/vt/ren/CutUpdate.h>

#include <config/coConfigConstants.h>
#include <config/coConfigLog.h>
#include <config/coConfig.h>
#include <config/coConfigString.h>
#include <config/coConfigEntryString.h>

#include <C:\src\covise\src\3rdparty\deskvox/virvo/virvo/vvtoolshed.h>
#include <cover/ui/SelectionList.h>
#include <cover/coVRStatsDisplay.h>
#include <cover/VRSceneGraph.h>
#include "cover/OpenCOVER.h"
#include <cover/VRWindow.h>
#include <cover/VRViewer.h>
#include <cover/coHud.h>

#include "Points.h"
#include <osgViewer/GraphicsWindow>
#include <osgViewer/Renderer>
#include <osg/PolygonMode>
#include <osgGA/EventQueue>

#include <util/coExport.h>
#include <PluginUtil/FeedbackManager.h>
#include <PluginUtil/ModuleInteraction.h>
#include <OpenVRUI/coButtonInteraction.h>
#include <config/CoviseConfig.h>

#include <lamure/utils.h>
#include "lamure/ren/data_provenance.h"
#include "lamure/ren/controller.h"
#include <lamure/config.h>
#include <lamure/ren/cut.h>
#include <scm/gl_util/primitives/primitives_fwd.h>
#include <scm/gl_util/primitives.h>
#include <scm/gl_core/buffer_objects/scoped_buffer_map.h>

#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>


//#include <C:/src/covise/src/3rdparty/deskvox/virvo/virvo/gl/util.cpp>

//static void GLClearError()
//{
//    while (glGetError() != GL_NO_ERROR);
//}
//
//#define ASSERT(x) if (!(x)) __debugbreak();
//#define GLCall(x) GLClearError();\
//    x;\
//    ASSERT(GLLogCall(#x,__FILE__, __LINE__))

#ifdef __cplusplus
extern "C" {
#endif

	__declspec(dllexport) DWORD NvOptimusEnablement = 1;
	__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;

#ifdef __cplusplus
}
#endif

COVERPLUGIN(LamurePointCloudPlugin)
LamurePointCloudPlugin* LamurePointCloudPlugin::plugin = nullptr;

static FileHandler handler =
{ NULL,
  LamurePointCloudPlugin::loadLMR,
  LamurePointCloudPlugin::unloadLMR,
  "lmr"
};

LamurePointCloudPlugin::LamurePointCloudPlugin() : ui::Owner("LamurePointCloud", cover->ui)
{
	coVRFileManager::instance()->registerFileHandler(&handler);
	plugin = this;
}

std::string vis_surfel_shader_vs_source;
std::string vis_surfel_shader_fs_source;
std::string vis_line_bb_vs_source;
std::string vis_line_bb_fs_source;
std::string vis_quad_vs_source;
std::string vis_quad_fs_source;
std::string vis_line_vs_source;
std::string vis_line_fs_source;
std::string vis_triangle_vs_source;
std::string vis_triangle_fs_source;
std::string vis_plane_vs_source;
std::string vis_plane_fs_source;
std::string vis_text_vs_source;
std::string vis_text_fs_source;
std::string vis_vt_vs_source;
std::string vis_vt_fs_source;
std::string vis_xyz_vs_source;
std::string vis_xyz_gs_source;
std::string vis_xyz_fs_source;
std::string vis_xyz_pass1_vs_source;
std::string vis_xyz_pass1_gs_source;
std::string vis_xyz_pass1_fs_source;
std::string vis_xyz_pass2_vs_source;
std::string vis_xyz_pass2_gs_source;
std::string vis_xyz_pass2_fs_source;
std::string vis_xyz_pass3_vs_source;
std::string vis_xyz_pass3_fs_source;
std::string vis_xyz_qz_vs_source;
std::string vis_xyz_qz_pass1_vs_source;
std::string vis_xyz_qz_pass2_vs_source;
std::string vis_box_vs_source;
std::string vis_box_gs_source;
std::string vis_box_fs_source;

/* parsed with optional lighting code */
std::string vis_xyz_vs_lighting_source;
std::string vis_xyz_gs_lighting_source;
std::string vis_xyz_fs_lighting_source;
std::string vis_xyz_pass2_vs_lighting_source;
std::string vis_xyz_pass2_gs_lighting_source;
std::string vis_xyz_pass2_fs_lighting_source;
std::string vis_xyz_pass3_vs_lighting_source;
std::string vis_xyz_pass3_fs_lighting_source;

std::string shader_root_path = LAMURE_SHADERS_DIR;
std::string font_root_path = LAMURE_FONTS_DIR;

static osg::Vec3f vecConv3F(scm::math::vec3f& v);
static osg::Vec3d vecConv3D(scm::math::vec3d& v);
static osg::Vec4f vecConv4F(scm::math::vec4f& v);
static osg::Vec4d vecConv4D(scm::math::vec4d& v);

static scm::math::vec3f vecConv3F(osg::Vec3f& v);
static scm::math::vec3d vecConv3D(osg::Vec3d& v);
static scm::math::vec4f vecConv3F(osg::Vec4f& v);
static scm::math::vec4d vecConv3D(osg::Vec4d& v);

static osg::Matrixf matConv4F(scm::math::mat4f& m);
static osg::Matrixd matConv4D(scm::math::mat4d& m);
static scm::math::mat4f matConv4F(osg::Matrixd& m);
static scm::math::mat4d matConv4D(osg::Matrixd& m);


osg::Vec3f vecConv3F(scm::math::vec3f& v) {
	osg::Vec3f vec_osg = osg::Vec3f(v[0], v[1], v[2]);
	return vec_osg;
}
osg::Vec3d vecConv3D(scm::math::vec3d& v) {
	osg::Vec3d vec_osg = osg::Vec3d(v[0], v[1], v[2]);
	return vec_osg;
}
osg::Vec4f vecConv4F(scm::math::vec4f& v) {
	osg::Vec4f vec_osg = osg::Vec4f(v[0], v[1], v[2], v[3]);
	return vec_osg;
}
osg::Vec4d vecConv4D(scm::math::vec4d& v) {
	osg::Vec4d vec_osg = osg::Vec4d(v[0], v[1], v[2], v[3]);
	return vec_osg;
}
scm::math::vec3f vecConv3F(osg::Vec3f& v) {
	scm::math::vec3f vec_scm = scm::math::vec3f(v[0], v[1], v[2]);
	return vec_scm;
}
scm::math::vec3d vecConv3D(osg::Vec3d& v) {
	scm::math::vec3d vec_scm = scm::math::vec3d(v[0], v[1], v[2]);
	return vec_scm;
}
scm::math::vec4f vecConv4F(osg::Vec4f& v) {
	scm::math::vec4f vec_scm = scm::math::vec4f(v[0], v[1], v[2], v[3]);
	return vec_scm;
}
scm::math::vec4d vecConv4D(osg::Vec4d& v) {
	scm::math::vec4d vec_scm = scm::math::vec4d(v[0], v[1], v[2], v[3]);
	return vec_scm;
}
osg::Matrixf matConv4F(scm::math::mat4f& m) {
	osg::Matrix mat_osg = osg::Matrixf(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15]);
	return mat_osg;
}
osg::Matrixd matConv4D(scm::math::mat4d& m) {
	osg::Matrixd mat_osg = osg::Matrixd(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15]);
	return mat_osg;
}
scm::math::mat4f matConv4F(osg::Matrixd& m) {
	scm::math::mat4f mat_scm = scm::math::mat4f(m(0, 0), m(0, 1), m(0, 2), m(0, 3), m(1, 0), m(1, 1), m(1, 2), m(1, 3), m(2, 0), m(2, 1), m(2, 2), m(2, 3), m(3, 0), m(3, 1), m(3, 2), m(3, 3));
	return mat_scm;
}
scm::math::mat4d matConv4D(osg::Matrixd& m) {
	scm::math::mat4d mat_scm = scm::math::mat4d(m(0, 0), m(0, 1), m(0, 2), m(0, 3), m(1, 0), m(1, 1), m(1, 2), m(1, 3), m(2, 0), m(2, 1), m(2, 2), m(2, 3), m(3, 0), m(3, 1), m(3, 2), m(3, 3));
	return mat_scm;
}
scm::math::mat4f matConv4F(const osg::Matrixd& m) {
	scm::math::mat4f mat_scm = scm::math::mat4f(m(0, 0), m(0, 1), m(0, 2), m(0, 3), m(1, 0), m(1, 1), m(1, 2), m(1, 3), m(2, 0), m(2, 1), m(2, 2), m(2, 3), m(3, 0), m(3, 1), m(3, 2), m(3, 3));
	return mat_scm;
}
scm::math::mat4d matConv4D(const osg::Matrixd& m) {
	scm::math::mat4d mat_scm = scm::math::mat4d(m(0, 0), m(0, 1), m(0, 2), m(0, 3), m(1, 0), m(1, 1), m(1, 2), m(1, 3), m(2, 0), m(2, 1), m(2, 2), m(2, 3), m(3, 0), m(3, 1), m(3, 2), m(3, 3));
	return mat_scm;
}

FT_Library ft_;
FT_Face face_;
static GLuint       g_FontTexture = 0;
static const osg::GraphicsContext::Traits* traits = coVRConfig::instance()->windows[0].context->getTraits();
static lamure::context_t lmr_ctx;
boost::mutex m;
std::mutex gl_state_mutex;
uint32_t render_width_;
uint32_t render_height_;
lamure::ren::Data_Provenance data_provenance_;
float height_divided_by_top_minus_bottom_ = 0.0f;
uint32_t num_models_ = 0;

scm::gl::render_device_ptr      device_;
scm::gl::render_context_ptr     context_;
scm::gl::quad_geometry_ptr      screen_quad_;

scm::gl::text_renderer_ptr      text_renderer_;
scm::gl::text_ptr               renderable_text_;


lmr_camera* lamure_camera_;
lamure::ren::camera* scm_camera_;
osg::ref_ptr<osg::Camera>   osg_camera_;
osg::ref_ptr<osg::Camera>   rtt_camera_;

scm::gl::program_ptr vis_surfel_shader_;
scm::gl::program_ptr vis_line_bb_shader_; 
scm::gl::program_ptr vis_text_shader_;
scm::gl::program_ptr vis_box_shader_;
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
scm::gl::program_ptr vis_plane_shader_;
scm::gl::program_ptr vis_line_shader_;
scm::gl::program_ptr vis_triangle_shader_;
scm::gl::program_ptr vis_vt_shader_;

scm::gl::frame_buffer_ptr fbo_;
scm::gl::texture_2d_ptr fbo_color_buffer_;
scm::gl::texture_2d_ptr fbo_depth_buffer_;
scm::gl::frame_buffer_ptr pass1_fbo_;
scm::gl::frame_buffer_ptr pass2_fbo_;
scm::gl::frame_buffer_ptr pass3_fbo_;
scm::gl::texture_2d_ptr pass1_depth_buffer_;
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

scm::time::accum_timer<scm::time::high_res_timer> frame_time_;

struct settings {
	int32_t width_{ traits->width };
	int32_t height_{ traits->height };
	int32_t frame_div_{ 1 };
	int32_t vram_{ 1024 };
	int32_t ram_{ 4096 };
	int32_t upload_{ 32 };
	bool provenance_{ 1 };
	bool create_aux_resources_{ 1 };
	double near_plane_{ 0.001f };
	double far_plane_{ 1000.0f };
	float fov_{ 30.0f };
	bool splatting_{ 0 };  // multipass is not working yet (bc of storage buffers/rtt mode?)
	bool gamma_correction_{ 0 };
	int32_t gui_{ 1 };
	int32_t travel_{ 2 };
	float travel_speed_{ 20.5f };
	int32_t max_brush_size_{ 4096 };
	bool lod_update_{ 1 };
	bool use_pvs_{ 0 };
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
	bool enable_lighting_{ 0 };
	bool use_material_color_{ 1 };
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
	int32_t use_view_tf_{ 1 };
	scm::math::mat4d view_tf_ { scm::math::mat4d::identity() };
	scm::math::vec3d model_tl_ { scm::math::vec3f::zero() };
	scm::math::mat4d model_rot_ { scm::math::mat4d::identity() };
	std::vector<std::string> models_;
	std::map<uint32_t, scm::math::mat4d> transforms_;
	std::map<uint32_t, std::shared_ptr<lamure::prov::octree>> octrees_;
	std::map<uint32_t, std::vector<lamure::prov::aux::view>> views_;
	std::map<uint32_t, std::string> aux_;
	std::string selection_{ "" };
	float max_radius_{ std::min(std::numeric_limits<float>::max(), 1.0f) };
	float scale_radius_{ 1.0f };
	std::vector<float> bvh_color_{ 1.0f, 1.0f, 0.0f, 1.0f };
	std::vector<float> frustum_color_{ 0.0f, 0.0f, 0.0f, 1.0f };
};
settings settings_;

struct gl_info {
	scm::math::mat4f	mvp_ { scm::math::mat4f::identity()};
	HGLRC				oc_ { NULL };
};
gl_info gl_info_;


struct pcl_resource {
	GLuint vbo_ = 0;
	GLuint ibo_ = 0;
	GLuint vao_ = 0;
	GLuint program_ = 0;
	
};
pcl_resource pcl_resource_;


struct box_resource {
	GLuint vbo_ = 0;
	GLuint ibo_ = 0;
	GLuint vao_ = 0;
	GLuint program_ = 0;
	std::vector<std::vector<float>> vertices_;
	std::array<unsigned short, 24> idx_ = {
		0, 1, 2, 3, 4, 5, 6, 7,
		0, 2, 1, 3, 4, 6, 5, 7,
		0, 4, 1, 5, 2, 6, 3, 7,
	};
	GLuint mvp_location_;
	GLuint color_location_;
};
box_resource box_resource_;


struct plane_resource {
	GLuint vbo_{ 0 };
	GLuint ibo_{ 0 };
	GLuint vao_{ 0 };
	GLuint program_{ 0 };
	std::array<unsigned short, 6> idx_ = {
		1,3,7,5,1,7
	};
};
plane_resource plane_resource_;


struct sphere_resource {
	GLuint vbo_{ 0 };
	GLuint vao_{ 0 };
	GLuint ibo_{ 0 };
	std::array<float, 3> points = {
		0.0f,0.0f,0.0f
	};
};
sphere_resource sphere_resource_;


struct coord_recourse {
	GLuint vao_ = 0;
	GLuint vbo_ = 0;
	GLuint ibo_ = 0;
	GLuint program_ = 0;
	std::array<float, 12> vertices_ = {
		  0.f,   0.f,   0.f, 
		100.f,   0.f,   0.f, // X-Achse
		  0.f, 100.f,   0.f, // Y-Achse
		  0.f,   0.f, 100.f  // Z-Achse
	};
	std::array<unsigned short, 6> idx_ = {
	0, 1,
	0, 2,
	0, 3,
	};
};
coord_recourse coord_resource_;


struct frustum_resource {
	GLuint vao_ = 0;
	GLuint vbo_ = 0;
	GLuint ibo_ = 0;
	GLuint program_ = 0;
	std::array<float, 24> vertices_;
	std::array<unsigned short, 24> idx_ = {
	0, 1, 2, 3, 4, 5, 6, 7,
	0, 2, 1, 3, 4, 6, 5, 7,
	0, 4, 1, 5, 2, 6, 3, 7,
	};
};
frustum_resource frustum_resource_;


struct text_resource {
	GLuint vao_{ 0 };
	GLuint vbo_{ 0 };
	GLuint program_{ 0 };
	GLuint atlas_texture_{ 0 };
	std::string text_;
	size_t num_vertices_{ 0 };
};
text_resource text_resource_;


struct Character {
	int SizeX;
	int SizeY;
	int BearingX;
	int BearingY;
	GLuint Advance;
	float TexCoordX;
	float TexCoordY;
	float TexCoordWidth;
	float TexCoordHeight;
};
std::map<char, Character> characters_;


struct resource {
	uint64_t num_primitives_{ 0 };
	scm::gl::buffer_ptr buffer_;
	scm::gl::vertex_array_ptr array_;
	std::vector<std::vector<float>> corners_;
};
resource brush_resource_;
resource pvs_resource_;

// singels are declared locally
std::map<uint32_t, resource> bvh_res_;
std::map<uint32_t, resource> octree_res_;
std::map<uint32_t, resource> sparse_res_;
std::map<uint32_t, resource> image_plane_res_;


struct model_info {
	std::vector<scm::math::mat4d> model_transformations_;
	std::vector<scm::math::vec3f> root_bb_min;
	std::vector<scm::math::vec3f> root_bb_max;
	std::vector<scm::math::vec3f> root_center;
	scm::math::vec3f models_min;
	scm::math::vec3f models_max;
	scm::math::vec3d models_center;
};
model_info model_info_;


struct gui {
	bool selection_settings_{ true };
	bool view_settings_{ true };
	bool visual_settings_{ true };
	bool provenance_settings_{ true };
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


struct render_info {
	uint64_t rendered_splats_{ 0 };
	uint64_t rendered_nodes_{ 0 };
	uint64_t rendered_bounding_boxes_{ 0 };
	float fps_{0.0f};
};
render_info render_info_;


struct GLState {
	GLenum last_active_texture;
	GLint last_program;
	GLint last_texture;
	GLint last_sampler;
	GLint last_array_buffer;
	GLint last_element_array_buffer;
	GLint last_vertex_array;
	GLint last_polygon_mode[2];
	GLint last_viewport[4];
	GLint last_scissor_box[4];
	GLenum last_blend_src_rgb;
	GLenum last_blend_dst_rgb;
	GLenum last_blend_src_alpha;
	GLenum last_blend_dst_alpha;
	GLenum last_blend_equation_rgb;
	GLenum last_blend_equation_alpha;
	GLboolean last_enable_blend;
	GLboolean last_enable_cull_face;
	GLboolean last_enable_depth_test;
	GLboolean last_enable_scissor_test;
	GLint last_cull_face_mode;
	GLint last_front_face;
	GLint last_depth_func;
	GLboolean last_depth_mask;
	GLboolean last_color_writemask[4];
	GLboolean last_stencil_test;
	GLint last_stencil_func;
	GLint last_stencil_ref;
	GLint last_stencil_value_mask;
	GLint last_stencil_fail;
	GLint last_stencil_pass_depth_fail;
	GLint last_stencil_pass_depth_pass;
	GLfloat last_line_width;
	GLboolean last_enable_line_smooth;
	GLboolean last_enable_line_stipple;
	GLint last_line_stipple_factor;
	GLushort last_line_stipple_pattern;
	GLfloat last_point_size;
	GLboolean last_enable_point_smooth;
};


int CheckGLError(char* file, int line)
{
	GLenum glErr;
	int    retCode = 0;

	glErr = glGetError();
	while (glErr != GL_NO_ERROR)
	{
		const GLubyte* sError = gluErrorString(glErr);

		if (sError)
			cerr << "GL Error #" << glErr << "(" << gluErrorString(glErr) << ") " << " in File " << file << " at line: " << line << endl;
		else
			cerr << "GL Error #" << glErr << " (no message available)" << " in File " << file << " at line: " << line << endl;

		retCode = 1;
		glErr = glGetError();
	}
	return retCode;
}
#define CHECK_GL_ERROR() CheckGLError(__FILE__, __LINE__)

#define BACKUP_GL_STATE(state) do { \
    glGetIntegerv(GL_ACTIVE_TEXTURE, (GLint*)&(state).last_active_texture); \
    glGetIntegerv(GL_CURRENT_PROGRAM, &(state).last_program); \
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &(state).last_texture); \
    glGetIntegerv(GL_SAMPLER_BINDING, &(state).last_sampler); \
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &(state).last_array_buffer); \
    glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, &(state).last_element_array_buffer); \
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &(state).last_vertex_array); \
    glGetIntegerv(GL_POLYGON_MODE, (state).last_polygon_mode); \
    glGetIntegerv(GL_VIEWPORT, (state).last_viewport); \
    glGetIntegerv(GL_SCISSOR_BOX, (state).last_scissor_box); \
    glGetIntegerv(GL_BLEND_SRC_RGB, (GLint*)&(state).last_blend_src_rgb); \
    glGetIntegerv(GL_BLEND_DST_RGB, (GLint*)&(state).last_blend_dst_rgb); \
    glGetIntegerv(GL_BLEND_SRC_ALPHA, (GLint*)&(state).last_blend_src_alpha); \
    glGetIntegerv(GL_BLEND_DST_ALPHA, (GLint*)&(state).last_blend_dst_alpha); \
    glGetIntegerv(GL_BLEND_EQUATION_RGB, (GLint*)&(state).last_blend_equation_rgb); \
    glGetIntegerv(GL_BLEND_EQUATION_ALPHA, (GLint*)&(state).last_blend_equation_alpha); \
    (state).last_enable_blend = glIsEnabled(GL_BLEND); \
    (state).last_enable_cull_face = glIsEnabled(GL_CULL_FACE); \
    (state).last_enable_depth_test = glIsEnabled(GL_DEPTH_TEST); \
    (state).last_enable_scissor_test = glIsEnabled(GL_SCISSOR_TEST); \
    glGetIntegerv(GL_CULL_FACE_MODE, &(state).last_cull_face_mode); \
    glGetIntegerv(GL_FRONT_FACE, &(state).last_front_face); \
    glGetIntegerv(GL_DEPTH_FUNC, &(state).last_depth_func); \
    glGetBooleanv(GL_DEPTH_WRITEMASK, &(state).last_depth_mask); \
    glGetBooleanv(GL_COLOR_WRITEMASK, (state).last_color_writemask); \
    (state).last_stencil_test = glIsEnabled(GL_STENCIL_TEST); \
    glGetIntegerv(GL_STENCIL_FUNC, &(state).last_stencil_func); \
    glGetIntegerv(GL_STENCIL_REF, &(state).last_stencil_ref); \
    glGetIntegerv(GL_STENCIL_VALUE_MASK, &(state).last_stencil_value_mask); \
    glGetIntegerv(GL_STENCIL_FAIL, &(state).last_stencil_fail); \
    glGetIntegerv(GL_STENCIL_PASS_DEPTH_FAIL, &(state).last_stencil_pass_depth_fail); \
    glGetIntegerv(GL_STENCIL_PASS_DEPTH_PASS, &(state).last_stencil_pass_depth_pass); \
    glGetFloatv(GL_LINE_WIDTH, &(state).last_line_width); \
    (state).last_enable_line_smooth = glIsEnabled(GL_LINE_SMOOTH); \
    (state).last_enable_line_stipple = glIsEnabled(GL_LINE_STIPPLE); \
    glGetIntegerv(GL_LINE_STIPPLE_REPEAT, &(state).last_line_stipple_factor); \
    glGetIntegerv(GL_LINE_STIPPLE_PATTERN, (GLint*)&(state).last_line_stipple_pattern); \
    glGetFloatv(GL_POINT_SIZE, &(state).last_point_size); \
    (state).last_enable_point_smooth = glIsEnabled(GL_POINT_SMOOTH); \
} while(0)

#define RESTORE_GL_STATE(state) do { \
    glUseProgram((state).last_program); \
    glBindTexture(GL_TEXTURE_2D, (state).last_texture); \
    glBindSampler(0, (state).last_sampler); \
    glActiveTexture((state).last_active_texture); \
    glBindVertexArray((state).last_vertex_array); \
    glBindBuffer(GL_ARRAY_BUFFER, (state).last_array_buffer); \
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, (state).last_element_array_buffer); \
    glBlendEquationSeparate((state).last_blend_equation_rgb, (state).last_blend_equation_alpha); \
    glBlendFuncSeparate((state).last_blend_src_rgb, (state).last_blend_dst_rgb, \
                        (state).last_blend_src_alpha, (state).last_blend_dst_alpha); \
    if ((state).last_enable_blend) glEnable(GL_BLEND); else glDisable(GL_BLEND); \
    if ((state).last_enable_cull_face) glEnable(GL_CULL_FACE); else glDisable(GL_CULL_FACE); \
    if ((state).last_enable_depth_test) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST); \
    if ((state).last_enable_scissor_test) glEnable(GL_SCISSOR_TEST); else glDisable(GL_SCISSOR_TEST); \
    glPolygonMode(GL_FRONT_AND_BACK, (state).last_polygon_mode[0]); \
    glViewport((state).last_viewport[0], (state).last_viewport[1], \
               (GLsizei)(state).last_viewport[2], (GLsizei)(state).last_viewport[3]); \
    glScissor((state).last_scissor_box[0], (state).last_scissor_box[1], \
              (GLsizei)(state).last_scissor_box[2], (GLsizei)(state).last_scissor_box[3]); \
    glColorMask((state).last_color_writemask[0], (state).last_color_writemask[1], \
                (state).last_color_writemask[2], (state).last_color_writemask[3]); \
    glCullFace((state).last_cull_face_mode); \
    glFrontFace((state).last_front_face); \
    glDepthFunc((state).last_depth_func); \
    glDepthMask((state).last_depth_mask); \
    if ((state).last_stencil_test) glEnable(GL_STENCIL_TEST); else glDisable(GL_STENCIL_TEST); \
    glStencilFunc((state).last_stencil_func, (state).last_stencil_ref, (state).last_stencil_value_mask); \
    glStencilOp((state).last_stencil_fail, (state).last_stencil_pass_depth_fail, (state).last_stencil_pass_depth_pass); \
    glLineWidth((state).last_line_width); \
    if ((state).last_enable_line_smooth) glEnable(GL_LINE_SMOOTH); else glDisable(GL_LINE_SMOOTH); \
    if ((state).last_enable_line_stipple) { \
        glEnable(GL_LINE_STIPPLE); \
        glLineStipple((GLint)(state).last_line_stipple_factor, (GLushort)(state).last_line_stipple_pattern); \
    } else { \
        glDisable(GL_LINE_STIPPLE); \
    } \
    glPointSize((state).last_point_size); \
    if ((state).last_enable_point_smooth) glEnable(GL_POINT_SMOOTH); else glDisable(GL_POINT_SMOOTH); \
} while(0)

#define DRAW_ALL_MODELS(shader_) \
if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {  \
    context_->bind_vertex_array(controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, device_, data_provenance_)); \
} \
else { context_->bind_vertex_array(controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, device_)); } \
rendered_splats_ = 0; \
rendered_nodes_ = 0; \
for (uint16_t model_id = 0; model_id < num_models_; ++model_id) { \
    lamure::context_t context_id = controller->deduce_context_id(lmr_ctx); \
    lamure::ren::cut& cut = cuts->get_cut(context_id, lmr_ctx, model_id); \
    std::vector<lamure::ren::cut::node_slot_aggregate> renderable = cut.complete_set(); \
    const lamure::ren::bvh* bvh = database->get_model(model_id)->get_bvh(); \
    size_t surfels_per_node = database->get_primitives_per_node(); \
    std::vector<scm::gl::boxf>const& bounding_box_vector = bvh->get_bounding_boxes(); \
    scm::math::mat4d model_matrix = model_info_.model_transformations_[model_id] * translationMatrix * rotationMatrix; \
    scm::gl::frustum frustum_ = scm_camera_->get_frustum_(); \
    scm::math::mat4d projection_matrix = scm::math::mat4d(gl_projection_matrix_d); \
    scm::math::mat4d view_matrix = gl_view_matrix_d; \
    scm::math::mat4d model_view_matrix = view_matrix * model_matrix; \
    scm::math::mat4d model_view_projection_matrix = projection_matrix * model_view_matrix; \
    shader_->uniform("mvp_matrix", scm::math::mat4f(model_view_projection_matrix)); \
    shader_->uniform("model_matrix", scm::math::mat4f(model_matrix)); \
    shader_->uniform("model_view_matrix", scm::math::mat4f(model_view_matrix)); \
    shader_->uniform("inv_mv_matrix", scm::math::mat4f(scm::math::transpose(scm::math::inverse(model_view_matrix)))); \
    const scm::math::mat4d viewport_scale = scm::math::make_scale(traits->width * 0.5, traits->height * 0.5, 0.5); \
    const scm::math::mat4d viewport_translate = scm::math::make_translation(1.0, 1.0, 1.0); \
    const scm::math::mat4d model_to_screen = viewport_scale * viewport_translate; \
    shader_->uniform("model_to_screen_matrix", scm::math::mat4f(model_to_screen)); \
    shader_->uniform("model_radius_scale", settings_.scale_radius_); \
    shader_->uniform("max_radius", settings_.max_radius_); \
    context_->apply_uniform_buffer_bindings(); \
    context_->bind_program(shader_); \
    context_->set_blend_state(color_no_blending_state_); \
    context_->set_depth_stencil_state(depth_state_less_); \
    context_->apply_state_objects(); \
    context_->apply_program(); \
    bool draw = true; \
    for (auto const& node_slot_aggregate : renderable) { \
        uint32_t node_culling_result = scm_camera_->cull_against_frustum(frustum_, bounding_box_vector[node_slot_aggregate.node_id_]); \
        if (node_culling_result != 1) { \
            if (draw) { \
                context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, (node_slot_aggregate.slot_id_) * (GLsizei)surfels_per_node, surfels_per_node); \
                rendered_splats_ += surfels_per_node; \
                ++rendered_nodes_; \
            } \
        } \
    } \
} \


void printChildNodes(osg::Node * node, int depth = 0) {
	if (!node) return;
	for (int i = 0; i < depth; ++i) {
		std::cout << "  ";
	}
	std::cout << "- " << node->className();
	if (node->getName().empty()) {
		std::cout << " (unnamed)";
	}
	else {
		std::cout << " (" << node->getName() << ")";
	}
	std::cout << std::endl;

	osg::Group* group = node->asGroup();
	if (group) {
		for (unsigned int i = 0; i < group->getNumChildren(); ++i) {
			printChildNodes(group->getChild(i), depth + 1);
		}
	}
}


void LamurePointCloudPlugin::printNodePath(osg::ref_ptr<osg::Node> pointer) {
	osg::NodePathList npl = pointer->getParentalNodePaths();
	int path_size = npl.size();
	std::cout << pointer->className() << " at level " << path_size << std::endl;
	if (path_size > 0) {
		for (int j = 0; j < npl[0].size(); j++) {
			std::cout << "[" << j << "] " << npl[0][j]->className() << ":  " << npl[0][j]->getName() << std::endl;
		}
		std::cout << "" << std::endl;
	}
	std::cout << "" << std::endl;
}


void APIENTRY openglCallbackFunction(GLenum source,
	GLenum type,
	GLuint id,
	GLenum severity,
	GLsizei length,
	const GLchar* message,
	const void* userParam)
{
	std::cerr << "---------------------" << std::endl;
	std::cerr << "Debug message (" << id << "): " << message << std::endl;
	std::cerr << "Source: " << source << ", Type: " << type << ", Severity: " << severity << std::endl;
	std::cerr << "---------------------" << std::endl;
}




std::string vec3ToString(const osg::Vec3& v) {
	std::ostringstream oss;
	oss << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
	return oss.str();
}


std::string matrixdToString(const osg::Matrixd& m) {
	std::ostringstream oss;
	oss << "[";
	for (int i = 0; i < 4; ++i) {
		oss << "(";
		for (int j = 0; j < 4; ++j) {
			oss << m(i, j);
			if (j < 3) oss << ", ";
		}
		oss << ")";
		if (i < 3) oss << ", ";
	}
	oss << "]";
	return oss.str();
}


float* gl_mat_to_array(GLdouble mat[16]) {

	scm::math::mat4d gl_mat = scm::math::mat4d(mat[0], mat[1], mat[2], mat[3], mat[4], mat[5], mat[6], mat[7], mat[8], mat[9], mat[10], mat[11], mat[12], mat[13], mat[14], mat[15]);
	float* gl_array = scm::math::mat4f(gl_mat).data_array;
	return gl_array;
}


scm::math::mat4d gl_mat(GLdouble mat[16]) {
	scm::math::mat4d gl_mat = scm::math::mat4d(mat[0], mat[1], mat[2], mat[3], mat[4], mat[5], mat[6], mat[7], mat[8], mat[9], mat[10], mat[11], mat[12], mat[13], mat[14], mat[15]);
	return gl_mat;
}


void printGraphicsContextAttributes(const osg::GraphicsContext* gc)
{
	std::cerr << "---------------------" << std::endl;
	printCurrentContext();
	if (!gc)
	{
		std::cout << "GraphicsContext is null." << std::endl;
		return;
	}
	std::cout << "GraphicsContext Pointer: " << gc << std::endl;
	std::cout << "Default FBO ID: " << gc->getDefaultFboId() << std::endl;

	const osg::GraphicsContext::Traits* traits = gc->getTraits();
	if (!traits)
	{
		std::cout << "GraphicsContext Traits are null." << std::endl;
		return;
	}
	std::ostringstream oss;
	oss << "x: " << traits->x << std::endl;
	oss << "y: " << traits->y << std::endl;
	oss << "width: " << traits->width << std::endl;
	oss << "height: " << traits->height << std::endl;
	oss << "Window Decoration: " << (traits->windowDecoration ? "true" : "false") << std::endl;
	oss << "Supports Resize: " << (traits->supportsResize ? "true" : "false") << std::endl;
	oss << "Red Bits: " << traits->red << std::endl;
	oss << "Green Bits: " << traits->green << std::endl;
	oss << "Blue Bits: " << traits->blue << std::endl;
	oss << "Alpha Bits: " << traits->alpha << std::endl;
	oss << "Depth Bits: " << traits->depth << std::endl;
	oss << "Stencil Bits: " << traits->stencil << std::endl;
	oss << "Sample Buffers: " << traits->sampleBuffers << std::endl;
	oss << "Samples: " << traits->samples << std::endl;
	oss << "Pbuffer: " << (traits->pbuffer ? "true" : "false") << std::endl;
	oss << "Quad Buffer Stereo: " << (traits->quadBufferStereo ? "true" : "false") << std::endl;
	oss << "Double Buffer: " << (traits->doubleBuffer ? "true" : "false") << std::endl;
	oss << "VSync: " << (traits->vsync ? "true" : "false") << std::endl;
	oss << "Window Name: " << traits->windowName << std::endl;
	oss << "Windowing System Preference: " << traits->windowingSystemPreference << std::endl;

	if (traits->sharedContext.valid())
	{
		oss << "Shared Context Pointer: " << traits->sharedContext.get() << std::endl;
	}
	else
	{
		oss << "Shared Context Pointer: none" << std::endl;
	}

	std::cout << oss.str() << std::endl;

	std::cerr << "---------------------" << std::endl;
}

void printScreenStruct(const screenStruct& s)
{
	std::ostringstream oss;
	oss << "hsize = " << s.hsize << std::endl
		<< "vsize = " << s.vsize << std::endl
		<< "configuredHsize = " << s.configuredHsize << std::endl
		<< "configuredVsize = " << s.configuredVsize << std::endl
		<< "xyz = " << vec3ToString(s.xyz) << std::endl
		<< "hpr = " << vec3ToString(s.hpr) << std::endl
		<< "name = \"" << s.name << "\"" << std::endl
		<< "render = " << (s.render ? "true" : "false") << std::endl
		<< "lTan = " << s.lTan << std::endl
		<< "rTan = " << s.rTan << std::endl
		<< "tTan = " << s.tTan << std::endl
		<< "bTan = " << s.bTan;
	std::cout << oss.str() << std::endl;
}

void printChannelStruct(const channelStruct& ch)
{
	std::ostringstream oss;
	oss << "name = \"" << ch.name << "\"" << std::endl
		<< "PBONum = " << ch.PBONum << std::endl
		<< "viewportNum = " << ch.viewportNum << std::endl
		<< "screenNum = " << ch.screenNum << std::endl
		<< "camera = " << ch.camera.get() << std::endl
		<< "ds = " << ch.ds << std::endl
		<< "stereo = " << (ch.stereo ? "true" : "false") << std::endl
		<< "stereoMode = " << ch.stereoMode << std::endl
		<< "fixedViewer = " << (ch.fixedViewer ? "true" : "false") << std::endl
		<< "stereoOffset = " << ch.stereoOffset << std::endl
		<< "leftView = " << matrixdToString(ch.leftView) << std::endl
		<< "rightView = " << matrixdToString(ch.rightView) << std::endl
		<< "leftProj = " << matrixdToString(ch.leftProj) << std::endl
		<< "rightProj = " << matrixdToString(ch.rightProj);
	std::cout << oss.str() << std::endl;
}

void printPBOStruct(const PBOStruct& pbo)
{
	std::ostringstream oss;
	oss << "PBOsx = " << pbo.PBOsx << std::endl
		<< "PBOsy = " << pbo.PBOsy << std::endl
		<< "windowNum = " << pbo.windowNum << std::endl
		<< "renderTargetTexture = " << pbo.renderTargetTexture.get();
	std::cout << oss.str() << std::endl;
}

void printAngleStruct(const angleStruct& a)
{
	std::ostringstream oss;
	oss << "analogInput = " << a.analogInput << std::endl
		<< "cmin = " << a.cmin << std::endl
		<< "cmax = " << a.cmax << std::endl
		<< "minangle = " << a.minangle << std::endl
		<< "maxangle = " << a.maxangle << std::endl
		<< "screen = " << a.screen << std::endl
		<< "value = " << static_cast<const void*>(a.value) << std::endl
		<< "hpr = " << a.hpr;
	std::cout << oss.str() << std::endl;
}

void printWindowStruct(const windowStruct& w)
{
	std::ostringstream oss;
	oss << "ox = " << w.ox << std::endl
		<< "oy = " << w.oy << std::endl
		<< "sx = " << w.sx << std::endl
		<< "sy = " << w.sy << std::endl
		<< "context = " << w.context.get() << std::endl
		<< "window = " << w.window.get() << std::endl
		<< "pipeNum = " << w.pipeNum << std::endl
		<< "name = \"" << w.name << "\"" << std::endl
		<< "decoration = " << (w.decoration ? "true" : "false") << std::endl
		<< "resize = " << (w.resize ? "true" : "false") << std::endl
		<< "stereo = " << (w.stereo ? "true" : "false") << std::endl
		<< "embedded = " << (w.embedded ? "true" : "false") << std::endl
		<< "pbuffer = " << (w.pbuffer ? "true" : "false") << std::endl
		<< "doublebuffer = " << (w.doublebuffer ? "true" : "false") << std::endl
		<< "swapGroup = " << w.swapGroup << std::endl
		<< "swapBarrier = " << w.swapBarrier << std::endl
		<< "screenNum = " << w.screenNum << std::endl
		<< "type = \"" << w.type << "\"" << std::endl
		<< "windowPlugin = " << w.windowPlugin;
	std::cout << oss.str() << std::endl;
}

void printViewportStruct(const viewportStruct& vp)
{
	std::ostringstream oss;
	oss << "mode = ";
	switch (vp.mode)
	{
	case viewportStruct::Channel: oss << "Channel"; break;
	case viewportStruct::PBO: oss << "PBO"; break;
	case viewportStruct::TridelityML: oss << "TridelityML"; break;
	case viewportStruct::TridelityMV: oss << "TridelityMV"; break;
	default: oss << "Unknown"; break;
	}
	oss << std::endl
		<< "window = " << vp.window << std::endl
		<< "PBOnum = " << vp.PBOnum << std::endl
		<< "sourceXMin = " << vp.sourceXMin << std::endl
		<< "sourceYMin = " << vp.sourceYMin << std::endl
		<< "sourceXMax = " << vp.sourceXMax << std::endl
		<< "sourceYMax = " << vp.sourceYMax << std::endl
		<< "viewportXMin = " << vp.viewportXMin << std::endl
		<< "viewportYMin = " << vp.viewportYMin << std::endl
		<< "viewportXMax = " << vp.viewportXMax << std::endl
		<< "viewportYMax = " << vp.viewportYMax << std::endl
		<< "distortMeshName = \"" << vp.distortMeshName << "\"" << std::endl
		<< "blendingTextureName = \"" << vp.blendingTextureName << "\"" << std::endl
		<< "pbos = [";
	for (size_t i = 0; i < vp.pbos.size(); ++i)
	{
		oss << vp.pbos[i];
		if (i < vp.pbos.size() - 1)
			oss << ", ";
	}
	oss << "]";
	std::cout << oss.str() << std::endl;
}

void printBlendingTextureStruct(const blendingTextureStruct& bt)
{
	std::ostringstream oss;
	oss << "window = " << bt.window << std::endl
		<< "viewportXMin = " << bt.viewportXMin << std::endl
		<< "viewportYMin = " << bt.viewportYMin << std::endl
		<< "viewportXMax = " << bt.viewportXMax << std::endl
		<< "viewportYMax = " << bt.viewportYMax << std::endl
		<< "blendingTextureName = \"" << bt.blendingTextureName << "\"";
	std::cout << oss.str() << std::endl;
}

void printPipeStruct(const pipeStruct& p)
{
	std::ostringstream oss;
	oss << "x11DisplayNum = " << p.x11DisplayNum << std::endl
		<< "x11ScreenNum = " << p.x11ScreenNum << std::endl
		<< "x11DisplayHost = \"" << p.x11DisplayHost << "\"" << std::endl
		<< "useDISPLAY = " << (p.useDISPLAY ? "true" : "false");
	std::cout << oss.str() << std::endl;
}

void printCoVRConfigOverview()
{
	coVRConfig* config = coVRConfig::instance();
	if (!config)
	{
		std::cerr << "coVRConfig::instance() is null." << std::endl;
		return;
	}

	std::cout << "Configured with "
		<< config->numScreens() << " screens, " << std::endl
		<< config->numWindows() << " windows, " << std::endl
		<< config->numChannels() << " channels, " << std::endl
		<< config->numViewports() << " viewports, " << std::endl
		<< config->numBlendingTextures() << " blending textures, " << std::endl
		<< config->numPBOs() << " PBOs." << std::endl << std::endl;

	std::cout << "=== Screens ===" << std::endl;
	for (size_t i = 0; i < config->screens.size(); ++i)
	{
		std::cout << "Screen[" << i << "]: ";
		printScreenStruct(config->screens[i]);
	}
	std::cout << std::endl;

	std::cout << "=== Windows ===" << std::endl;
	for (size_t i = 0; i < config->windows.size(); ++i)
	{
		std::cout << "Window[" << i << "]: ";
		printWindowStruct(config->windows[i]);
	}
	std::cout << std::endl;

	std::cout << "=== Channels ===" << std::endl;
	for (size_t i = 0; i < config->channels.size(); ++i)
	{
		std::cout << "Channel[" << i << "]: ";
		printChannelStruct(config->channels[i]);
	}
	std::cout << std::endl;

	std::cout << "=== Viewports ===" << std::endl;
	for (size_t i = 0; i < config->viewports.size(); ++i)
	{
		std::cout << "Viewport[" << i << "]: ";
		printViewportStruct(config->viewports[i]);
	}
	std::cout << std::endl;

	std::cout << "=== Blending Textures ===" << std::endl;
	for (size_t i = 0; i < config->blendingTextures.size(); ++i)
	{
		std::cout << "BlendingTexture[" << i << "]: ";
		printBlendingTextureStruct(config->blendingTextures[i]);
	}
	std::cout << std::endl;

	std::cout << "=== PBOs ===" << std::endl;
	for (size_t i = 0; i < config->PBOs.size(); ++i)
	{
		std::cout << "PBO[" << i << "]: ";
		printPBOStruct(config->PBOs[i]);
	}
	std::cout << std::endl;

	std::cout << "=== Pipes ===" << std::endl;
	for (size_t i = 0; i < config->pipes.size(); ++i)
	{
		std::cout << "Pipe[" << i << "]: ";
		printPipeStruct(config->pipes[i]);
	}
	std::cout << std::endl;
}


void printAllGraphicsContextsAndWindows()
{
	// 1. Alle registrierten GraphicsContexts aus OSG abrufen.
	osg::GraphicsContext::GraphicsContexts contexts = osg::GraphicsContext::getAllRegisteredGraphicsContexts();
	std::cout << "=== Registered GraphicsContexts (" << contexts.size() << ") ===" << std::endl;
	for (size_t i = 0; i < contexts.size(); ++i)
	{
		std::cout << "Context[" << i << "]:" << std::endl;
		printGraphicsContextAttributes(contexts[i]);
		std::cout << std::endl;
	}

	// 2. Fensterinformationen aus coVRConfig ausgeben.
	// Hier gehen wir davon aus, dass coVRConfig::instance()->windows ein Container (z.B. std::vector) mit Fensterstrukturen ist,
	// die mindestens einen Namen, eine Kontextzeiger und Auflösungsinformationen enthalten.
	std::cout << "=== Registered Windows ===" << std::endl;
	// Prüfen, ob coVRConfig verfügbar ist:
	if (coVRConfig::instance())
	{
		const auto& windows = coVRConfig::instance()->windows; // Angenommen, windows ist ein std::vector<WindowInfo>
		std::cout << "Anzahl Fenster: " << windows.size() << std::endl;
		for (size_t i = 0; i < windows.size(); ++i)
		{
			std::cout << "Window[" << i << "]:" << std::endl;
			std::cout << "  Name: " << windows[i].name << std::endl;
			std::cout << "  Context Pointer: " << windows[i].context << std::endl;
			//std::cout << "  Resolution: " << windows[i].width << "x" << windows[i].height << std::endl;
			// Falls weitere Parameter vorhanden sind, können diese hier ergänzt werden.
			std::cout << std::endl;
		}
	}
	else
	{
		std::cout << "coVRConfig ist nicht verfügbar." << std::endl;
	}
}


void printCandidateVAO(GLuint candidate)
{
	if (!glIsVertexArray(candidate))
		return;

	std::cout << "VAO Handle: " << candidate << std::endl;

	GLint oldVAO = 0;
	glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &oldVAO);

	glBindVertexArray(candidate);

	GLint currentEAB = 0;
	glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, &currentEAB);
	//std::cout << "  Element Array Buffer Binding: " << currentEAB << std::endl;

	GLint maxAttribs = 0;
	glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, &maxAttribs);

	int activeAttribCount = 0;
	for (GLint i = 0; i < maxAttribs; ++i)
	{
		GLint enabled = 0;
		glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &enabled);
		if (enabled)
			activeAttribCount++;
	}
	std::cout << "  Active Vertex Attributes: " << activeAttribCount << "/" << maxAttribs << std::endl;

	for (GLint i = 0; i < maxAttribs; ++i)
	{
		GLint enabled = 0;
		glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &enabled);
		if (enabled)
		{
			GLint size = 0, type = 0, stride = 0, bufferBinding = 0;
			glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_SIZE, &size);
			glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_TYPE, &type);
			glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_STRIDE, &stride);
			glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_BUFFER_BINDING, &bufferBinding);
			void* pointer = nullptr;
			glGetVertexAttribPointerv(i, GL_VERTEX_ATTRIB_ARRAY_POINTER, &pointer);

			std::ostringstream oss;
			oss << "  Attribute " << i << ":" << std::endl;
			oss << "    Size: " << size << std::endl;
			oss << "    Type: " << type << std::endl;
			oss << "    Stride: " << stride << std::endl;
			oss << "    Offset Pointer: " << pointer << std::endl;
			oss << "    Element Array Buffer Binding: " << currentEAB << std::endl;
			oss << "    Array Buffer Binding: " << bufferBinding << std::endl;
			std::cout << oss.str();
		}
	}
	glBindVertexArray(oldVAO);
	std::cout << std::endl;
}


void printAllExistingVAOs(GLuint maxID = 10000)
{
	std::cout << "=== Overview of all VAOs ===" << std::endl;
	printCurrentContext();
	GLint activeVAO = 0;
	glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &activeVAO);
	std::cout << "Active VAO: " << activeVAO << std::endl << std::endl;
	for (GLuint i = 1; i <= maxID; ++i)
	{
		if (glIsVertexArray(i))
		{
			printCandidateVAO(i);
		}
	}
}


void wait_for_opengl_context() {
	unsigned int max_wait_time_ms = 10000;
	auto start_time = std::chrono::steady_clock::now();
	bool context_ready = false;
	while (!context_ready) {
		if (wglGetCurrentContext() != nullptr) {
			context_ready = true;
			break;
		}
		std::cout << "Warte auf OpenGL-Kontext..." << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Kürzere Wartezeit für bessere Reaktionsfähigkeit
		// Prüfen, ob die maximale Wartezeit überschritten wurde
		auto current_time = std::chrono::steady_clock::now();
		auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
		if (elapsed_ms >= max_wait_time_ms) {
			std::cerr << "Fehler: OpenGL-Kontext nicht verfügbar nach " << max_wait_time_ms << " Millisekunden." << std::endl;
			return;
		}
	}

	std::cout << "OpenGL-Kontext ist jetzt bereit!" << std::endl;
	// Stellen Sie sicher, dass ein gültiger OpenGL-Kontext vorhanden ist
	const GLubyte* version = glGetString(GL_VERSION);
	const GLubyte* renderer = glGetString(GL_RENDERER);
	const GLubyte* vendor = glGetString(GL_VENDOR);
	const GLubyte* shadingLanguageVersion = glGetString(GL_SHADING_LANGUAGE_VERSION);

	// Überprüfen, ob die Rückgabewerte gültig sind
	if (!version || !renderer || !vendor || !shadingLanguageVersion) {
		std::cerr << "Fehler beim Abrufen von OpenGL-Informationen. Ist der OpenGL-Kontext aktiv?" << std::endl;
		return;
	}

	// Profilmaske abrufen
	GLint profileMask = 0;
	glGetIntegerv(GL_CONTEXT_PROFILE_MASK, &profileMask);

	GLenum err = glGetError();
	if (err != GL_NO_ERROR) {
		std::cerr << "OpenGL-Fehler beim Abrufen der Profilmaske: " << err << std::endl;
	}

	// Ausgabe der Informationen
	std::cout << "OpenGL-Version: " << version << std::endl;
	std::cout << "Renderer: " << renderer << std::endl;
	std::cout << "Anbieter: " << vendor << std::endl;
	std::cout << "GLSL-Version: " << shadingLanguageVersion << std::endl;

	// Profiltyp bestimmen
	std::cout << "OpenGL-Profil: ";
	if (profileMask & GL_CONTEXT_CORE_PROFILE_BIT)
		std::cout << "Core Profile" << std::endl;
	else if (profileMask & GL_CONTEXT_COMPATIBILITY_PROFILE_BIT)
		std::cout << "Compatibility Profile" << std::endl;
	else
		std::cout << "Unbekanntes Profil" << std::endl;
}



osg::ref_ptr<osg::Vec3Array> createBoundingBoxInstanceData(const std::vector<scm::gl::boxf>& bounding_boxes)
{
	osg::ref_ptr<osg::Vec3Array> instanceData = new osg::Vec3Array;
	instanceData->reserve(bounding_boxes.size() * 2);
	for (size_t i = 0; i < bounding_boxes.size(); ++i)
	{
		scm::math::vec3f min_vertex = bounding_boxes[i].min_vertex();
		scm::math::vec3f max_vertex = bounding_boxes[i].max_vertex();
		instanceData->push_back(osg::Vec3(min_vertex.x, min_vertex.y, min_vertex.z));
		instanceData->push_back(osg::Vec3(max_vertex.x, max_vertex.y, max_vertex.z));
	}

	return instanceData;
}


std::vector<float> LamurePointCloudPlugin::getBoxCorners(scm::gl::boxf bbv) {
	std::vector<float> corners_ = {
		bbv.corner(0).data_array[0], bbv.corner(0).data_array[1], bbv.corner(0).data_array[2],
		bbv.corner(1).data_array[0], bbv.corner(1).data_array[1], bbv.corner(1).data_array[2],
		bbv.corner(2).data_array[0], bbv.corner(2).data_array[1], bbv.corner(2).data_array[2],
		bbv.corner(3).data_array[0], bbv.corner(3).data_array[1], bbv.corner(3).data_array[2],
		bbv.corner(4).data_array[0], bbv.corner(4).data_array[1], bbv.corner(4).data_array[2],
		bbv.corner(5).data_array[0], bbv.corner(5).data_array[1], bbv.corner(5).data_array[2],
		bbv.corner(6).data_array[0], bbv.corner(6).data_array[1], bbv.corner(6).data_array[2],
		bbv.corner(7).data_array[0], bbv.corner(7).data_array[1], bbv.corner(7).data_array[2],
	};
	return corners_;
}


float* LamurePointCloudPlugin::VecToArr(std::vector<std::vector<float>> vec) {
	std::size_t totalsize = 0;
	for (int i = 0; i < vec.size(); i++) {
		totalsize += vec[i].size();
	}
	float* newarr = new float[totalsize];
	int index = 0;
	for (int i = 0; i < vec.size(); i++) {
		std::copy(vec[i].begin(), vec[i].end(), &newarr[index]);
		index += vec[i].size();
	}
	return newarr;
}


int* LamurePointCloudPlugin::VecToArr(std::vector<std::vector<int>> vec) {
	std::size_t totalsize = 0;
	for (int i = 0; i < vec.size(); i++) {
		totalsize += vec[i].size();
	}
	int* newarr = new int[totalsize];
	int index = 0;
	for (int i = 0; i < vec.size(); i++) {
		std::copy(vec[i].begin(), vec[i].end(), &newarr[index]);
		index += vec[i].size();
	}
	return newarr;
}


std::vector<vector<float>> LamurePointCloudPlugin::getSerializedBvhMinMax(const std::vector<scm::gl::boxf> bounding_boxes) {
	std::vector<vector<float>> vecOfVec;
	for (uint64_t node_id = 0; node_id < bounding_boxes.size(); ++node_id) {
		scm::math::vec3f min_vertex = bounding_boxes[node_id].min_vertex();
		scm::math::vec3f max_vertex = bounding_boxes[node_id].max_vertex();
		vector<float> elements{
					min_vertex.x, min_vertex.y, min_vertex.z,
					max_vertex.x, max_vertex.y, max_vertex.z };
		vecOfVec.push_back(elements);
	}
	return vecOfVec;
}


double roundToDecimal(double value, int decimals) {
	if (decimals < 0) {
		return value;
	}
	double factor = std::pow(10.0, decimals);
	return std::round(value * factor) / factor;
}


int LamurePointCloudPlugin::loadLMR(const char* filename, osg::Group* parent, const char* covise_key)
{
	std::printf("loadLMR()\n");
	assert(plugin);
	std::string lmr_file = std::string(filename);
	plugin->load_settings(lmr_file);
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

	render_width_ = settings_.width_ / settings_.frame_div_;
	render_height_ = settings_.height_ / settings_.frame_div_;

	char str[200];
	sprintf(str, "COVER.WindowConfig.Window:%d", 0);

	std::printf("render_width_: %03" PRId32 "\n", render_width_);
	std::printf("render_height_: %03" PRId32 "\n", render_height_);

	lamure::ren::policy* policy = lamure::ren::policy::get_instance();
	policy->set_max_upload_budget_in_mb(settings_.upload_);
	policy->set_render_budget_in_mb(settings_.vram_);
	policy->set_out_of_core_budget_in_mb(settings_.ram_);
	policy->set_window_width(render_width_);
	policy->set_window_height(render_height_);

	lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
	lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
	lamure::ren::controller* controller = lamure::ren::controller::get_instance();

	for (const auto& input_file : settings_.models_) {
		lamure::model_t model_id = database->add_model(input_file, std::to_string(num_models_));
		model_info_.model_transformations_.push_back(settings_.transforms_[num_models_] * scm::math::mat4d(scm::math::make_translation(database->get_model(num_models_)->get_bvh()->get_translation())));
		++num_models_;
	}
	return 1;
}


using namespace scm::gl;

struct LamureBoundingBoxUpdater : public osg::NodeCallback
{
	LamureBoundingBoxUpdater(LamurePointCloudPlugin* plugin)
		: _plugin(plugin) {
	}

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		osg::Geometry* geom = dynamic_cast<osg::Geometry*>(node);
		if (!geom)
		{
			traverse(node, nv);
			return;
		}

		// Retrieve or create the vertex array.
		osg::ref_ptr<osg::Vec3Array> vertices =
			dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
		if (!vertices)
		{
			vertices = new osg::Vec3Array;
			geom->setVertexArray(vertices.get());
		}
		vertices->clear();

		// Retrieve or create the index array.
		osg::ref_ptr<osg::DrawElementsUInt> indices = nullptr;
		if (geom->getNumPrimitiveSets() > 0)
		{
			indices = dynamic_cast<osg::DrawElementsUInt*>(geom->getPrimitiveSet(0));
		}
		if (!indices)
		{
			indices = new osg::DrawElementsUInt(GL_LINES);
			geom->addPrimitiveSet(indices.get());
		}
		else
		{
			indices->clear();
		}

		// === Lamure data update (sending transforms, camera, and dispatch) ===
		lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
		lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
		lamure::ren::controller* controller = lamure::ren::controller::get_instance();
		lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();

		if (lamure::ren::policy::get_instance()->size_of_provenance() > 0)
			controller->reset_system(data_provenance_);
		else
			controller->reset_system();

		scm::math::vec3d translation = settings_.model_tl_;
		scm::math::mat4d translationMatrix = scm::math::make_translation(translation);
		scm::math::mat4d rotationMatrix = settings_.model_rot_;

		lamure::context_t context_id = controller->deduce_context_id(lmr_ctx);
		for (lamure::model_t model_id = 0; model_id < num_models_; ++model_id)
		{
			lamure::model_t m_id = controller->deduce_model_id(std::to_string(model_id));
			cuts->send_transform(context_id, m_id, scm::math::mat4f(model_info_.model_transformations_[m_id] * translationMatrix * rotationMatrix));
			cuts->send_threshold(context_id, m_id, settings_.lod_error_);
			cuts->send_rendered(context_id, m_id);
			database->get_model(m_id)->set_transform(scm::math::mat4f(model_info_.model_transformations_[m_id] * translationMatrix * rotationMatrix));
		}

		lamure::view_t view_id = controller->deduce_view_id(context_id, scm_camera_->view_id());
		cuts->send_camera(context_id, view_id, *scm_camera_);
		std::vector<scm::math::vec3d> corner_values = scm_camera_->get_frustum_corners();
		double top_minus_bottom = scm::math::length(corner_values[2] - corner_values[0]);
		height_divided_by_top_minus_bottom_ = lamure::ren::policy::get_instance()->window_height() / top_minus_bottom;
		cuts->send_height_divided_by_top_minus_bottom(context_id, view_id, height_divided_by_top_minus_bottom_);

		if (settings_.lod_update_)
		{
			if (lamure::ren::policy::get_instance()->size_of_provenance() > 0)
				controller->dispatch(context_id, device_, data_provenance_);
			else
				controller->dispatch(context_id, device_);
		}
		// === End Lamure update ===

		for (lamure::model_t model_id = 0; model_id < num_models_; ++model_id)
		{
			const auto& bounding_boxes = database->get_model(model_id)->get_bvh()->get_bounding_boxes();
			lamure::ren::cut& cut = cuts->get_cut(context_id, lmr_ctx, model_id);
			std::vector<lamure::ren::cut::node_slot_aggregate> visibleNodes = cut.complete_set();
			std::cout << "Model " << model_id << " visible nodes: " << visibleNodes.size() << std::endl;

			for (const auto& node_slot : visibleNodes)
			{
				// getBoxCorners returns 24 floats: 8 vertices * 3 components.
				std::vector<float> boxCorners = _plugin->getBoxCorners(bounding_boxes[node_slot.node_id_]);
				if (boxCorners.size() < 24) continue; // sanity check

				// Remember the starting index of this box's vertices.
				unsigned int baseIndex = vertices->size();

				// Add 8 vertices to the vertex array.
				for (int v = 0; v < 8; ++v)
				{
					float x = boxCorners[3 * v + 0];
					float y = boxCorners[3 * v + 1];
					float z = boxCorners[3 * v + 2];
					vertices->push_back(osg::Vec3(x, y, z));
				}

				// Append indices for the 12 edges of the box.
				// Bottom face: vertices 0,1,2,3.
				indices->push_back(baseIndex + 0); indices->push_back(baseIndex + 1);
				indices->push_back(baseIndex + 1); indices->push_back(baseIndex + 2);
				indices->push_back(baseIndex + 2); indices->push_back(baseIndex + 3);
				indices->push_back(baseIndex + 3); indices->push_back(baseIndex + 0);
				// Top face: vertices 4,5,6,7.
				indices->push_back(baseIndex + 4); indices->push_back(baseIndex + 5);
				indices->push_back(baseIndex + 5); indices->push_back(baseIndex + 6);
				indices->push_back(baseIndex + 6); indices->push_back(baseIndex + 7);
				indices->push_back(baseIndex + 7); indices->push_back(baseIndex + 4);
				// Vertical edges: 0-4, 1-5, 2-6, 3-7.
				indices->push_back(baseIndex + 0); indices->push_back(baseIndex + 4);
				indices->push_back(baseIndex + 1); indices->push_back(baseIndex + 5);
				indices->push_back(baseIndex + 2); indices->push_back(baseIndex + 6);
				indices->push_back(baseIndex + 3); indices->push_back(baseIndex + 7);
			}
		}

		vertices->dirty();
		geom->dirtyDisplayList();

		traverse(node, nv);
	}
private:
	LamurePointCloudPlugin* _plugin;
};


struct CoordGeometry : public osg::Geometry
{
	CoordGeometry(osg::ref_ptr<osg::StateSet> stateset, LamurePointCloudPlugin* plugin)
		: _stateset(stateset), _plugin(plugin)
	{
		if (plugin->notify_button->state()) { std::cout << "[Notify] CoordGeometry()" << std::endl; }
		setUseDisplayList(false);
		setUseVertexBufferObjects(true);
		setUseVertexArrayObject(false);

		osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
		vertices->push_back(osg::Vec3(0.f, 0.f, 0.f));
		vertices->push_back(osg::Vec3(30.f, 0.f, 0.f));
		vertices->push_back(osg::Vec3(0.f, 30.f, 0.f));
		vertices->push_back(osg::Vec3(0.f, 0.f, 30.f));
		setVertexArray(vertices.get());

		osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_LINES);
		indices->push_back(0); indices->push_back(1);
		indices->push_back(0); indices->push_back(2);
		indices->push_back(0); indices->push_back(3);
		addPrimitiveSet(indices.get());
	}
	osg::ref_ptr<osg::StateSet> _stateset;
	LamurePointCloudPlugin* _plugin;
};

struct CoordDrawCallbackGL : public osg::Drawable::DrawCallback
{
	CoordDrawCallbackGL(osg::ref_ptr<osg::StateSet> stateset, LamurePointCloudPlugin* plugin)
		: _stateset(stateset),
		_plugin(plugin),
		_initialized(false) {
	}

	virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const override
	{
		GLStateBackup stateBackup = captureGLBackup();
		GLStateSnapshot stateBefore = captureGLState();
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		if (!_initialized)
		{
			_plugin->create_coord_resources();
			_initialized = true;
		}

		if (!glIsBuffer(coord_resource_.vbo_)) { std::cerr << "VBO ist ungültig!" << std::endl; }
		if (!glIsBuffer(coord_resource_.ibo_)) { std::cerr << "IBO ist ungültig!" << std::endl; }
		if (!glIsVertexArray(coord_resource_.vao_)) { std::cerr << "VAO ist ungültig!" << std::endl; }

		glBindVertexArray(coord_resource_.vao_);
		glBindBuffer(GL_ARRAY_BUFFER, coord_resource_.vbo_);
		glUseProgram(coord_resource_.program_);

		GLdouble gl_mvm[16];
		GLdouble gl_pm[16];
		glGetDoublev(GL_MODELVIEW_MATRIX, gl_mvm);
		glGetDoublev(GL_PROJECTION_MATRIX, gl_pm);
		scm::math::mat4d gl_view_matrix_d = scm::math::mat4d(gl_mvm[0], gl_mvm[1], gl_mvm[2], gl_mvm[3], gl_mvm[4], gl_mvm[5], gl_mvm[6], gl_mvm[7], gl_mvm[8], gl_mvm[9], gl_mvm[10], gl_mvm[11], gl_mvm[12], gl_mvm[13], gl_mvm[14], gl_mvm[15]);
		scm::math::mat4d gl_projection_matrix_d = scm::math::mat4d(gl_pm[0], gl_pm[1], gl_pm[2], gl_pm[3], gl_pm[4], gl_pm[5], gl_pm[6], gl_pm[7], gl_pm[8], gl_pm[9], gl_pm[10], gl_pm[11], gl_pm[12], gl_pm[13], gl_pm[14], gl_pm[15]);
		scm::math::mat4d view_matrix = gl_view_matrix_d;
		scm::math::mat4d projection_matrix = gl_projection_matrix_d;

		scm::math::mat4d mvp_matrix = projection_matrix * view_matrix;
		float* mvp = scm::math::mat4f(mvp_matrix).data_array;

		glUniformMatrix4fv(glGetUniformLocation(coord_resource_.program_, "mvp_matrix"), 1, GL_FALSE, &mvp[0]);
		glUniform4f(glGetUniformLocation(coord_resource_.program_, "in_color"), settings_.frustum_color_[0], settings_.frustum_color_[1], settings_.frustum_color_[2], settings_.frustum_color_[3]);
		glDrawElements(GL_LINES, coord_resource_.idx_.size(), GL_UNSIGNED_SHORT, nullptr);
		glBindVertexArray(0);

		glPopAttrib();
		restoreGLBackup(stateBackup);

		if (_plugin->notify_button->state()) {
			GLStateSnapshot stateAfter = captureGLState();
			compareGLStateSnapshots(stateBefore, stateAfter, "[Notify] BoundingBoxDrawCallback::drawImplementation()");
		}
	}
	osg::ref_ptr<osg::StateSet> _stateset;
	LamurePointCloudPlugin* _plugin;
	mutable bool _initialized;
};

struct CoordGeometryGL : public osg::Geometry
{
	CoordGeometryGL(osg::ref_ptr<osg::StateSet> stateset, LamurePointCloudPlugin* plugin)
		: _stateset(stateset), _plugin(plugin)
	{
		if (plugin->notify_button->state()) { std::cout << "[Notify] CoordGeometryGL()" << std::endl; }
		setUseDisplayList(false);
		setUseVertexBufferObjects(true);
		setUseVertexArrayObject(false);
		setDrawCallback(new CoordDrawCallbackGL(stateset, plugin));
	}
	osg::ref_ptr<osg::StateSet> _stateset;
	LamurePointCloudPlugin* _plugin;
};

struct FrustumDrawCallbackGL : public osg::Drawable::DrawCallback
{
	FrustumDrawCallbackGL(osg::ref_ptr<osg::StateSet> stateset, LamurePointCloudPlugin* plugin)
		: _stateset(stateset),
		_plugin(plugin),
		_initialized(false) {
	}

	virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const override
	{
		GLStateBackup stateBackup = captureGLBackup();
		GLStateSnapshot stateBefore = captureGLState();
		glPushAttrib(GL_ALL_ATTRIB_BITS);

		if (!_initialized)
		{
			_plugin->create_frustum_resources();
			_initialized = true;
		}

		std::vector<scm::math::vec3d> corner_values = scm_camera_->get_frustum_corners();
		for (size_t i = 0; i < corner_values.size(); ++i) {
			auto vv = scm::math::vec3f(corner_values[i]);
			frustum_resource_.vertices_[i * 3 + 0] = vv.x;
			frustum_resource_.vertices_[i * 3 + 1] = vv.y;
			frustum_resource_.vertices_[i * 3 + 2] = vv.z;
		}

		glBindVertexArray(frustum_resource_.vao_);
		glBindBuffer(GL_ARRAY_BUFFER, frustum_resource_.vbo_);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * frustum_resource_.vertices_.size(), frustum_resource_.vertices_.data(), GL_STATIC_DRAW);
		//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, frustum_resource_.ibo_);
		//glBufferData(GL_ELEMENT_ARRAY_BUFFER, frustum_resource_.idx_.size() * sizeof(unsigned short), frustum_resource_.idx_.data(), GL_STATIC_DRAW);
		glUseProgram(frustum_resource_.program_);
		//printAllExistingVAOs();

		GLdouble gl_mvm[16];
		GLdouble gl_pm[16];
		glGetDoublev(GL_MODELVIEW_MATRIX, gl_mvm);
		glGetDoublev(GL_PROJECTION_MATRIX, gl_pm);
		scm::math::mat4d gl_view_matrix_d = scm::math::mat4d(gl_mvm[0], gl_mvm[1], gl_mvm[2], gl_mvm[3], gl_mvm[4], gl_mvm[5], gl_mvm[6], gl_mvm[7], gl_mvm[8], gl_mvm[9], gl_mvm[10], gl_mvm[11], gl_mvm[12], gl_mvm[13], gl_mvm[14], gl_mvm[15]);
		scm::math::mat4d gl_projection_matrix_d = scm::math::mat4d(gl_pm[0], gl_pm[1], gl_pm[2], gl_pm[3], gl_pm[4], gl_pm[5], gl_pm[6], gl_pm[7], gl_pm[8], gl_pm[9], gl_pm[10], gl_pm[11], gl_pm[12], gl_pm[13], gl_pm[14], gl_pm[15]);
		scm::math::mat4d view_matrix = gl_view_matrix_d;
		scm::math::mat4d projection_matrix = gl_projection_matrix_d;

		scm::math::mat4d mvp_matrix = projection_matrix * view_matrix;
		float* mvp = scm::math::mat4f(mvp_matrix).data_array;

		glUniformMatrix4fv(glGetUniformLocation(frustum_resource_.program_, "mvp_matrix"), 1, GL_FALSE, &mvp[0]);
		glUniform4f(glGetUniformLocation(frustum_resource_.program_, "in_color"), settings_.frustum_color_[0], settings_.frustum_color_[1], settings_.frustum_color_[2], settings_.frustum_color_[3]);
		//glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 24, &frustum_resource_.vertices_[0], GL_STATIC_DRAW);
		glDrawElements(GL_LINES, frustum_resource_.idx_.size(), GL_UNSIGNED_SHORT, nullptr);

		//printBufferContents(GL_ELEMENT_ARRAY_BUFFER, frustum_resource_.ibo_, frustum_resource_.idx_.size() * sizeof(unsigned short));
		//printBufferContents(GL_ARRAY_BUFFER, frustum_resource_.vbo_, sizeof(float) * frustum_resource_.vertices_[0].size());
		//printVAOAttributes(frustum_resource_.vao_);

		glPopAttrib();
		restoreGLBackup(stateBackup);

		if (_plugin->notify_button->state()) {
			GLStateSnapshot stateAfter = captureGLState();
			compareGLStateSnapshots(stateBefore, stateAfter, "[Notify] BoundingBoxDrawCallback::drawImplementation()");
		}
	}
	osg::ref_ptr<osg::StateSet> _stateset;
	LamurePointCloudPlugin* _plugin;
	mutable bool _initialized;
};


struct FrustumGeometryGL : public osg::Geometry
{
	FrustumGeometryGL(osg::ref_ptr<osg::StateSet> stateset, LamurePointCloudPlugin* plugin)
	{
		if (plugin->notify_button->state()) { std::cout << "[Notify] FrustumGeometryGL()" << std::endl; }
		setUseDisplayList(false);
		setUseVertexBufferObjects(true);
		setUseVertexArrayObject(false);
		setDrawCallback(new FrustumDrawCallbackGL(stateset, plugin));
	}
};

struct FrustumGeometry : public osg::Geometry
{
	FrustumGeometry(osg::ref_ptr<osg::StateSet> stateset, LamurePointCloudPlugin* plugin)
	{
		if (plugin->notify_button->state()) { std::cout << "[Notify] FrustumGeometry()" << std::endl; }
		setUseDisplayList(false);
		setUseVertexBufferObjects(true);
		setUseVertexArrayObject(false);

		std::vector<scm::math::vec3d> frustumCorners = scm_camera_->get_frustum_corners();
		osg::ref_ptr<osg::Vec3Array> vertexArray = new osg::Vec3Array;

		for (const auto& c : frustumCorners)
		{
			vertexArray->push_back(osg::Vec3(static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(c.z)));
		}

		osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_LINES);

		indices->push_back(0); indices->push_back(1);
		indices->push_back(2); indices->push_back(3);
		indices->push_back(4); indices->push_back(5);
		indices->push_back(6); indices->push_back(7);

		indices->push_back(0); indices->push_back(2);
		indices->push_back(1); indices->push_back(3);
		indices->push_back(4); indices->push_back(6);
		indices->push_back(5); indices->push_back(7);

		indices->push_back(0); indices->push_back(4);
		indices->push_back(1); indices->push_back(5);
		indices->push_back(2); indices->push_back(6);
		indices->push_back(3); indices->push_back(7);

		this->setVertexArray(vertexArray);
		this->addPrimitiveSet(indices);

		//stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
		//osg::ref_ptr<osg::LineWidth> lineWidth = new osg::LineWidth(2.0f);
		//stateset->setAttribute(lineWidth);
	}
};


struct BoundingBoxDrawCallback : public osg::Drawable::DrawCallback
{
	BoundingBoxDrawCallback(osg::ref_ptr<osg::StateSet> stateset, LamurePointCloudPlugin* plugin)
		: _stateset(stateset),
		_plugin(plugin) 
	{
		if (plugin->notify_button->state()) { std::cout << "[Notify] BoundingBoxDrawCallback()" << std::endl; }
	}

	virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const override
	{
		GLdouble gl_mvm[16];
		GLdouble gl_pm[16];
		glGetDoublev(GL_MODELVIEW_MATRIX, gl_mvm);
		glGetDoublev(GL_PROJECTION_MATRIX, gl_pm);

		osg::Matrixd osg_view_matrix;
		osg_view_matrix.set(gl_mvm);

		osg::Matrixd osg_projection_matrix;
		osg_projection_matrix.set(gl_pm);

		scm::math::vec3d translation = settings_.model_tl_;
		scm::math::mat4d translationMatrix = scm::math::make_translation(translation);
		scm::math::mat4d rotationMatrix = settings_.model_rot_;

		scm::math::mat4d model_matrix = model_info_.model_transformations_[0] * translationMatrix * rotationMatrix;

		_stateset->getUniform("modelMatrix")->set(matConv4F(scm::math::mat4f(model_matrix)));
		_stateset->getUniform("viewMatrix")->set(osg_view_matrix);
		_stateset->getUniform("projectionMatrix")->set(osg_projection_matrix);

		const osg::Geometry* geom = drawable->asGeometry();
		if (!geom) return;
		const osg::Vec3Array* instanceMins = dynamic_cast<const osg::Vec3Array*>(geom->getVertexAttribArray(1));
		const osg::Vec3Array* instanceMaxs = dynamic_cast<const osg::Vec3Array*>(geom->getVertexAttribArray(2));
		if (!instanceMins || !instanceMaxs) return;

		GLuint instanceCount = instanceMins->size();

		osg::Uniform* instanceMinUniform = _stateset->getUniform("instanceMin");
		osg::Uniform* instanceMaxUniform = _stateset->getUniform("instanceMax");
		if (!instanceMinUniform || !instanceMaxUniform)
		{
			_stateset->addUniform(new osg::Uniform("instanceMin", osg::Vec3(0.0f, 0.0f, 0.0f)));
			_stateset->addUniform(new osg::Uniform("instanceMax", osg::Vec3(0.0f, 0.0f, 0.0f)));
		}

		for (GLuint i = 0; i < instanceCount; ++i)
		{
			instanceMinUniform->set(instanceMins->at(i));
			instanceMaxUniform->set(instanceMaxs->at(i));
			glDrawArrays(GL_POINTS, 0, 1);
		}
	}
private:
	osg::ref_ptr<osg::StateSet> _stateset;
	LamurePointCloudPlugin* _plugin;
};

struct BoundingBoxGeometry : public osg::Geometry
{
	BoundingBoxGeometry(osg::ref_ptr<osg::StateSet> stateset, LamurePointCloudPlugin* plugin)
	{
		if (plugin->notify_button->state()) { std::cout << "[Notify] BoundingBoxGeometry()" << std::endl; }
		setUseDisplayList(false);
		setUseVertexBufferObjects(true);
		setUseVertexArrayObject(false);

		osg::ref_ptr<osg::Vec3Array> baseVertices = new osg::Vec3Array;
		baseVertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
		this->setVertexArray(baseVertices.get());
		this->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, baseVertices->size()));

		lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
		const auto& bvh_ = database->get_model(0)->get_bvh();
		std::vector<scm::gl::boxf> const& bounding_boxes = bvh_->get_bounding_boxes();

		osg::ref_ptr<osg::Vec3Array> instanceMins = new osg::Vec3Array;
		osg::ref_ptr<osg::Vec3Array> instanceMaxs = new osg::Vec3Array;
		instanceMins->reserve(bounding_boxes.size());
		instanceMaxs->reserve(bounding_boxes.size());

		for (size_t i = 0; i < bounding_boxes.size(); ++i)
		{
			scm::math::vec3f minV = bounding_boxes[i].min_vertex();
			scm::math::vec3f maxV = bounding_boxes[i].max_vertex();
			instanceMins->push_back(osg::Vec3(minV.x, minV.y, minV.z));
			instanceMaxs->push_back(osg::Vec3(maxV.x, maxV.y, maxV.z));
		}

		this->setVertexAttribArray(1, instanceMins.get());
		this->setVertexAttribBinding(1, osg::Geometry::BIND_PER_VERTEX);
		this->setVertexAttribArray(2, instanceMaxs.get());
		this->setVertexAttribBinding(2, osg::Geometry::BIND_PER_VERTEX);

		baseVertices->setDataVariance(osg::Object::DYNAMIC);
		instanceMins->setDataVariance(osg::Object::DYNAMIC);
		instanceMaxs->setDataVariance(osg::Object::DYNAMIC);
		this->setDataVariance(osg::Object::DYNAMIC);

		this->setDrawCallback(new BoundingBoxDrawCallback(stateset, plugin));
	}
};

struct BoundingBoxDrawCallbackGL : public virtual osg::Drawable::DrawCallback
{
	BoundingBoxDrawCallbackGL(osg::ref_ptr<osg::StateSet> stateset, LamurePointCloudPlugin* plugin)
		: _stateset(stateset),
		_plugin(plugin),
		_initialized(false)
	{ 
		if (plugin->notify_button->state()) { std::cout << "[Notify] BoundingBoxDrawCallbackGL()" << std::endl; } 
	}

	void initializeResources() const {

		for (uint32_t model_id = 0; model_id < num_models_; ++model_id) {
			std::vector<vector<float>> corners_;
			const auto& bvh_ = lamure::ren::model_database::get_instance()->get_model(model_id)->get_bvh();
			const auto& bounding_boxes = bvh_->get_bounding_boxes();
			for (uint64_t node_id = 0; node_id < bounding_boxes.size(); ++node_id) {
				corners_.push_back(_plugin->getBoxCorners(bounding_boxes[node_id]));
			}
			bvh_res_[model_id].corners_ = corners_;
		}

		GLuint vao_;
		glGenVertexArrays(1, &vao_);
		glBindVertexArray(vao_);

		GLuint ibo_;
		glGenBuffers(1, &ibo_);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, box_resource_.idx_.size() * sizeof(unsigned short), box_resource_.idx_.data(), GL_STATIC_DRAW);

		GLuint vbo_;
		glGenBuffers(1, &vbo_);
		glBindBuffer(GL_ARRAY_BUFFER, vbo_);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 24, nullptr, GL_DYNAMIC_DRAW);

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

		unsigned int program_ = glCreateProgram();
		unsigned int vs = CompileShader(GL_VERTEX_SHADER, vis_line_bb_vs_source, 0);
		unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, vis_line_bb_fs_source, 0);
		glAttachShader(program_, vs);
		glAttachShader(program_, fs);
		glLinkProgram(program_);
		glValidateProgram(program_);
		glDeleteShader(vs);
		glDeleteShader(fs);

		box_resource_.vao_ = vao_;
		box_resource_.vbo_ = vbo_;
		box_resource_.ibo_ = ibo_;
		box_resource_.program_ = program_;

		glBindVertexArray(0);
	}

	virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const {

		osg::State* state = renderInfo.getState();
		state->setCheckForGLErrors(osg::State::CheckForGLErrors::ONCE_PER_ATTRIBUTE);
		GLStateBackup stateBackup = captureGLBackup();
		GLStateSnapshot stateBefore = captureGLState();
		glPushAttrib(GL_ALL_ATTRIB_BITS);

		if (!_initialized) {
			if (_plugin->notify_button->state()) { std::cout << "[Notify] BoundingBoxDrawCallbackGL::drawImplementation()" << std::endl; }
			//_plugin->HDC_draw = wglGetCurrentDC();
			//_plugin->HGLRC_draw = wglGetCurrentContext();
			//_plugin->init_schism_objects();
		}

		GLdouble vm[16];
		glGetDoublev(GL_MODELVIEW_MATRIX, vm);
		GLdouble pm[16];
		glGetDoublev(GL_PROJECTION_MATRIX, pm);

		scm::math::mat4d gl_view_matrix_d = scm::math::mat4d(vm[0], vm[1], vm[2], vm[3], vm[4], vm[5], vm[6], vm[7], vm[8], vm[9], vm[10], vm[11], vm[12], vm[13], vm[14], vm[15]);
		scm::math::mat4d gl_projection_matrix_d = scm::math::mat4d(pm[0], pm[1], pm[2], pm[3], pm[4], pm[5], pm[6], pm[7], pm[8], pm[9], pm[10], pm[11], pm[12], pm[13], pm[14], pm[15]);
		float* vmm = scm::math::mat4f(gl_view_matrix_d).data_array;
		float* pmm = scm::math::mat4f(gl_projection_matrix_d).data_array;
		//screen_quad_.reset(new scm::gl::quad_geometry(device_, scm::math::vec2f(-1.0f, -1.0f), scm::math::vec2f(1.0f, 1.0f)));

		lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
		lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
		lamure::ren::controller* controller = lamure::ren::controller::get_instance();
		lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();
		if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) { controller->reset_system(data_provenance_); }
		else { controller->reset_system(); }

		scm::math::vec3d translation = settings_.model_tl_;
		scm::math::mat4d translationMatrix = scm::math::make_translation(translation);
		scm::math::mat4d rotationMatrix = settings_.model_rot_;

		lamure::context_t context_id = controller->deduce_context_id(lmr_ctx);
		for (lamure::model_t model_id = 0; model_id < num_models_; ++model_id) {
			lamure::model_t m_id = controller->deduce_model_id(std::to_string(model_id));
			cuts->send_transform(context_id, m_id, scm::math::mat4f(model_info_.model_transformations_[m_id] * translationMatrix * rotationMatrix));
			cuts->send_threshold(context_id, m_id, settings_.lod_error_);
			cuts->send_rendered(context_id, m_id);
			database->get_model(m_id)->set_transform(scm::math::mat4f(model_info_.model_transformations_[m_id] * translationMatrix * rotationMatrix));
		}

		lamure::view_t view_id = controller->deduce_view_id(context_id, scm_camera_->view_id());
		cuts->send_camera(context_id, view_id, *scm_camera_);
		std::vector<scm::math::vec3d> corner_values = scm_camera_->get_frustum_corners();
		double top_minus_bottom = scm::math::length((corner_values[2]) - (corner_values[0]));
		height_divided_by_top_minus_bottom_ = lamure::ren::policy::get_instance()->window_height() / top_minus_bottom;
		cuts->send_height_divided_by_top_minus_bottom(context_id, view_id, height_divided_by_top_minus_bottom_);

		if (settings_.use_pvs_) {
			scm::math::vec3d cam_pos = scm_camera_->get_cam_pos();
			pvs->set_viewer_position(cam_pos);
		}

		if (settings_.lod_update_) {
			if (lamure::ren::policy::get_instance()->size_of_provenance() > 0)
			{ controller->dispatch(context_id, device_, data_provenance_); }
			else { controller->dispatch(context_id, device_); }
		}

		if (!_initialized) {
			initializeResources();
			//_plugin->create_box_resources();
			_initialized = true;
		}

		//if (!glIsVertexArray(box_resource_.vao_)) { std::cerr << "VAO ist ungültig!" << std::endl; }
		//if (!glIsBuffer(box_resource_.vbo_)) { std::cerr << "VBO ist ungültig!" << std::endl; }
		//if (!glIsBuffer(box_resource_.ibo_)) { std::cerr << "IBO ist ungültig!" << std::endl; }

		glBindVertexArray(box_resource_.vao_);
		glUseProgram(box_resource_.program_);
		GLint mvpLocation = glGetUniformLocation(box_resource_.program_, "mvp_matrix");
		GLint colorLocation = glGetUniformLocation(box_resource_.program_, "in_color");
		glUniform4f(colorLocation, settings_.bvh_color_[0], settings_.bvh_color_[1], settings_.bvh_color_[2], settings_.bvh_color_[3]);

		uint64_t rendered_bounding_boxes = 0;
		for (uint16_t model_id = 0; model_id < num_models_; ++model_id) {
			scm::math::mat4d model_matrix = model_info_.model_transformations_[model_id] * translationMatrix * rotationMatrix;
			scm::math::mat4d projection_matrix = scm::math::mat4d(gl_projection_matrix_d);
			scm::math::mat4d view_matrix = gl_view_matrix_d;
			scm::math::mat4d mvp_matrix = projection_matrix * view_matrix * model_matrix;
			scm::gl::frustum frustum_ = scm_camera_->get_frustum_by_model(model_matrix);

			float* mvp = scm::math::mat4f(mvp_matrix).data_array;
			glUniformMatrix4fv(mvpLocation, 1, GL_FALSE, mvp);

			lamure::context_t context_id = controller->deduce_context_id(lmr_ctx);
			lamure::ren::cut& cut = cuts->get_cut(context_id, lmr_ctx, model_id);
			std::vector<lamure::ren::cut::node_slot_aggregate> renderable = cut.complete_set();
			const lamure::ren::bvh* bvh = database->get_model(model_id)->get_bvh();
			std::vector<scm::gl::boxf>const& bbv = bvh->get_bounding_boxes();

			for (auto const& node_slot_aggregate : renderable) {
				uint32_t node_culling_result = scm_camera_->cull_against_frustum(frustum_, bbv[node_slot_aggregate.node_id_]);
				if (node_culling_result != 1) {
					const std::vector<float>& corners_ = bvh_res_[model_id].corners_[node_slot_aggregate.node_id_];
					glBindBuffer(GL_ARRAY_BUFFER, box_resource_.vbo_);
					glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * corners_.size(), corners_.data());
					glDrawElements(GL_LINES, 24, GL_UNSIGNED_SHORT, nullptr);
					rendered_bounding_boxes++;
				}
			}
		}
		render_info_.rendered_bounding_boxes_ = rendered_bounding_boxes;
		glPopAttrib();
		restoreGLBackup(stateBackup);

		if (_plugin->notify_button->state()) {
			GLStateSnapshot stateAfter = captureGLState();
			compareGLStateSnapshots(stateBefore, stateAfter, "[Notify] BoundingBoxDrawCallback::drawImplementation()");
		}
	};
	mutable bool _initialized;
	osg::ref_ptr<osg::StateSet> _stateset;
	LamurePointCloudPlugin* _plugin;
};

struct BoundingBoxGeometryGL : public osg::Geometry
{
	BoundingBoxGeometryGL(osg::ref_ptr<osg::StateSet> stateset, LamurePointCloudPlugin* plugin)
	{
		if (plugin->notify_button->state()) { std::cout << "[Notify] BoundingBoxGeometryGL()" << std::endl; }
		setUseDisplayList(false);
		setUseVertexBufferObjects(true);
		setUseVertexArrayObject(false);
		setDrawCallback(new BoundingBoxDrawCallbackGL(stateset, plugin));
	}
};


struct PointsDrawCallback : public virtual osg::Drawable::DrawCallback
{
	PointsDrawCallback(osg::ref_ptr<osg::StateSet> pointcloud_stateset, LamurePointCloudPlugin* plugin)
		: _stateset(pointcloud_stateset),
		_plugin(plugin),
		_initialized(false)
	{ if (_plugin->notify_button->state()) { std::cout << "[Notify] PointsDrawCallback()" << std::endl; } }


	virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const
	{
		if (_plugin->rendering_) { return; }
		_plugin->rendering_ = true;

		GLStateBackup stateBackup = captureGLBackup();
		GLStateSnapshot stateBefore = captureGLState();

		GLint prevVAO = 0;
		glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &prevVAO);

		osg::State* state = renderInfo.getState();
		state->setCheckForGLErrors(osg::State::CheckForGLErrors::ONCE_PER_ATTRIBUTE);


		glDisable(GL_CULL_FACE);
		scm::math::mat4d gl_view_matrix_d = matConv4D(osg::Matrixd(renderInfo.getState()->getModelViewMatrix()));
		scm::math::mat4d gl_projection_matrix_d = matConv4D(osg::Matrixd(renderInfo.getState()->getProjectionMatrix()));

		scm::math::mat4 gl_view_matrix = matConv4F(osg::Matrix(renderInfo.getState()->getModelViewMatrix()));
		scm::math::mat4 gl_projection_matrix = matConv4F(osg::Matrix(renderInfo.getState()->getProjectionMatrix()));

		lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
		lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
		lamure::ren::controller* controller = lamure::ren::controller::get_instance();
		lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();
		if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) { controller->reset_system(data_provenance_); }
		else { controller->reset_system(); }

		scm::math::vec3d translation = settings_.model_tl_;
		scm::math::mat4d translationMatrix = scm::math::make_translation(translation);
		scm::math::mat4d rotationMatrix = settings_.model_rot_;

		lamure::context_t context_id = controller->deduce_context_id(lmr_ctx);
		for (lamure::model_t model_id = 0; model_id < num_models_; ++model_id) {
			lamure::model_t m_id = controller->deduce_model_id(std::to_string(model_id));
			cuts->send_transform(context_id, m_id, scm::math::mat4f(model_info_.model_transformations_[m_id] * translationMatrix * rotationMatrix));
			cuts->send_threshold(context_id, m_id, settings_.lod_error_);
			cuts->send_rendered(context_id, m_id);
			database->get_model(m_id)->set_transform(scm::math::mat4f(model_info_.model_transformations_[m_id] * translationMatrix * rotationMatrix));
		}

		lamure::view_t view_id = controller->deduce_view_id(context_id, scm_camera_->view_id());
		cuts->send_camera(context_id, view_id, *scm_camera_);
		std::vector<scm::math::vec3d> corner_values = scm_camera_->get_frustum_corners();
		double top_minus_bottom = scm::math::length((corner_values[2]) - (corner_values[0]));
		height_divided_by_top_minus_bottom_ = lamure::ren::policy::get_instance()->window_height() / top_minus_bottom;
		cuts->send_height_divided_by_top_minus_bottom(context_id, view_id, height_divided_by_top_minus_bottom_);

		if (settings_.use_pvs_) {
			scm::math::vec3d cam_pos = scm_camera_->get_cam_pos();
			pvs->set_viewer_position(cam_pos);
		}
		if (settings_.lod_update_) {
			if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) 
			{ controller->dispatch(context_id, device_, data_provenance_); }
			else { controller->dispatch(context_id, device_); }
		}


		if (_plugin->dump_button->state()) {
			printVAOAttributes(prevVAO);
		}

		if (!_initialized) {
			if (_plugin->notify_button->state()) { std::cout << "[Notify] PointsDrawCallback::drawImplementation()" << std::endl; }
		}

		if (_initialized) {
			glBindVertexArray(pcl_resource_.vao_);
		}


		//context_->set_rasterizer_state(no_backface_culling_rasterizer_state_, 1.0f, 1.0f);
		//auto selected_single_pass_shading_program = vis_xyz_shader_;
		//if (settings_.enable_lighting_) {
		//	selected_single_pass_shading_program = vis_xyz_lighting_shader_;
		//}

		//context_->clear_color_buffer(fbo_, 0, scm::math::vec4f(settings_.background_color_.x, settings_.background_color_.y, settings_.background_color_.z, 1.0f));
		//context_->clear_depth_stencil_buffer(fbo_);
		//context_->set_frame_buffer(fbo_);
		//context_->apply_frame_buffer();
		
		//context_->bind_program(selected_single_pass_shading_program);
		//_plugin->set_lamure_uniforms(selected_single_pass_shading_program);
		//context_->set_blend_state(color_no_blending_state_);
		//context_->set_depth_stencil_state(depth_state_less_);
		//context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(traits->width, traits->height)));
		//context_->apply();

		//context_->apply_texture_units();
		//context_->apply_image_units();
		//context_->apply_frame_buffer();
		context_->apply_vertex_input();
		//context_->apply_state_objects();
		//context_->apply_uniform_buffer_bindings();
		//context_->apply_atomic_counter_bindings();
		//context_->apply_storage_buffer_bindings();
		//context_->apply_program();

		//_plugin->set_stateset_uniforms(_stateset);

		if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
			context_->bind_vertex_array(controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, device_, data_provenance_));
		}
		else { context_->bind_vertex_array(controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, device_)); }



		glUseProgram(pcl_resource_.program_);
		_plugin->set_gl_uniforms(pcl_resource_.program_);

		uint64_t rendered_splats_ = 0;
		uint64_t rendered_nodes_ = 0;
		for (uint16_t model_id = 0; model_id < num_models_; ++model_id) {
			lamure::context_t context_id = controller->deduce_context_id(lmr_ctx);
			lamure::ren::cut& cut = cuts->get_cut(context_id, lmr_ctx, model_id);
			std::vector<lamure::ren::cut::node_slot_aggregate> renderable = cut.complete_set();
			const lamure::ren::bvh* bvh = database->get_model(model_id)->get_bvh();
			size_t surfels_per_node = database->get_primitives_per_node();
			std::vector<scm::gl::boxf>const& bounding_box_vector = bvh->get_bounding_boxes();

			scm::math::mat4d model_matrix = model_info_.model_transformations_[model_id] * translationMatrix * rotationMatrix;
			scm::gl::frustum frustum_ = scm_camera_->get_frustum_by_model(model_info_.model_transformations_[model_id]);
			scm::math::mat4d projection_matrix = scm::math::mat4d(gl_projection_matrix_d);
			scm::math::mat4d view_matrix = gl_view_matrix_d;
			scm::math::mat4d model_view_matrix = view_matrix * model_matrix;
			scm::math::mat4d model_view_projection_matrix = projection_matrix * model_view_matrix;
			const scm::math::mat4d viewport_scale = scm::math::make_scale(traits->width * 0.5, traits->height * 0.5, 0.5);
			const scm::math::mat4d viewport_translate = scm::math::make_translation(1.0, 1.0, 1.0);
			const scm::math::mat4d model_to_screen = viewport_scale * viewport_translate;

			scm::math::mat4 model_matrix_ = scm::math::mat4(model_info_.model_transformations_[model_id] * translationMatrix * rotationMatrix);
			scm::math::mat4 model_view_matrix_ = gl_view_matrix * model_matrix_;
			scm::math::mat4 model_view_projection_matrix_ = gl_projection_matrix * gl_view_matrix;

			float* mvp_ = scm::math::mat4f(model_view_projection_matrix).data_array;
			float* mmm_ = scm::math::mat4f(model_matrix).data_array;
			float* mvm_ = scm::math::mat4f(model_view_matrix).data_array;
			float* ivm_ = scm::math::mat4f(scm::math::transpose(scm::math::inverse(model_view_matrix))).data_array;
			float* mms_ = scm::math::mat4f(model_to_screen).data_array;

			glUniformMatrix4fv(glGetUniformLocation(pcl_resource_.program_, "mvp_matrix"), 1, GL_FALSE, &mvp_[0]);
			glUniformMatrix4fv(glGetUniformLocation(pcl_resource_.program_, "model_matrix"), 1, GL_FALSE, &mmm_[0]);
			glUniformMatrix4fv(glGetUniformLocation(pcl_resource_.program_, "model_view_matrix"), 1, GL_FALSE, &mvm_[0]);
			glUniformMatrix4fv(glGetUniformLocation(pcl_resource_.program_, "inv_mv_matrix"), 1, GL_FALSE, &ivm_[0]);
			glUniformMatrix4fv(glGetUniformLocation(pcl_resource_.program_, "model_to_screen_matrix"), 1, GL_FALSE, &mms_[0]);

			bool draw = true;
			for (auto const& node_slot_aggregate : renderable) {
				uint32_t node_culling_result = scm_camera_->cull_against_frustum(frustum_, bounding_box_vector[node_slot_aggregate.node_id_]);
				if (node_culling_result != 1) {
					if (draw) {
						glDrawArrays(scm::gl::PRIMITIVE_POINT_LIST, (node_slot_aggregate.slot_id_) * (GLsizei)surfels_per_node, surfels_per_node);
						//context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, (node_slot_aggregate.slot_id_) * (GLsizei)surfels_per_node, surfels_per_node); 
						rendered_splats_ += surfels_per_node;
						++rendered_nodes_;
					}
				}
			}
		}
		render_info_.rendered_splats_ = rendered_splats_;
		render_info_.rendered_nodes_ = rendered_nodes_;
		_plugin->rendering_ = false;

		GLStateSnapshot stateAfter = captureGLState();
		if ((!_initialized) && (stateBefore.vertexArrayBinding != stateAfter.vertexArrayBinding)) {
			compareGLStateSnapshots(stateBefore, stateAfter, "[Notify] PointsDrawCallback::drawImplementation()");
			pcl_resource_.vao_ = stateAfter.vertexArrayBinding;
			_initialized = true;
		}

		if (_plugin->dump_button->state()) {
			//_plugin->dump_button->setState(false);
			//stringstream stream;
			//renderInfo.getState()->print(stream);
			//std::cout << stream.str() << std::endl;
			//dumpStateSet(drawable->getStateSet());
			//dumpStateSet(_stateset);
			//dumpAllModes(drawable->getStateSet());
			//dumpAllModes(_stateset);
			//dumpStateSetWithInheritance(_stateset);
			//osg_util::dumpAllStateAttributes(_stateset);
			//glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &prevVAO);
			glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &prevVAO);
			printVAOAttributes(prevVAO);
			_plugin->dump_button->setState(false);
		}

		if (_initialized) {
			restoreGLBackup(stateBackup);
		}

		if (_plugin->notify_button->state()) {
			GLStateSnapshot stateAfter = captureGLState();
			compareGLStateSnapshots(stateBefore, stateAfter, "[Notify] PointsDrawCallback::drawImplementation()");
		}
	}
	osg::ref_ptr<osg::StateSet> _stateset;
	LamurePointCloudPlugin* _plugin;
	mutable bool _initialized;
};


struct PointsGeometry : public osg::Geometry
{
	PointsGeometry(osg::ref_ptr<osg::StateSet> stateset, LamurePointCloudPlugin* plugin):
		_plugin(plugin)
	{
		if (plugin->notify_button->state()) { std::cout << "[Notify] PointsGeometry()" << std::endl; }
		setUseDisplayList(false);
		setUseVertexBufferObjects(true);
		setUseVertexArrayObject(false);
		setDrawCallback(new PointsDrawCallback(stateset, plugin));
	}
	LamurePointCloudPlugin* _plugin;
};


struct TextCullCallback : public osg::Drawable::CullCallback {
	TextCullCallback(LamurePointCloudPlugin* plugin, osgText::Text* values, render_info* render_info)
		: _plugin(plugin),
		_values(values),
		_render_info(render_info)
	{
		_lastUpdateTime = std::chrono::steady_clock::now();
		_minInterval = std::chrono::milliseconds(100);
	}

	virtual bool cull(osg::NodeVisitor* nv, osg::Drawable* drawable, osg::RenderInfo* renderInfo) const override {
		auto now = std::chrono::steady_clock::now();
		if (now - _lastUpdateTime >= _minInterval) {
			scm::math::vec3d camPos = scm_camera_->get_cam_pos();

			scm::math::mat4d osg_model = matConv4D(cover->getInvBaseMat());
			scm::math::mat4d osg_view = _plugin->swapMiddleRows(matConv4D(cover->getViewerMat()));
			scm::math::mat4d osg_projection = matConv4D(osg_camera_->getProjectionMatrix());
			scm::math::mat4d osg_xform = matConv4D(cover->getObjectsXform()->getMatrix());
			scm::math::mat4d osg_scale = matConv4D(cover->getObjectsScale()->getMatrix());

			std::stringstream gl_modelview_ss;
			gl_modelview_ss << _plugin->gl_modelview_matrix;
			std::stringstream gl_projection_ss;
			gl_projection_ss << _plugin->gl_projection_matrix;
			std::stringstream gl_mvp_ss;
			gl_mvp_ss << _plugin->gl_projection_matrix * _plugin->gl_modelview_matrix;

			std::stringstream osg_modelview_ss;
			osg_modelview_ss << osg_view * osg_model;
			std::stringstream osg_projection_ss;
			osg_projection_ss << osg_projection;
			std::stringstream osg_mvp_ss;
			osg_mvp_ss << osg_projection * osg_view * osg_model;

			std::stringstream scm_modelview_ss;
			scm_modelview_ss << scm_camera_->get_view_matrix();
			std::stringstream scm_projection_ss;
			scm_projection_ss << scm_camera_->get_projection_matrix();
			std::stringstream scm_mvp_ss;
			scm_mvp_ss << scm_camera_->get_projection_matrix() * scm_camera_->get_view_matrix();


			std::stringstream value_ss;
			value_ss << "\n"
				<< std::fixed << std::setprecision(2)
				<< 1.0f / cover->frameDuration() << "\n"
				<< _render_info->rendered_nodes_ << "\n"
				<< _render_info->rendered_splats_ << "\n"
				<< _render_info->rendered_bounding_boxes_ << "\n\n\n"
				<< camPos.x << "\n"
				<< camPos.y << "\n"
				<< camPos.z << "\n\n\n\n"
				<< gl_modelview_ss.str() << "\n\n\n"
				<< gl_projection_ss.str() << "\n\n\n"
				<< gl_mvp_ss.str() << "\n\n\n"
				<< osg_modelview_ss.str() << "\n\n\n"
				<< osg_projection_ss.str() << "\n\n\n"
				<< osg_mvp_ss.str() << "\n\n\n"
				<< scm_modelview_ss.str() << "\n\n\n"
				<< scm_projection_ss.str() << "\n\n\n"
				<< scm_mvp_ss.str() << "\n";
			_values->setText(value_ss.str(), osgText::String::ENCODING_UTF8);
			_lastUpdateTime = now;
		}
		return false;
	}
	LamurePointCloudPlugin* _plugin;
	osg::ref_ptr<osgText::Text> _values;
	render_info* _render_info;
	mutable std::chrono::steady_clock::time_point _lastUpdateTime;
	std::chrono::milliseconds _minInterval;
};

struct TextGeode : public osg::Geode {
	TextGeode(LamurePointCloudPlugin* plugin): 
		_plugin(plugin) {
		osg::Quat rotation(osg::DegreesToRadians(90.0f), osg::Vec3(1.0f, 0.0f, 0.0f));
		osg::Vec4 color(1.0f, 1.0f, 1.0f, 1.0f);
		std::string font = coVRFileManager::instance()->getFontFile(NULL);
		float characterSize = 20.0f;

		osg::Vec3 pos_label(+traits->width * 0.5f, 0.0f, traits->height * 0.7f);
		osg::Vec3 pos_value = pos_label + osg::Vec3(100.0f, 0.0f, 0.0f);


		osg::ref_ptr<osgText::Text> label = new osgText::Text();
		label->setRotation(rotation);
		label->setColor(color);
		label->setFont(font);
		label->setCharacterSize(characterSize);
		label->setPosition(pos_label);
		std::stringstream label_ss;
		label_ss << "Rendering" << "\n"
			<< "FPS:" << "\n" 
			<< "Nodes:" << "\n" 
			<< "Splats:" << "\n" 
			<< "Boxes:" << "\n\n"
			<< "Frustum Position" << "\n"
			<< "X:" << "\n" 
			<< "Y:" <<  "\n" 
			<< "Z:" << "\n\n\n" 
			<< "GL ModelView:" << "\n\n\n\n\n\n"
			<< "GL Projection:" << "\n\n\n\n\n\n"
			<< "GL MVP:" << "\n\n\n\n\n\n"
			<< "OSG ModelView:" << "\n\n\n\n\n\n"
			<< "OSG Projection:" << "\n\n\n\n\n\n"
			<< "OSG MVP:" << "\n\n\n\n\n\n"
			<< "SCM ModelView:" << "\n\n\n\n\n\n"
			<< "SCM Projection:" << "\n\n\n\n\n\n"
			<< "SCM MVP:" << "\n";
		label->setText(label_ss.str(), osgText::String::ENCODING_UTF8);

		osg::ref_ptr<osgText::Text> value = new osgText::Text();
		value->setRotation(rotation);
		value->setColor(color);
		value->setFont(font);
		value->setCharacterSize(characterSize);
		value->setPosition(pos_value);
		std::stringstream value_ss;
		value_ss << "\n"
			<< "0.00:" << "\n"
			<< "0.00" << "\n"
			<< "0.00" << "\n"
			<< "0.00:" << "\n\n\n"
			<< "0.00" << "\n"
			<< "0.00" << "\n"
			<< "0.00" << "\n\n\n\n\n"
			<< "0.00" << "\n\n\n\n"
			<< "0.00" << "\n\n\n\n"
			<< "0.00" << "\n\n\n\n"
			<< "0.00" << "\n\n\n\n"
			<< "0.00" << "\n\n\n\n"
			<< "0.00" << "\n\n\n\n"
			<< "0.00" << "\n";
		value->setText(value_ss.str(), osgText::String::ENCODING_UTF8);

		this->addDrawable(label.get());
		this->addDrawable(value.get());

		value->setCullCallback(new TextCullCallback(_plugin, value.get(), &render_info_));
	}
	LamurePointCloudPlugin* _plugin;
};


struct InitDrawCallback : public osg::Drawable::DrawCallback {
	InitDrawCallback(osg::ref_ptr<osg::StateSet> stateset, LamurePointCloudPlugin* plugin)
		: _stateset(stateset),
		_plugin(plugin),
		_initialized(false)
	{
		if (plugin->notify_button->state()) { std::cout << "[Notify] InitDrawCallback()" << std::endl; }
	}

	virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const override
	{
		if (!_initialized) {
			_plugin->init_schism_objects();
			_plugin->HDC_draw = wglGetCurrentDC();
			_plugin->HGLRC_draw = wglGetCurrentContext();
			//scm_camera_->set_view_matrix(matConv4D(osg::Matrixd(renderInfo.getState()->getModelViewMatrix())));
			_plugin->create_pcl_resources();
			_initialized = true;
		}

		//GLdouble vm[16];
		//GLdouble pm[16];
		//glGetDoublev(GL_MODELVIEW_MATRIX, vm);
		//glGetDoublev(GL_PROJECTION_MATRIX, pm);
		//_plugin->gl_modelview_matrix = scm::math::mat4d(vm[0], vm[1], vm[2], vm[3], vm[4], vm[5], vm[6], vm[7], vm[8], vm[9], vm[10], vm[11], vm[12], vm[13], vm[14], vm[15]);
		//_plugin->gl_projection_matrix = scm::math::mat4d(pm[0], pm[1], pm[2], pm[3], pm[4], pm[5], pm[6], pm[7], pm[8], pm[9], pm[10], pm[11], pm[12], pm[13], pm[14], pm[15]);

		_plugin->gl_modelview_matrix = matConv4D(osg::Matrixd(renderInfo.getState()->getModelViewMatrix()));
		_plugin->gl_projection_matrix = matConv4D(osg::Matrixd(renderInfo.getState()->getProjectionMatrix()));

	}
	osg::ref_ptr<osg::StateSet> _stateset;
	LamurePointCloudPlugin* _plugin;
	mutable bool _initialized;
};


struct InitGeometry : public osg::Geometry {
	InitGeometry(osg::ref_ptr<osg::StateSet> stateset, LamurePointCloudPlugin* plugin):
		_plugin(plugin)
	{
		if (plugin->notify_button->state()) { std::cout << "[Notify] InitGeometry()" << std::endl; }
		osg::ref_ptr<osg::StateSet> stateSet = new osg::StateSet();
		setUseDisplayList(false);
		setUseVertexBufferObjects(true);
		setUseVertexArrayObject(false);
		setDrawCallback(new InitDrawCallback(stateset, plugin));
	}
	LamurePointCloudPlugin* _plugin;
};


void updateFrustumTransform(osg::ref_ptr<osg::MatrixTransform> matrixTransform, const osg::Vec3& translation) {
	osg::Matrix transMatrix = osg::Matrix::translate(translation);
	matrixTransform->setMatrix(transMatrix);
};


bool LamurePointCloudPlugin::init2() {
	std::cout << "init2()" << std::endl;
	std::cout << "getConfigEntry(COVER.Plugin.LamurePointCloud).c_str(): " << getConfigEntry("COVER.Plugin.LamurePointCloud").c_str() << std::endl;
	file = coVRFileManager::instance()->loadFile(getConfigEntry("COVER.Plugin.LamurePointCloud").c_str());

	std::cerr << "hostname: " << covise::coConfigConstants::getHostname() << std::endl;

	//Create main menu button
	menu = new ui::Menu("menu", this);
	menu->setText("Lamure");

	group = new ui::Group(menu, "group");
	model_grp = new ui::Group(menu, "model_grp");

	for (uint16_t m_id = 0; m_id < num_models_; m_id++) {
		std::filesystem::path pathObj(settings_.models_[m_id]);
		std::string filename = pathObj.filename().string();
		model_grp->add(new ui::Button(model_grp, filename));
		model_grp->child(m_id)->setShared(true);
	}

	pointcloud_button = new ui::Button(group, "pointcloud");
	boundingbox_button = new ui::Button(group, "boundingboxes");
	boundingbox_button_gl = new ui::Button(group, "boundingboxes_gl");
	frustum_button = new ui::Button(group, "frustum");
	frustum_button_gl = new ui::Button(group, "frustum_gl");
	coord_button = new ui::Button(group, "coordinates");
	coord_button_gl = new ui::Button(group, "coordinates_gl");
	sync_button = new ui::Button(group, "sync");
	notify_button = new ui::Button(group, "notify");
	text_button = new ui::Button(group, "text");
	dump_button = new ui::Button(group, "dump");

	pointcloud_button->setShared(true);
	boundingbox_button->setShared(true);
	boundingbox_button_gl->setShared(true);
	frustum_button->setShared(true);
	frustum_button_gl->setShared(true);
	coord_button->setShared(true);
	coord_button_gl->setShared(true);
	sync_button->setShared(true);
	notify_button->setShared(true);
	text_button->setShared(true);
	dump_button->setShared(true);

	plugin->LamureGroup = new osg::Group();
	plugin->LamureGroup->setName("LamureGroup");
	cover->getObjectsRoot()->addChild(plugin->LamureGroup);
	osg::ref_ptr<osg::MatrixTransform> frustumTransform = new osg::MatrixTransform;

	wait_for_opengl_context();

	plugin->HGLRC_init = wglGetCurrentContext();
	plugin->HDC_init = wglGetCurrentDC();
	plugin->HWND_init = WindowFromDC(plugin->HDC_init);

	plugin->HWND_cover = FindWindow(NULL, "COVER");
	plugin->HWND_opencover = FindWindow(NULL, "OpenCOVER");

	plugin->HDC_cover = GetDC(plugin->HWND_cover);
	plugin->HDC_opencover = GetDC(plugin->HWND_opencover);

	std::cout << "HGLRC_init: " << wglGetCurrentContext() << std::endl;

	std::cout << "HDC_init: " << wglGetCurrentDC() << std::endl;
	std::cout << "HDC_cover: " << GetDC(FindWindow(NULL, "COVER")) << std::endl;
	std::cout << "HDC_opencover: " << GetDC(FindWindow(NULL, "OpenCOVER")) << std::endl;

	std::cout << "HWND_init: " << WindowFromDC(wglGetCurrentDC()) << std::endl;
	std::cout << "HWND_cover: " << FindWindow(NULL, "COVER") << std::endl;
	std::cout << "HWND_opencover: " << FindWindow(NULL, "OpenCOVER") << std::endl;

	glEnable(GL_DEBUG_OUTPUT);
	glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
	glDebugMessageCallback(openglCallbackFunction, nullptr);

	//GLState glState;
	//BACKUP_GL_STATE(glState);
	//device_.reset(new scm::gl::render_device());
	//if (!device_) { std::cout << "error creating device" << std::endl; }
	//context_ = device_->main_context();
	//if (!context_) { std::cout << "error creating context" << std::endl; }
	//plugin->create_framebuffers();
	//plugin->init_render_states();
	
	//plugin->init_lamure_shader();
	plugin->init_camera();
	//plugin->create_box_resources();
	//plugin->create_frustum_resources();
	//plugin->create_pcl_resources();
	//RESTORE_GL_STATE(glState);

	// Slider für Kamera X-Position
	cameraPosXSlider = new ui::Slider(menu, "camera_translation_x");
	cameraPosXSlider->setText("Camera Translation X");
	cameraPosXSlider->setBounds(-2500.0, 2500.0);
	cameraPosXSlider->setValue(scm_camera_->get_cam_pos()[0]);
	cameraPosXSlider->setShared(true);
	cameraPosXSlider->setCallback([this, frustumTransform](double value, bool released) {
		scm_camera_->set_cam_pos(scm::math::vec3d(roundToDecimal(value, 3), scm_camera_->get_cam_pos()[1], scm_camera_->get_cam_pos()[2]));
		//updateFrustumTransform(frustumTransform, osg::Vec3(roundToDecimal(value, 3), scm_camera_->get_cam_pos()[1], scm_camera_->get_cam_pos()[2]));
	});
	
	// Slider für Kamera Y-Position
	cameraPosYSlider = new ui::Slider(menu, "camera_translation_y");
	cameraPosYSlider->setText("Camera Translation Y");
	cameraPosYSlider->setBounds(-2500.0, 2500.0);
	cameraPosYSlider->setValue(scm_camera_->get_cam_pos()[1]);
	cameraPosYSlider->setShared(true);
	cameraPosYSlider->setCallback([this, frustumTransform](double value, bool released) {
		scm_camera_->set_cam_pos(scm::math::vec3d(scm_camera_->get_cam_pos()[0], roundToDecimal(value,3), scm_camera_->get_cam_pos()[2]));
		//updateFrustumTransform(frustumTransform, osg::Vec3(scm_camera_->get_cam_pos()[0], roundToDecimal(value, 3), scm_camera_->get_cam_pos()[2]));
	});

	// Slider für Kamera Z-Position
	cameraPosZSlider = new ui::Slider(menu, "camera_translation_z");
	cameraPosZSlider->setText("Camera Translation Z");
	cameraPosZSlider->setBounds(-2500.0, 2500.0);
	cameraPosZSlider->setValue(scm_camera_->get_cam_pos()[2]);
	cameraPosZSlider->setShared(true);
	cameraPosZSlider->setCallback([this, frustumTransform](double value, bool released) {
		scm_camera_->set_cam_pos(scm::math::vec3d(scm_camera_->get_cam_pos()[0], scm_camera_->get_cam_pos()[1], roundToDecimal(value,3)));
		//updateFrustumTransform(frustumTransform, osg::Vec3(scm_camera_->get_cam_pos()[0], scm_camera_->get_cam_pos()[1], roundToDecimal(value, 3)));
	});

	// Slider für Modell X-Position
	modelPosXSlider = new ui::Slider(menu, "model_translation_x");
	modelPosXSlider->setText("Model Translation X");
	modelPosXSlider->setBounds(-2500.0, 2500.0);
	modelPosXSlider->setValue(settings_.model_tl_.x);
	modelPosXSlider->setShared(true);
	modelPosXSlider->setCallback([this](double value, bool released) { settings_.model_tl_.x = value; });

	// Slider für Modell Y-Position
	modelPosYSlider = new ui::Slider(menu, "model_translation_y");
	modelPosYSlider->setText("Model Translation Y");
	modelPosYSlider->setBounds(-2500.0, 2500.0);
	modelPosYSlider->setValue(settings_.model_tl_.y);
	modelPosYSlider->setShared(true);
	modelPosYSlider->setCallback([this](double value, bool released) { settings_.model_tl_.y = value; });

	// Slider für Modell Z-Position
	modelPosZSlider = new ui::Slider(menu, "model_translation_z");
	modelPosZSlider->setText("Model Translation Z");
	modelPosZSlider->setBounds(-2500.0, 2500.0);
	modelPosZSlider->setValue(settings_.model_tl_.z);
	modelPosZSlider->setShared(true);
	modelPosZSlider->setCallback([this](double value, bool released) { settings_.model_tl_.z = value; });

	// Slider für Rotation um X-Achse
	rotationXSlider = new ui::Slider(menu, "model_rotation_x");
	rotationXSlider->setText("Model Rotation X");
	rotationXSlider->setBounds(-180.0, 180.0);
	rotationXSlider->setValue(rotationAngles.x);
	rotationXSlider->setShared(true);
	rotationXSlider->setCallback([this](double value, bool released) { rotationAngles.x = value; updateModelRotation(); });

	// Slider für Rotation um Y-Achse
	rotationYSlider = new ui::Slider(menu, "model_rotation_y");
	rotationYSlider->setText("Model Rotation Y");
	rotationYSlider->setBounds(-180.0, 180.0);
	rotationYSlider->setValue(rotationAngles.y);
	rotationYSlider->setShared(true);
	rotationYSlider->setCallback([this](double value, bool released) { rotationAngles.y = value; updateModelRotation(); });

	// Slider für Rotation um Z-Achse
	rotationZSlider = new ui::Slider(menu, "model_rotation_z");
	rotationZSlider->setText("Model Rotation Z");
	rotationZSlider->setBounds(-180.0, 180.0);
	rotationZSlider->setValue(rotationAngles.z);
	rotationZSlider->setShared(true);
	rotationZSlider->setCallback([this](double value, bool released) { rotationAngles.z = value; updateModelRotation(); });

	maxRadiusSlider = new ui::Slider(menu, "max_radius");
	maxRadiusSlider->setText("max. radius");
	maxRadiusSlider->setBounds(0.1, 5.0);
	maxRadiusSlider->setValue(settings_.max_radius_);
	maxRadiusSlider->setShared(true);
	maxRadiusSlider->setCallback([this](double value, bool released) { settings_.max_radius_ = value; });

	scaleRadiusSlider = new ui::Slider(menu, "scale_radius");
	scaleRadiusSlider->setText("scale radius");
	scaleRadiusSlider->setBounds(0.1, 5.0);
	scaleRadiusSlider->setValue(settings_.scale_radius_);
	scaleRadiusSlider->setShared(true);
	scaleRadiusSlider->setCallback([this](double value, bool released) { settings_.scale_radius_ = value; });

	init_stateset = new osg::StateSet();
	pointcloud_stateset = new osg::StateSet();
	boundingbox_stateset = new osg::StateSet();
	boundingbox_stateset_gl = new osg::StateSet();
	frustum_stateset = new osg::StateSet();
	frustum_stateset_gl = new osg::StateSet();
	coord_stateset = new osg::StateSet();
	coord_stateset_gl = new osg::StateSet();
	text_stateset = new osg::StateSet();

	init_geode = new osg::Geode();
	pointcloud_geode = new osg::Geode();
	boundingbox_geode = new osg::Geode();
	boundingbox_geode_gl = new osg::Geode();
	frustum_geode = new osg::Geode();
	frustum_geode_gl = new osg::Geode();
	coord_geode = new osg::Geode();
	coord_geode_gl = new osg::Geode();
	text_geode = new TextGeode(plugin);

	pointcloud_button->setState(false);
	boundingbox_button->setState(false);
	boundingbox_button_gl->setState(false);
	frustum_button->setState(false);
	frustum_button_gl->setState(false);
	coord_button->setState(false);
	coord_button_gl->setState(false);
	text_button->setState(false);
	notify_button->setState(true);

	pointcloud_geode->setNodeMask(0);
	boundingbox_geode->setNodeMask(0);
	boundingbox_geode_gl->setNodeMask(0);
	frustum_geode->setNodeMask(0);
	frustum_geode_gl->setNodeMask(0);
	coord_geode->setNodeMask(0);
	coord_geode_gl->setNodeMask(0);
	text_geode->setNodeMask(0);

	pointcloud_button->setCallback([this](bool state) {
		pointcloud_geode->setNodeMask(state ? 0xFFFFFFFF : 0x0);
		});

	boundingbox_button->setCallback([this](bool state) {
		boundingbox_geode->setNodeMask(state ? 0xFFFFFFFF : 0x0);
		});

	boundingbox_button_gl->setCallback([this](bool state) {
		boundingbox_geode_gl->setNodeMask(state ? 0xFFFFFFFF : 0x0);
		});

	frustum_button->setCallback([this](bool state) {
		frustum_geode->setNodeMask(state ? 0xFFFFFFFF : 0x0);
		});

	frustum_button_gl->setCallback([this](bool state) {
		frustum_geode_gl->setNodeMask(state ? 0xFFFFFFFF : 0x0);
		});

	coord_button->setCallback([this](bool state) {
		coord_geode->setNodeMask(state ? 0xFFFFFFFF : 0x0);
		});

	coord_button_gl->setCallback([this](bool state) {
		coord_geode_gl->setNodeMask(state ? 0xFFFFFFFF : 0x0);
		});

	text_button->setCallback([this](bool state) {
		text_geode->setNodeMask(state ? 0xFFFFFFFF : 0x0);
		});

	dump_button->setCallback([this](bool state) {
		});


	osg::ref_ptr<osg::Program> vis_line = new osg::Program();
	vis_line->addShader(new osg::Shader(osg::Shader::VERTEX, vis_line_vs_source));
	vis_line->addShader(new osg::Shader(osg::Shader::FRAGMENT, vis_line_fs_source));

	osg::ref_ptr<osg::Program> vis_line_bb = new osg::Program();
	vis_line_bb->addShader(new osg::Shader(osg::Shader::VERTEX, vis_line_bb_vs_source));
	vis_line_bb->addShader(new osg::Shader(osg::Shader::FRAGMENT, vis_line_bb_fs_source));

	osg::ref_ptr<osg::Program> vis_xyz = new osg::Program();
	vis_xyz->addShader(new osg::Shader(osg::Shader::VERTEX, vis_xyz_vs_source));
	vis_xyz->addShader(new osg::Shader(osg::Shader::GEOMETRY, vis_xyz_gs_source));
	vis_xyz->addShader(new osg::Shader(osg::Shader::FRAGMENT, vis_xyz_fs_source));

	osg::ref_ptr<osg::Program> vis_box = new osg::Program();
	vis_box->addShader(new osg::Shader(osg::Shader::VERTEX, vis_box_vs_source));
	vis_box->addShader(new osg::Shader(osg::Shader::GEOMETRY, vis_box_gs_source));
	vis_box->addShader(new osg::Shader(osg::Shader::FRAGMENT, vis_box_fs_source));

	//pointcloud_stateset->setAttributeAndModes(vis_xyz, osg::StateAttribute::ON);
	//pointcloud_stateset->addUniform(new osg::Uniform(osg::Uniform::FLOAT_MAT4, "mvp_matrix"));
	//add_pointcloud_uniforms(pointcloud_stateset);

	//boundingbox_stateset->setAttributeAndModes(vis_line_bb, osg::StateAttribute::ON);
	//boundingbox_stateset->addUniform(new osg::Uniform("mvp_matrix", osg::Matrixf()));
	//boundingbox_stateset->addUniform(new osg::Uniform("in_color", osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f)));
	
	//frustum_stateset->setAttributeAndModes(vis_line_bb, osg::StateAttribute::ON);
	//frustum_stateset->addUniform(new osg::Uniform("mvp_matrix", osg::Matrixf()));
	//frustum_stateset->addUniform(new osg::Uniform("in_color", osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f)));
	
	//coord_stateset->setAttributeAndModes(vis_line_bb, osg::StateAttribute::ON);
	//coord_stateset->addUniform(new osg::Uniform("mvp_matrix", osg::Matrixf()));
	//coord_stateset->addUniform(new osg::Uniform("in_color", osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f)));

	coord_stateset->setRenderBinDetails(5, "RenderBin");
	coord_stateset->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

	text_stateset->setRenderBinDetails(10, "RenderBin");

	boundingbox_stateset->setAttributeAndModes(vis_box, osg::StateAttribute::ON);
	boundingbox_stateset->addUniform(new osg::Uniform("modelMatrix", osg::Matrixf()));
	boundingbox_stateset->addUniform(new osg::Uniform("viewMatrix", osg::Matrixf()));
	boundingbox_stateset->addUniform(new osg::Uniform("projectionMatrix", osg::Matrixf()));
	boundingbox_stateset->addUniform(new osg::Uniform("in_color", osg::Vec4(1.0, 1.0, 0.0, 1.0)));

	init_geode->setStateSet(init_stateset.get());
	pointcloud_geode->setStateSet(pointcloud_stateset.get());
	boundingbox_geode->setStateSet(boundingbox_stateset.get());
	boundingbox_geode_gl->setStateSet(boundingbox_stateset_gl.get());
	frustum_geode->setStateSet(frustum_stateset.get());
	frustum_geode_gl->setStateSet(frustum_stateset_gl.get());
	coord_geode->setStateSet(coord_stateset.get());
	coord_geode_gl->setStateSet(coord_stateset_gl.get());
	text_geode->setStateSet(text_stateset.get());

	init_geode->setName("init_geode");

	updateFrustumTransform(frustumTransform, osg::Vec3(scm_camera_->get_cam_pos()[0], scm_camera_->get_cam_pos()[1], scm_camera_->get_cam_pos()[2]));
	LamureGroup->addChild(frustumTransform);
	frustumTransform->addChild(frustum_geode);
	LamureGroup->addChild(frustum_geode_gl);
	LamureGroup->addChild(coord_geode);
	LamureGroup->addChild(coord_geode_gl);
	LamureGroup->addChild(boundingbox_geode);
	LamureGroup->addChild(boundingbox_geode_gl);
	LamureGroup->addChild(pointcloud_geode);
	LamureGroup->addChild(init_geode);
	hud_camera->addChild(text_geode);

	init_geometry = new InitGeometry(init_stateset, plugin);
	pointcloud_geometry = new PointsGeometry(pointcloud_stateset, plugin);
	boundingbox_geometry = new BoundingBoxGeometry(boundingbox_stateset, plugin);
	boundingbox_geometry_gl = new BoundingBoxGeometryGL(boundingbox_stateset_gl, plugin);
	frustum_geometry = new FrustumGeometry(frustum_stateset, plugin);
	frustum_geometry_gl = new FrustumGeometryGL(frustum_stateset_gl, plugin);
	coord_geometry = new CoordGeometry(coord_stateset, plugin);
	coord_geometry_gl = new CoordGeometryGL(coord_stateset_gl, plugin);

	init_geode->addDrawable(init_geometry);
	pointcloud_geode->addDrawable(pointcloud_geometry);
	boundingbox_geode->addDrawable(boundingbox_geometry);
	boundingbox_geode_gl->addDrawable(boundingbox_geometry_gl);
	frustum_geode->addDrawable(frustum_geometry);
	frustum_geode_gl->addDrawable(frustum_geometry_gl);
	coord_geode->addDrawable(coord_geometry);
	coord_geode_gl->addDrawable(coord_geometry_gl);

	//addAxesToScene(osg::Vec3(0.f, 0.f, 0.f), 0.0f);
	//coVRConfig::instance()->windows[0].context->getState()->print(std::cout);
	//VRViewer::instance()->statsDisplay->showStats(coVRStatsDisplay::VIEWER_SCENE_STATS, VRViewer::instance());
	//VRSceneGraph::instance()->viewAll();
	//printNodePath(text_geode);

	//osg::ref_ptr<osg::Geometry> boundingBoxGeom = new osg::Geometry;
	//boundingBoxGeom->setUseDisplayList(false);
	//boundingBoxGeom->setUseVertexBufferObjects(true);
	//boundingBoxGeom->setUseVertexArrayObject(false);
	//boundingBoxGeom->setVertexArray(new osg::Vec3Array);
	//boundingBoxGeom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 0));
	//boundingBoxGeom->setStateSet(boundingbox_stateset.get());
	//boundingBoxGeom->setUpdateCallback(new LamureBoundingBoxUpdater(this));
	//osg::ref_ptr<osg::Geode> bboxGeode = new osg::Geode;
	//bboxGeode->addDrawable(boundingBoxGeom.get());
	//LamureGroup->addChild(bboxGeode.get());

	if (plugin->notify_button->state()) { 
		std::cout << "[Notify] === SceneGraph ===" << std::endl;
		printChildNodes(osg_camera_, 5);
	}

	return 1;
}


osg::Matrix getCompleteTransformationMatrix(osg::Node* node)
{
	osg::NodePathList paths = node->getParentalNodePaths();
	std::cout << paths.size() << std::endl;
	if (!paths.empty())
	{
		osg::Matrix completeMatrix = osg::Matrix::identity();
		const osg::NodePath& path = paths.front();
		for (osg::NodePath::const_iterator itr = path.begin(); itr != path.end(); ++itr)
		{
			osg::MatrixTransform* mt = dynamic_cast<osg::MatrixTransform*>(*itr);
			if (mt)
			{
				completeMatrix.preMult(mt->getMatrix());
			}
		}
		return completeMatrix;
	}
	return osg::Matrix::identity();
}


unsigned int counter = 0;
void LamurePointCloudPlugin::preFrame() {
	if (cover->getPointerButton()->getState() == 1 && counter == 0) {
		counter = counter + 1;

		//VRViewer::instance()->updateViewerMat(mat);
		//osg::Vec3 newPos(0.0f, -1000.0f, 0.0f);            // Neue Kameraposition
		//osg::Vec3 target(0.0f, -999.0f, 0.0f); // Punkt, auf den die Kamera blicken soll
		//osg::Vec3 up(0.0f, 0.0f, 1.0f);        // Up-Vektor (z.B. Z-Achse)

		//osg::Matrix viewMatrix;
		//viewMatrix.makeLookAt(newPos, target, up);
		//osg_camera_->setViewMatrix(viewMatrix);
		//VRViewer::instance()->updateViewerMat(viewMatrix);
	}
	else if (cover->getPointerButton()->getState() == 1) {
		//std::cout << "gl_view_matrix: " << std::endl << gl_view_matrix << std::endl << std::endl;
		//std::cout << "gl_projection_matrix: " << std::endl << gl_projection_matrix << std::endl << std::endl;
	}

	if (plugin->sync_button->state() == 1) {
		scm_camera_->set_view_matrix(gl_modelview_matrix);
	}
}


unsigned int counter2 = 0;
void LamurePointCloudPlugin::postFrame() {
	if (plugin->notify_button->state() == 0) {
		osg_camera_->getGraphicsContext()->getState()->setCheckForGLErrors(osg::State::NEVER_CHECK_GL_ERRORS);
	}
	else {
		osg_camera_->getGraphicsContext()->getState()->setCheckForGLErrors(osg::State::ONCE_PER_FRAME);
	}
	if (plugin->sync_button->state() == 1 && cover->getPointerButton()->getState()) {
		//std::cout << "getCompleteTransformationMatrix:" << std::endl << matConv4D(getCompleteTransformationMatrix(LamureGroup)) << std::endl;
		//std::cout << "osg_camera_->getViewMatrix():" << std::endl << matConv4D(osg_camera_->getViewMatrix()) << std::endl;
		//std::cout << "osg_camera_->getProjectionMatrix():" << std::endl << matConv4D(osg_camera_->getProjectionMatrix()) << std::endl;
	}
}


unsigned int counter1 = 0;
bool LamurePointCloudPlugin::update() {
	return 1;
}


struct SyncCameraToNodeCallback : public osg::NodeCallback {
	SyncCameraToNodeCallback(osg::MatrixTransform* srcTrafo, osg::Camera* camera)
		: _srcTrafo(srcTrafo), _camera(camera) {}

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) override
	{
		if (_srcTrafo.valid() && _camera.valid())
		{
				osg::Matrix worldMat = _srcTrafo->getMatrix();
				_camera->setViewMatrix(osg::Matrix::inverse(worldMat));
				std::cout << "SyncCameraToNodeCallback " << std::endl;
		}
		traverse(node, nv);
	}
	osg::observer_ptr<osg::MatrixTransform> _srcTrafo;
	osg::observer_ptr<osg::Camera> _camera;
};


void LamurePointCloudPlugin::init_schism_objects() {
	if (!device_) {
		device_.reset(new scm::gl::render_device());
		if (!device_) { std::cout << "error creating device" << std::endl; }

		plugin->create_framebuffers();
		plugin->init_render_states();
		plugin->init_lamure_shader();

		if (plugin->notify_button->state()) {
			std::cout << "[Notify] init_schism_objects()" << std::endl;
			std::ostringstream oss;
			device_->dump_memory_info(oss);
			std::cout << oss.str() << std::endl;
			//std::cout << (*device_);
			scm::gl::render_device::device_capabilities capa = device_->capabilities();
		}
	}
	if (!context_) {
		context_ = device_->main_context();
		if (!context_) { std::cout << "error creating context" << std::endl; }
	}
}


void LamurePointCloudPlugin::init_camera() {
	osg_camera_ = VRViewer::instance()->getCamera();
	lmr_ctx = osg_camera_->getGraphicsContext()->getState()->getContextID();

	double left, right, bottom, top, zNear, zFar;
	osg_camera_->getProjectionMatrixAsFrustum(left, right, bottom, top, zNear, zFar);

	double look_dist = 1.0;
	osg::Vec3d eye, center, up;
	osg_camera_->getViewMatrixAsLookAt(eye, center, up, look_dist);
	double fovy = 2.0 * std::atan((top - bottom) / (2.0 * zNear));
	double aspect = (right - left) / (top - bottom);

	std::cout << "OSG Projection Matrix as Frustum:" << std::endl;
	std::cout << "Left:  " << left << std::endl;
	std::cout << "Right: " << right << std::endl;
	std::cout << "Bottom:" << bottom << std::endl;
	std::cout << "Top:   " << top << std::endl;
	std::cout << "Near:  " << zNear << std::endl;
	std::cout << "Far:   " << zFar << std::endl;
	std::cout << "fovy (in Radiant): " << fovy << std::endl;
	std::cout << "fovy (in Grad): " << fovy * (180.0 / M_PI) << std::endl;
	std::cout << "Aspect Ratio: " << aspect << std::endl;

	std::cout << "OSG View Matrix as LookAt:" << std::endl;
	std::cout << "Eye:    (" << eye.x() << ", " << eye.y() << ", " << eye.z() << ")" << std::endl;
	std::cout << "Center: (" << center.x() << ", " << center.y() << ", " << center.z() << ")" << std::endl;
	std::cout << "Up:     (" << up.x() << ", " << up.y() << ", " << up.z() << ")" << std::endl;
	std::cout << "lookDistance: " << look_dist << std::endl;

	osg::Matrixd view = osg_camera_->getViewMatrix();
	osg::Matrixd proj = osg_camera_->getProjectionMatrix();

	scm_camera_ = new lamure::ren::camera((lamure::view_t)lmr_ctx, zNear, zFar, matConv4D(view), matConv4D(proj));
	
	//scm_camera_ = new lamure::ren::camera((lamure::view_t)lmr_ctx, left, right, bottom, top, zNear, zFar, vecConv3D(eye), vecConv3D(center), vecConv3D(up), look_dist);

	osgViewer::Viewer::Windows windows;
	VRViewer::instance()->getWindows(windows);
	osgViewer::GraphicsWindow* window = windows.front();
	hud_camera = new osg::Camera();
	hud_camera->setName("hud_camera");
	hud_camera->setGraphicsContext(window);
	hud_camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	hud_camera->setProjectionResizePolicy(osg::Camera::FIXED);
	hud_camera->setViewMatrix(osg_camera_->getViewMatrix());
	hud_camera->setProjectionMatrix(osg_camera_->getProjectionMatrix());
	hud_camera->setViewport(0, 0, traits->width, traits->height);
	hud_camera->setRenderOrder(osg::Camera::POST_RENDER, 2);
	hud_camera->setClearMask(0);
	hud_camera->setRenderer(new osgViewer::Renderer(hud_camera.get()));
	osg_camera_->addChild(hud_camera.get());

	scm::math::vec3f temp_center = scm::math::vec3f::zero();
	scm::math::vec3f root_min_temp = scm::math::vec3f::zero();
	scm::math::vec3f root_max_temp = scm::math::vec3f::zero();

	for (lamure::model_t model_id = 0; model_id < num_models_; ++model_id) {
		lamure::model_t m_id = lamure::ren::controller::get_instance()->deduce_model_id(std::to_string(model_id));

		auto root_bb = lamure::ren::model_database::get_instance()->get_model(model_id)->get_bvh()->get_bounding_boxes()[0];

		model_info_.root_bb_min.push_back(scm::math::mat4f(model_info_.model_transformations_[model_id]) * scm::math::vec4f(root_bb.min_vertex()[0], root_bb.min_vertex()[1], root_bb.min_vertex()[2], 1));
		model_info_.root_bb_max.push_back(scm::math::mat4f(model_info_.model_transformations_[model_id]) * scm::math::vec4f(root_bb.max_vertex()[0], root_bb.max_vertex()[1], root_bb.max_vertex()[2], 1));
		model_info_.root_center.push_back(scm::math::mat4f(model_info_.model_transformations_[model_id]) * scm::math::vec4f(root_bb.center()[0], root_bb.center()[1], root_bb.center()[2], 1));

		temp_center += model_info_.root_center.back();

		if (model_info_.root_bb_min[model_id][0] < root_min_temp[0]) { root_min_temp[0] = model_info_.root_bb_min[model_id][0]; }
		if (model_info_.root_bb_min[model_id][1] < root_min_temp[1]) { root_min_temp[1] = model_info_.root_bb_min[model_id][1]; }
		if (model_info_.root_bb_min[model_id][2] < root_min_temp[2]) { root_min_temp[2] = model_info_.root_bb_min[model_id][2]; }

		if (model_info_.root_bb_max[model_id][0] > root_min_temp[0]) { root_min_temp[0] = model_info_.root_bb_max[model_id][0]; }
		if (model_info_.root_bb_max[model_id][1] > root_min_temp[1]) { root_min_temp[1] = model_info_.root_bb_max[model_id][1]; }
		if (model_info_.root_bb_max[model_id][2] > root_min_temp[2]) { root_min_temp[2] = model_info_.root_bb_max[model_id][2]; }
	}

	model_info_.models_center = temp_center / num_models_;
	model_info_.models_min = root_min_temp;
	model_info_.models_max = root_max_temp;

	/*scm_camera_ = new lamure::ren::camera(
		(lamure::view_t)lmr_ctx,
		scm::math::vec3f(model_info_.models_center),
		settings_.fov_,
		float(traits->width) / float(traits->height),
		float(settings_.near_plane_),
		float(settings_.far_plane_)
	);*/

	//cover->getObjectsRoot()->addChild(pcl_camera.get());

	//VRViewer::instance()->addSlave(pcl_camera.get(), true);
	//osg::ref_ptr<osg::MatrixTransform> transformNode = new osg::MatrixTransform;
	//transformNode->setMatrix(cover->getObjectsRoot()->getWorldMatrices().front());
	//pcl_camera->setUpdateCallback(
	//	new SyncCameraToNodeCallback(transformNode.get(), pcl_camera.get())
	//);

	//std::cout << matConv4D(osg_camera_->getViewMatrix()) << std::endl;
	//std::cout << matConv4D(osg_camera_->getProjectionMatrix()) << std::endl;

	//osg::ref_ptr<osg::Texture2D> colorTexture = new osg::Texture2D();
	//colorTexture->setTextureSize(traits->width, traits->height);
	//colorTexture->setInternalFormat(GL_RGBA);
	//colorTexture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
	//colorTexture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
	//osg::ref_ptr<osg::Texture2D> depthTexture = new osg::Texture2D();
	//depthTexture->setTextureSize(traits->width, traits->height);
	//depthTexture->setInternalFormat(GL_DEPTH_COMPONENT24);
	//depthTexture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
	//depthTexture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
	//// Anfügen der Texturen an die Kamera
	//pcl_camera->attach(osg::Camera::COLOR_BUFFER, colorTexture.get());
	//pcl_camera->attach(osg::Camera::DEPTH_BUFFER, depthTexture.get());

	
	//plugin->text_geode = new osg::Geode();
	////pcl_camera->addChild(geode.get());
	//pcl_camera->addChild(plugin->text_geode);
	//osg::Vec4 colorFR(1.0f, 1.0f, 1.0f, 1.0f);
	//std::string font = coVRFileManager::instance()->getFontFile(NULL);
	//float leftPos = covise::coCoviseConfig::getFloat("leftPos", "COVER.Stats", 10.0f);
	//float startBlocks = 150.0f;
	//float characterSize = 20.0f;
	//osg::Vec3 pos(leftPos, 0.0f, 0.0f);
	//osg::ref_ptr<osgText::Text> text_test = new osgText::Text;
	//plugin->text_geode->addDrawable(text_test.get());
	//text_test->setColor(colorFR);
	//text_test->setFont(font);
	//text_test->setCharacterSize(characterSize);
	//text_test->setPosition(pos);
	//text_test->setText("Frames/s:X", osgText::String::ENCODING_UTF8);
	//pos.x() = text_test->getBoundingBox().xMax();
	//printNodePath(plugin->text_geode);

	//screen_quad_.reset(new scm::gl::quad_geometry(device_, scm::math::vec2f(-1.0f, -1.0f), scm::math::vec2f(1.0f, 1.0f)));

	//coVRConfig::instance()->channels[0].camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
}


void LamurePointCloudPlugin::sync_cameras() {
	//double fovy, aspectRatio, zNear, zFar;
	//bool proj_success = osg_camera_->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
	//osg::Matrix osg_proj_ = osg_camera_->getProjectionMatrix();
	//lamure_camera_->set_projection_matrix(fovy, aspectRatio, zNear, zFar);
	//scm::math::mat4d lmr_proj_mat = lamure_camera_->get_hp_projection_matrix();

	// multiplied matrices from scene node to objects root node
	osg::Matrixd base_mat = cover->getBaseMat();
	// object's transformation matrix
	osg::Matrixd x_form_mat = cover->getXformMat();
	// position and orientation of user
	osg::Matrixd viewer_mat = cover->getViewerMat();


	scm::math::mat4d model_matrix = model_info_.model_transformations_[0];
	scm::gl::frustum frustum_ = scm_camera_->get_frustum();
	scm::math::mat4f vm = createSwapYZMatrix() * scm_camera_->get_view_matrix();

	//osg_camera_->setViewMatrix(matConv4F(scm_camera_->get_view_matrix()) * cover->getInvBaseMat());
	//osg_camera_->setProjectionMatrix(matConv4F(scm_camera_->get_projection_matrix()));

	//cover->setXformMat(matConv4F(vm));
	//VRViewer::instance()->updateViewerMat(matConv4F(vm));
	//VRViewer::instance()->getCamera()->setViewMatrix(matConv4F(vm));

	//osg::Camera* cam_ = VRViewer::instance()->getCamera();
	//osg::Matrixd view_mat_osg = cam_->getViewMatrix();
	//osg::Matrixd proj_mat_osg = cam_->getProjectionMatrix();
	//scm::math::mat4d view_matrix_osg = matConv4D(view_mat_osg);
	//scm::math::mat4d proj_matrix_osg = matConv4D(proj_mat_osg);
	//scm::math::mat4d base_matrix = matConv4D(base_mat);
	//scm::math::mat4d x_form_matrix = matConv4D(x_form_mat);
	//scm::math::mat4d viewer_matrix = matConv4D(viewer_mat);


	//osg::Matrixd objects_scale_matrix = cover->getObjectsScale()->getMatrix();
	//osg::Matrixd objects_xform_matrix = cover->getObjectsXform()->getMatrix();

	//std::cout << "objects_scale_matrix" << std::endl << matConv4D(objects_scale_matrix) << std::endl << std::endl;
	//std::cout << "objects_xform_matrix" << std::endl << matConv4D(objects_xform_matrix )<< std::endl << std::endl;

	//std::cout << "base_matrix" << std::endl << base_matrix << std::endl << std::endl;
	//std::cout << "x_form_matrix" << std::endl << x_form_matrix << std::endl << std::endl;
	//std::cout << "viewer_matrix" << std::endl << viewer_matrix << std::endl << std::endl;

	//std::cout << "viewer_matrix * x_form_matrix" << std::endl << viewer_matrix * x_form_matrix << std::endl << std::endl;

	//std::cout << "worldMatrix" << std::endl << matConv4F(getCompleteTransformationMatrix(plugin->init_geode)) << std::endl << std::endl;

	//std::cout << "gl_modelview_matrix:" << std::endl << gl_modelview_matrix << std::endl;
	//std::cout << "swapMiddleRows(osg_camera_->getViewMatrix()):" << std::endl << swapMiddleRows(matConv4D(osg_camera_->getViewMatrix())) << std::endl;

	//std::cout << "gl_projection_matrix:" << std::endl << gl_projection_matrix << std::endl;
	//std::cout << "swapMiddleColumns(osg_camera_->getProjectionMatrix()):" << std::endl << swapMiddleColumns(matConv4D(osg_camera_->getProjectionMatrix())) << std::endl;


	//std::cout << "gl_mvp" << std::endl << gl_projection_matrix * gl_modelview_matrix << std::endl;

	//std::cout << "osg_mvp" << std::endl << matConv4D(osg_camera_->getProjectionMatrix()) * matConv4D(osg_camera_->getViewMatrix()) << std::endl;

	//std::cout << "gl_modelview_matrix" << std::endl << gl_modelview_matrix << std::endl << std::endl;
	//std::cout << "gl_projection_matrix" << std::endl << gl_projection_matrix << std::endl << std::endl;

	//std::cout << "scm_camera_->get_view_matrix()" << std::endl << scm_camera_->get_view_matrix() << std::endl << std::endl;
	//std::cout << "scm_camera_->get_projection_matrix()" << std::endl << scm_camera_->get_projection_matrix() << std::endl << std::endl;

	//std::cout << "VRViewer::instance()->getViewerMat()" << std::endl << matConv4F(VRViewer::instance()->getViewerMat()) << std::endl << std::endl;

	//scm::math::mat4d model_matrix = model_info_.model_transformations_[0];
	//scm::math::mat4d model_view_matrix = view_matrix_osg * model_matrix;
	//scm::math::mat4d inv_mv_matrix = scm::math::transpose(scm::math::inverse(model_view_matrix));
	//scm::math::mat4d mvp_matrix = gl_projection_matrix * gl_modelview_matrix;

	//const scm::math::mat4d viewport_scale = scm::math::make_scale(traits->width * 0.5, traits->height * 0.5, 0.5);
	//const scm::math::mat4d viewport_translate = scm::math::make_translation(1.0, 1.0, 1.0);
	//const scm::math::mat4d model_to_screen = viewport_scale * viewport_translate;

	//scm::gl::frustum frustum_ = scm_camera_->get_frustum();

	//std::cout << "inv_mv_matrix" << std::endl << inv_mv_matrix << std::endl << std::endl;
	//std::cout << "mvp_matrix" << std::endl << mvp_matrix << std::endl << std::endl;

	//std::cout << "scm_camera_->get_cam_pos()" << std::endl << scm_camera_->get_cam_pos() << std::endl << std::endl;
	//std::cout << "scm_camera_->get_cam_matrix()" << std::endl << scm_camera_->get_cam_matrix() << std::endl << std::endl;
	//std::cout << "scm_camera_->get_view_matrix()" << std::endl << scm_camera_->get_view_matrix() << std::endl << std::endl;

	//no change

	//std::cout << "inv_mv_matrix" << std::endl << inv_mv_matrix << std::endl << std::endl;
	//std::cout << "mvp_matrix_osg" << std::endl << proj_matrix_osg*view_matrix_osg << std::endl << std::endl;
	//std::cout << "view_matrix_osg" << std::endl << view_matrix_osg << std::endl << std::endl;
	//std::cout << "proj_matrix_osg" << std::endl << proj_matrix_osg << std::endl << std::endl;
	//std::cout << "model_view_matrix" << std::endl << model_view_matrix << std::endl << std::endl;
	//std::cout << "model_matrix" << std::endl << model_matrix << std::endl << std::endl;

	//std::cout << "viewport_scale" << std::endl << viewport_scale << std::endl << std::endl;
	//std::cout << "viewport_translate" << std::endl << viewport_translate << std::endl << std::endl;
	//std::cout << "model_to_screen" << std::endl << model_to_screen << std::endl << std::endl;
}


void LamurePointCloudPlugin::create_pcl_resources() {
	if (plugin->notify_button->state()) {
		std::cout << "[Notify] create_pcl_resources() " << std::endl;
		std::cout << "[Notify] wglGetCurrentContext(): " << wglGetCurrentContext() << std::endl;
	}

	unsigned int program = glCreateProgram();
	unsigned int vs = CompileShader(GL_VERTEX_SHADER, vis_xyz_vs_source, 0);
	unsigned int gs = CompileShader(GL_GEOMETRY_SHADER, vis_xyz_gs_source, 0);
	unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, vis_xyz_fs_source, 0);
	glAttachShader(program, vs);
	glAttachShader(program, gs);
	glAttachShader(program, fs);
	glLinkProgram(program);
	glValidateProgram(program);
	glDeleteShader(vs);
	glDeleteShader(gs);
	glDeleteShader(fs);

	pcl_resource_.program_ = program;
}


void LamurePointCloudPlugin::create_frustum_resources() {
	if (plugin->notify_button->state()) {
		std::cout << "[Notify] create_frustum_resources() " << std::endl;
		std::cout << "[Notify] wglGetCurrentContext(): " << wglGetCurrentContext() << std::endl;
	}
	std::vector<scm::math::vec3d> corner_values = scm_camera_->get_frustum_corners();
	for (size_t i = 0; i < corner_values.size(); ++i) {
		auto vv = scm::math::vec3f(corner_values[i]);
		frustum_resource_.vertices_[i * 3 + 0] = vv.x;
		frustum_resource_.vertices_[i * 3 + 1] = vv.y;
		frustum_resource_.vertices_[i * 3 + 2] = vv.z;
	}

	GLuint vao_;
	glGenVertexArrays(1, &vao_);
	glBindVertexArray(vao_);

	GLuint ibo_;
	glGenBuffers(1, &ibo_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, frustum_resource_.idx_.size() * sizeof(unsigned short), frustum_resource_.idx_.data(), GL_STATIC_DRAW);

	GLuint vbo_;
	glGenBuffers(1, &vbo_);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * frustum_resource_.vertices_.size(), frustum_resource_.vertices_.data(), GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	unsigned int program = glCreateProgram();
	unsigned int vs = CompileShader(GL_VERTEX_SHADER, vis_line_bb_vs_source, 0);
	unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, vis_line_bb_fs_source, 0);
	glAttachShader(program, vs);
	glAttachShader(program, fs);
	glLinkProgram(program);
	glValidateProgram(program);
	glDeleteShader(vs);
	glDeleteShader(fs);

	frustum_resource_.vao_ = vao_;
	frustum_resource_.vbo_ = vbo_;
	frustum_resource_.ibo_ = ibo_;
	frustum_resource_.program_ = program;

	//if (!glIsBuffer(frustum_resource_.vbo_)) { std::cerr << "VBO ist ungültig!" << std::endl; }
	//if (!glIsBuffer(frustum_resource_.ibo_)) { std::cerr << "IBO ist ungültig!" << std::endl; }
	//if (!glIsVertexArray(frustum_resource_.vao_)) { std::cerr << "VAO ist ungültig!" << std::endl; }

	//printBufferContents(GL_ELEMENT_ARRAY_BUFFER, frustum_resource_.ibo_, frustum_resource_.idx_.size() * sizeof(unsigned short));
	//printBufferContents(GL_ARRAY_BUFFER, frustum_resource_.vbo_, sizeof(float) * vertices_.size());
	//printVAOAttributes(frustum_resource_.vao_);

	glBindVertexArray(0);
}

void LamurePointCloudPlugin::create_coord_resources() {
	if (plugin->notify_button->state()) { 
		std::cout << "[Notify] create_coord_resources() " << std::endl; 
		std::cout << "[Notify] wglGetCurrentContext(): " << wglGetCurrentContext() << std::endl;
	}
	GLuint vao_;
	glGenVertexArrays(1, &vao_);
	glBindVertexArray(vao_);

	GLuint ibo_;
	glGenBuffers(1, &ibo_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, coord_resource_.idx_.size() * sizeof(unsigned short), coord_resource_.idx_.data(), GL_STATIC_DRAW);

	GLuint vbo_;
	glGenBuffers(1, &vbo_);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * coord_resource_.vertices_.size(), coord_resource_.vertices_.data(), GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

	unsigned int program = glCreateProgram();
	unsigned int vs = CompileShader(GL_VERTEX_SHADER, vis_line_bb_vs_source, 0);
	unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, vis_line_bb_fs_source, 0);
	glAttachShader(program, vs);
	glAttachShader(program, fs);
	glLinkProgram(program);
	glValidateProgram(program);
	glDeleteShader(vs);
	glDeleteShader(fs);

	coord_resource_.vao_ = vao_;
	coord_resource_.vbo_ = vbo_;
	coord_resource_.ibo_ = ibo_;
	coord_resource_.program_ = program;

	glBindVertexArray(0);
}


void LamurePointCloudPlugin::create_box_resources() {
	if (plugin->notify_button->state()) {
		std::cout << "[Notify] create_box_resources() " << std::endl;
		std::cout << "[Notify] wglGetCurrentContext(): " << wglGetCurrentContext() << std::endl;
	}

	for (uint32_t model_id = 0; model_id < num_models_; ++model_id) {
		std::vector<vector<float>> corners_;
		const auto& bvh_ = lamure::ren::model_database::get_instance()->get_model(model_id)->get_bvh();
		const auto& bounding_boxes = bvh_->get_bounding_boxes();
		for (uint64_t node_id = 0; node_id < bounding_boxes.size(); ++node_id) {
			corners_.push_back(plugin->getBoxCorners(bounding_boxes[node_id]));
		}
		bvh_res_[model_id].corners_ = corners_;
	}
	vector<float> vertices_ = plugin->getBoxCorners(lamure::ren::model_database::get_instance()->get_model(0)->get_bvh()->get_bounding_boxes()[0]);

	GLuint vao_;
	glGenVertexArrays(1, &vao_);
	glBindVertexArray(vao_);

	GLuint ibo_;
	glGenBuffers(1, &ibo_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, box_resource_.idx_.size() * sizeof(unsigned short), box_resource_.idx_.data(), GL_STATIC_DRAW);

	GLuint vbo_;
	glGenBuffers(1, &vbo_);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices_.size(), vertices_.data(), GL_STREAM_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

	unsigned int program_ = glCreateProgram();
	unsigned int vs = CompileShader(GL_VERTEX_SHADER, vis_line_bb_vs_source, 0);
	unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, vis_line_bb_fs_source, 0);
	glAttachShader(program_, vs);
	glAttachShader(program_, fs);
	glLinkProgram(program_);
	glValidateProgram(program_);
	glDeleteShader(vs);
	glDeleteShader(fs);

	box_resource_.vao_ = vao_;
	box_resource_.vbo_ = vbo_;
	box_resource_.ibo_ = ibo_;
	box_resource_.program_ = program_;

	glBindVertexArray(0);
}


void LamurePointCloudPlugin::create_aux_resources() {
	if (!settings_.create_aux_resources_) {
		return;
	}
	// init sphere rendering
	GLuint vbo_s;
	glGenBuffers(1, &vbo_s);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_s);
	glBufferData(GL_ARRAY_BUFFER, sphere_resource_.points.size() * sizeof(float), sphere_resource_.points.data(), GL_STREAM_DRAW);

	GLuint vao_s;
	glGenVertexArrays(1, &vao_s);
	glBindVertexArray(vao_s);
	glBindVertexArray(0);

	sphere_resource_.vbo_ = vbo_s;
	sphere_resource_.vao_ = vao_s;

	GLuint ibo_p;
	glGenBuffers(1, &ibo_p);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_p);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, plane_resource_.idx_.size() * sizeof(unsigned short), plane_resource_.idx_.data(), GL_STATIC_DRAW);
	plane_resource_.ibo_ = ibo_p;

	//// Erstellen von VAO und VBO
	//GLuint vao_t;
	//glGenVertexArrays(1, &vao_t);
	//glBindVertexArray(vao_t);

	//GLuint vbo_t;
	//glGenBuffers(1, &vbo_t);
	//glBindBuffer(GL_ARRAY_BUFFER, vbo_t);
	//glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), &vertices[0], GL_STATIC_DRAW);

	//glEnableVertexAttribArray(0);
	//glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);

	//glBindBuffer(GL_ARRAY_BUFFER, 0);
	//glBindVertexArray(0);

	//// Erstellen des Shader-Programms
	//unsigned int text_program = glCreateProgram();
	//unsigned int text_vs = CompileShader(GL_VERTEX_SHADER, vis_text_vs_source, 0);
	//unsigned int text_fs = CompileShader(GL_FRAGMENT_SHADER, vis_text_fs_source, 0);
	//glAttachShader(text_program, text_vs);
	//glAttachShader(text_program, text_fs);
	//glLinkProgram(text_program);
	//glValidateProgram(text_program);
	//glDeleteShader(text_vs);
	//glDeleteShader(text_fs);
	//text_resource_.program_ = text_program;

	//// Setzen des Sampler2D auf Textureinheit 0
	//glUseProgram(text_resource_.program_);
	//glUniform1i(glGetUniformLocation(text_resource_.program_, "text"), 0);
	//glUseProgram(0);

	//text_resource_.vao_ = vao_t;
	//text_resource_.vbo_ = vbo_t;
}


void LamurePointCloudPlugin::updateModelRotation() {
	scm::math::mat4d rotX = scm::math::make_rotation(rotationAngles.x, scm::math::vec3d(1.0, 0.0, 0.0));
	scm::math::mat4d rotY = scm::math::make_rotation(rotationAngles.y, scm::math::vec3d(0.0, 1.0, 0.0));
	scm::math::mat4d rotZ = scm::math::make_rotation(rotationAngles.z, scm::math::vec3d(0.0, 0.0, 1.0));
	settings_.model_rot_ = rotZ * rotY * rotX;
}


scm::math::mat4f LamurePointCloudPlugin::createSwapYZMatrix() {
	scm::math::mat4f swapMatrix = scm::math::mat4f::identity();
	swapMatrix[5] = 0.0f;
	swapMatrix[6] = 1.0f;
	swapMatrix[9] = -1.0f;
	swapMatrix[10] = 0.0f;
	return swapMatrix;
}


scm::math::mat4d LamurePointCloudPlugin::createSwapYZ() {
	scm::math::mat4d swapMatrix = scm::math::mat4d::identity();
	swapMatrix[5] = 0.0f;
	swapMatrix[6] = 1.0f;
	swapMatrix[9] = -1.0f;
	swapMatrix[10] = 0.0f;
	return swapMatrix;
}

scm::math::mat4d LamurePointCloudPlugin::swapMiddleColumns(const scm::math::mat4d& m)
{
	scm::math::mat4d swapMatrix = scm::math::mat4d::identity();
	swapMatrix[5] = 0.0f;
	swapMatrix[6] = -1.0f;
	swapMatrix[9] = 1.0f;
	swapMatrix[10] = 0.0f;

	scm::math::mat4d result(m * swapMatrix);

	return result;
}

scm::math::mat4d LamurePointCloudPlugin::swapMiddleColumns(scm::math::mat4d& m)
{
	scm::math::mat4d swapMatrix = scm::math::mat4d::identity();
	swapMatrix[5] = 0.0f;
	swapMatrix[6] = -1.0f;
	swapMatrix[9] = 1.0f;
	swapMatrix[10] = 0.0f;

	scm::math::mat4d result(m * swapMatrix);

	return result;
}

scm::math::mat4d LamurePointCloudPlugin::swapMiddleRows(scm::math::mat4d& m)
{
	scm::math::mat4d swapMatrix = scm::math::mat4d::identity();
	swapMatrix[5] = 0.0f;
	swapMatrix[6] = 1.0f;
	swapMatrix[9] = -1.0f;
	swapMatrix[10] = 0.0f;

	scm::math::mat4d result(swapMatrix * m);

	return result;
}

scm::math::mat4d LamurePointCloudPlugin::swapMiddleRows(const scm::math::mat4d& m)
{
	scm::math::mat4d swapMatrix = scm::math::mat4d::identity();
	swapMatrix[5] = 0.0f;
	swapMatrix[6] = 1.0f;
	swapMatrix[9] = -1.0f;
	swapMatrix[10] = 0.0f;

	scm::math::mat4d result(swapMatrix * m);

	return result;
}


bool LamurePointCloudPlugin::read_shader(std::string const& path_string, std::string& shader_string, bool keep_optional_shader_code = false) 
{
	if (!boost::filesystem::exists(path_string)) {
		std::cout << "WARNING: File " << path_string << "does not exist." << std::endl;
		return false;
	}
	std::ifstream shader_source(path_string, std::ios::in);
	std::string line_buffer;
	std::string include_prefix("INCLUDE");
	std::string optional_begin_prefix("OPTIONAL_BEGIN");
	std::string optional_end_prefix("OPTIONAL_END");
	std::size_t slash_position = path_string.find_last_of("/\\");
	std::string const base_path = path_string.substr(0, slash_position + 1);

	bool disregard_code = false;
	while (std::getline(shader_source, line_buffer)) {
		line_buffer = strip_whitespace(line_buffer);
		if (parse_prefix(line_buffer, include_prefix)) {
			if (!disregard_code || keep_optional_shader_code) {
				std::string filename_string = line_buffer;
				read_shader(base_path + filename_string, shader_string);
			}
		}
		else if (parse_prefix(line_buffer, optional_begin_prefix)) {
			disregard_code = true;
		}
		else if (parse_prefix(line_buffer, optional_end_prefix)) {
			disregard_code = false;
		}
		else {
			if ((!disregard_code) || keep_optional_shader_code) {
				shader_string += line_buffer + "\n";
			}
		}
	}
	return true;
}


void LamurePointCloudPlugin::init_lamure_shader()
{
	if (plugin->notify_button->state() == 1) { std::cout << "[Notify] init_lamure_shader()" << std::endl; }
	try
	{
		if (!read_shader(shader_root_path + "/vis/vis_surfel_shader.glslv", vis_surfel_shader_vs_source)
			|| !read_shader(shader_root_path + "/vis/vis_surfel_shader.glslf", vis_surfel_shader_fs_source)
			|| !read_shader(shader_root_path + "/vis/vis_line_bb.glslv", vis_line_bb_vs_source)
			|| !read_shader(shader_root_path + "/vis/vis_line_bb.glslf", vis_line_bb_fs_source)
			|| !read_shader(shader_root_path + "/vis/vis_quad.glslv", vis_quad_vs_source)
			|| !read_shader(shader_root_path + "/vis/vis_quad.glslf", vis_quad_fs_source)
			|| !read_shader(shader_root_path + "/vis/vis_line.glslv", vis_line_vs_source)
			|| !read_shader(shader_root_path + "/vis/vis_line.glslf", vis_line_fs_source)
			|| !read_shader(shader_root_path + "/vis/vis_triangle.glslv", vis_triangle_vs_source)
			|| !read_shader(shader_root_path + "/vis/vis_triangle.glslf", vis_triangle_fs_source)
			|| !read_shader(shader_root_path + "/vis/vis_plane.glslv", vis_plane_vs_source)
			|| !read_shader(shader_root_path + "/vis/vis_plane.glslf", vis_plane_fs_source)
			|| !read_shader(shader_root_path + "/vis/vis_text.glslv", vis_text_vs_source)
			|| !read_shader(shader_root_path + "/vis/vis_text.glslf", vis_text_fs_source)
			|| !read_shader(shader_root_path + "/vis/vis_box.glslv", vis_box_vs_source)
			|| !read_shader(shader_root_path + "/vis/vis_box.glslg", vis_box_gs_source)
			|| !read_shader(shader_root_path + "/vis/vis_box.glslf", vis_box_fs_source)

			|| !read_shader(shader_root_path + "/vt/virtual_texturing.glslv", vis_vt_vs_source)
			|| !read_shader(shader_root_path + "/vt/virtual_texturing_hierarchical.glslf", vis_vt_fs_source)

			|| !read_shader(shader_root_path + "/vis/vis_xyz.glslv", vis_xyz_vs_source)
			|| !read_shader(shader_root_path + "/vis/vis_xyz.glslg", vis_xyz_gs_source)
			|| !read_shader(shader_root_path + "/vis/vis_xyz.glslf", vis_xyz_fs_source)
			|| !read_shader(shader_root_path + "/vis/vis_xyz_pass1.glslv", vis_xyz_pass1_vs_source)
			|| !read_shader(shader_root_path + "/vis/vis_xyz_pass1.glslg", vis_xyz_pass1_gs_source)
			|| !read_shader(shader_root_path + "/vis/vis_xyz_pass1.glslf", vis_xyz_pass1_fs_source)
			|| !read_shader(shader_root_path + "/vis/vis_xyz_pass2.glslv", vis_xyz_pass2_vs_source)
			|| !read_shader(shader_root_path + "/vis/vis_xyz_pass2.glslg", vis_xyz_pass2_gs_source)
			|| !read_shader(shader_root_path + "/vis/vis_xyz_pass2.glslf", vis_xyz_pass2_fs_source)
			|| !read_shader(shader_root_path + "/vis/vis_xyz_pass3.glslv", vis_xyz_pass3_vs_source)
			|| !read_shader(shader_root_path + "/vis/vis_xyz_pass3.glslf", vis_xyz_pass3_fs_source)
			|| !read_shader(shader_root_path + "/vis/vis_xyz_qz.glslv", vis_xyz_qz_vs_source)
			|| !read_shader(shader_root_path + "/vis/vis_xyz_qz_pass1.glslv", vis_xyz_qz_pass1_vs_source)
			|| !read_shader(shader_root_path + "/vis/vis_xyz_qz_pass2.glslv", vis_xyz_qz_pass2_vs_source)

			|| !read_shader(shader_root_path + "/vis/vis_xyz.glslv", vis_xyz_vs_lighting_source, true)
			|| !read_shader(shader_root_path + "/vis/vis_xyz.glslg", vis_xyz_gs_lighting_source, true)
			|| !read_shader(shader_root_path + "/vis/vis_xyz.glslf", vis_xyz_fs_lighting_source, true)
			|| !read_shader(shader_root_path + "/vis/vis_xyz_pass2.glslv", vis_xyz_pass2_vs_lighting_source, true)
			|| !read_shader(shader_root_path + "/vis/vis_xyz_pass2.glslg", vis_xyz_pass2_gs_lighting_source, true)
			|| !read_shader(shader_root_path + "/vis/vis_xyz_pass2.glslf", vis_xyz_pass2_fs_lighting_source, true)
			|| !read_shader(shader_root_path + "/vis/vis_xyz_pass3.glslv", vis_xyz_pass3_vs_lighting_source, true)
			|| !read_shader(shader_root_path + "/vis/vis_xyz_pass3.glslf", vis_xyz_pass3_fs_lighting_source, true)
			) {
			std::cout << "error reading shader files" << std::endl;
			exit(1);
		}

		vis_surfel_shader_ = device_->create_program(
			boost::assign::list_of
			(device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_surfel_shader_vs_source))
			(device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_surfel_shader_fs_source)));
		if (!vis_surfel_shader_) {
			std::cout << "error creating shader vis_surfel_shader_ program" << std::endl;
			exit(1);
		}
		vis_quad_shader_ = device_->create_program(
			boost::assign::list_of
			(device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_quad_vs_source))
			(device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_quad_fs_source)));
		if (!vis_quad_shader_) {
			std::cout << "error creating shader vis_quad_shader_ program" << std::endl;
			exit(1);
		}

		vis_plane_shader_ = device_->create_program(
			boost::assign::list_of
			(device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_plane_vs_source))
			(device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_plane_fs_source)));
		if (!vis_quad_shader_) {
			std::cout << "error creating shader vis_plane_shader_ program" << std::endl;
			exit(1);
		}
		vis_line_bb_shader_ = device_->create_program(
			boost::assign::list_of
			(device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_line_bb_vs_source))
			(device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_line_bb_fs_source)));

		if (!vis_line_bb_shader_) {
			std::cout << "error creating shader vis_line_bb_shader_ program" << std::endl;
			exit(1);
		}
		vis_text_shader_ = device_->create_program(
			boost::assign::list_of
			(device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_text_vs_source))
			(device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_text_fs_source)));

		if (!vis_text_shader_) {
			std::cout << "error creating shader vis_text_shader_ program" << std::endl;
			exit(1);
		}
		vis_line_shader_ = device_->create_program(
			boost::assign::list_of
			(device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_line_vs_source))
			(device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_line_fs_source)));

		if (!vis_line_shader_) {
			std::cout << "error creating shader vis_line_shader_ program" << std::endl;
			exit(1);
		}
		vis_triangle_shader_ = device_->create_program(
			boost::assign::list_of
			(device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_triangle_vs_source))
			(device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_triangle_fs_source)));

		if (!vis_triangle_shader_) {
			std::cout << "error creating shader vis_triangle_shader_ program" << std::endl;
			std::exit(1);
		}
		vis_vt_shader_ = device_->create_program(
			boost::assign::list_of
			(device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_vt_vs_source))
			(device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_vt_fs_source)));

		if (!vis_vt_shader_) {
			std::cout << "error creating shader vis_vt_shader_program" << std::endl;
			std::exit(1);
		}
		vis_xyz_shader_ = device_->create_program(
			boost::assign::list_of
			(device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_xyz_vs_source))
			(device_->create_shader(scm::gl::STAGE_GEOMETRY_SHADER, vis_xyz_gs_source))
			(device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_xyz_fs_source)));

		if (!vis_xyz_shader_) {
			//scm::err() << scm::log::error << scm::log::end;
			std::cout << "error creating shader vis_xyz_shader_ program" << std::endl;
			exit(1);
		}
		vis_xyz_pass1_shader_ = device_->create_program(
			boost::assign::list_of
			(device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_xyz_pass1_vs_source))
			(device_->create_shader(scm::gl::STAGE_GEOMETRY_SHADER, vis_xyz_pass1_gs_source))
			(device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_xyz_pass1_fs_source)));

		if (!vis_xyz_pass1_shader_) {
			std::cout << "error creating vis_xyz_pass1_shader_ program" << std::endl;
			exit(1);
		}
		vis_xyz_pass2_shader_ = device_->create_program(
			boost::assign::list_of
			(device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_xyz_pass2_vs_source))
			(device_->create_shader(scm::gl::STAGE_GEOMETRY_SHADER, vis_xyz_pass2_gs_source))
			(device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_xyz_pass2_fs_source)));

		if (!vis_xyz_pass2_shader_) {
			std::cout << "error creating vis_xyz_pass2_shader_ program" << std::endl;
			exit(1);
		}
		vis_xyz_pass3_shader_ = device_->create_program(
			boost::assign::list_of
			(device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_xyz_pass3_vs_source))
			(device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_xyz_pass3_fs_source)));

		if (!vis_xyz_pass3_shader_) {
			std::cout << "error creating vis_xyz_pass3_shader_ program" << std::endl;
			exit(1);
		}
		vis_xyz_lighting_shader_ = device_->create_program(
			boost::assign::list_of
			(device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_xyz_vs_lighting_source))
			(device_->create_shader(scm::gl::STAGE_GEOMETRY_SHADER, vis_xyz_gs_lighting_source))
			(device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_xyz_fs_lighting_source)));

		if (!vis_xyz_lighting_shader_) {
			std::cout << "error creating vis_xyz_lighting_shader_ program" << std::endl;
			exit(1);
		}
		vis_xyz_pass2_lighting_shader_ = device_->create_program(
			boost::assign::list_of
			(device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_xyz_pass2_vs_lighting_source))
			(device_->create_shader(scm::gl::STAGE_GEOMETRY_SHADER, vis_xyz_pass2_gs_lighting_source))
			(device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_xyz_pass2_fs_lighting_source)));

		if (!vis_xyz_pass2_lighting_shader_) {
			std::cout << "error creating vis_xyz_pass2_lighting_shader_ program" << std::endl;
			exit(1);
		}
		vis_xyz_pass3_lighting_shader_ = device_->create_program(
			boost::assign::list_of
			(device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_xyz_pass3_vs_lighting_source))
			(device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_xyz_pass3_fs_lighting_source)));

		if (!vis_xyz_pass3_lighting_shader_) {
			std::cout << "error creating vis_xyz_pass3_lighting_shader_ program" << std::endl;
			exit(1);
		}
	}
	catch (std::exception& e)
	{
		std::cout << e.what() << std::endl;
	}
}


void LamurePointCloudPlugin::create_framebuffers()
{
	if (plugin->notify_button->state()) { std::cout << "[Notify] create_framebuffers() " << std::endl; }
	fbo_.reset();
	fbo_color_buffer_.reset();
	fbo_depth_buffer_.reset();
	pass1_fbo_.reset();
	pass1_depth_buffer_.reset();
	pass2_fbo_.reset();
	pass2_color_buffer_.reset();
	pass2_normal_buffer_.reset();
	pass2_view_space_pos_buffer_.reset();

	fbo_ = device_->create_frame_buffer();
	fbo_color_buffer_ = device_->create_texture_2d(scm::math::vec2ui(traits->width, traits->height), scm::gl::FORMAT_RGBA_32F, 1, 1, 1);
	fbo_depth_buffer_ = device_->create_texture_2d(scm::math::vec2ui(traits->width, traits->height), scm::gl::FORMAT_D24, 1, 1, 1);
	fbo_->attach_color_buffer(0, fbo_color_buffer_);
	fbo_->attach_depth_stencil_buffer(fbo_depth_buffer_);

	pass1_fbo_ = device_->create_frame_buffer();
	pass1_depth_buffer_ = device_->create_texture_2d(scm::math::vec2ui(traits->width, render_height_), scm::gl::FORMAT_D24, 1, 1, 1);
	pass1_fbo_->attach_depth_stencil_buffer(pass1_depth_buffer_);

	//pass2_fbo_ = device_->create_frame_buffer();
	//pass2_color_buffer_ = device_->create_texture_2d(scm::math::vec2ui(traits->width, render_height_), scm::gl::FORMAT_RGBA_32F, 1, 1, 1);
	//pass2_fbo_->attach_color_buffer(0, pass2_color_buffer_);
	//pass2_fbo_->attach_depth_stencil_buffer(pass1_depth_buffer_);

	//pass2_normal_buffer_ = device_->create_texture_2d(scm::math::vec2ui(traits->width, render_height_), scm::gl::FORMAT_RGB_32F, 1, 1, 1);
	//pass2_fbo_->attach_color_buffer(1, pass2_normal_buffer_);
	//pass2_view_space_pos_buffer_ = device_->create_texture_2d(scm::math::vec2ui(traits->width, render_height_), scm::gl::FORMAT_RGB_32F, 1, 1, 1);
	//pass2_fbo_->attach_color_buffer(2, pass2_view_space_pos_buffer_);
}


void LamurePointCloudPlugin::init_render_states() 
{
	if (plugin->notify_button->state()) { std::cout << "[Notify] init_render_states() " << std::endl; }
	color_blending_state_ = device_->create_blend_state(true, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE,
		scm::gl::FUNC_ONE, scm::gl::EQ_FUNC_ADD, scm::gl::EQ_FUNC_ADD);
	color_no_blending_state_ = device_->create_blend_state(false);

	depth_state_less_ = device_->create_depth_stencil_state(true, true, scm::gl::COMPARISON_LESS);
	auto no_depth_test_descriptor = depth_state_less_->descriptor();
	no_depth_test_descriptor._depth_test = false;
	depth_state_disable_ = device_->create_depth_stencil_state(no_depth_test_descriptor);
	depth_state_without_writing_ = device_->create_depth_stencil_state(true, false, scm::gl::COMPARISON_LESS_EQUAL);
	no_backface_culling_rasterizer_state_ = device_->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false, false, 0.0, false, false);
	filter_linear_ = device_->create_sampler_state(scm::gl::FILTER_ANISOTROPIC, scm::gl::WRAP_CLAMP_TO_EDGE, 16u);
	filter_nearest_ = device_->create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);
	//vt_filter_linear_ = device_->create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);
	//vt_filter_nearest_ = device_->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);
}


void LamurePointCloudPlugin::add_stateset_uniforms(osg::ref_ptr<osg::StateSet> stateset) 
{
	stateset->addUniform(new osg::Uniform(osg::Uniform::FLOAT_MAT4, "model_matrix"));
	stateset->addUniform(new osg::Uniform(osg::Uniform::FLOAT_MAT4, "model_view_matrix"));
	stateset->addUniform(new osg::Uniform(osg::Uniform::FLOAT_MAT4, "inv_mv_matrix"));
	stateset->addUniform(new osg::Uniform(osg::Uniform::FLOAT_MAT4, "model_to_screen_matrix"));
	stateset->addUniform(new osg::Uniform(osg::Uniform::FLOAT_MAT4, "mvp_matrix"));

	stateset->addUniform(new osg::Uniform("win_size", osg::Vec2f(traits->width, traits->height)));
	stateset->addUniform(new osg::Uniform("near_plane", settings_.near_plane_));
	stateset->addUniform(new osg::Uniform("far_plane", settings_.far_plane_));
	stateset->addUniform(new osg::Uniform("point_size_factor", settings_.lod_point_scale_));
	stateset->addUniform(new osg::Uniform("show_normals", (bool)settings_.show_normals_));
	stateset->addUniform(new osg::Uniform("show_accuracy", (bool)settings_.show_accuracy_));
	stateset->addUniform(new osg::Uniform("show_radius_deviation", (bool)settings_.show_radius_deviation_));
	stateset->addUniform(new osg::Uniform("show_output_sensitivity", (bool)settings_.show_output_sensitivity_));
	stateset->addUniform(new osg::Uniform("channel", settings_.channel_));
	stateset->addUniform(new osg::Uniform("heatmap", (bool)settings_.heatmap_));
	stateset->addUniform(new osg::Uniform("face_eye", false));
	stateset->addUniform(new osg::Uniform("eye", osg::Vec3f(0.0f, 0.0f, 0.0f)));
	stateset->addUniform(new osg::Uniform("model_radius_scale", settings_.scale_radius_));
	stateset->addUniform(new osg::Uniform("max_radius", settings_.max_radius_));
	stateset->addUniform(new osg::Uniform("heatmap_min", settings_.heatmap_min_));
	stateset->addUniform(new osg::Uniform("heatmap_max", settings_.heatmap_max_));
	stateset->addUniform(new osg::Uniform("heatmap_min_color", osg::Vec3f(settings_.heatmap_color_min_[0], settings_.heatmap_color_min_[1], settings_.heatmap_color_min_[2])));
	stateset->addUniform(new osg::Uniform("heatmap_max_color", osg::Vec3f(settings_.heatmap_color_max_[0], settings_.heatmap_color_max_[1], settings_.heatmap_color_max_[2])));
	if (settings_.enable_lighting_) {
		stateset->addUniform(new osg::Uniform("use_material_color", (bool)settings_.use_material_color_));
		stateset->addUniform(new osg::Uniform("material_diffuse", osg::Vec3f(settings_.material_diffuse_[0], settings_.material_diffuse_[1], settings_.material_diffuse_[2])));
		stateset->addUniform(new osg::Uniform("material_specular", osg::Vec3f(settings_.material_specular_[0], settings_.material_specular_[1], settings_.material_specular_[2])));
		stateset->addUniform(new osg::Uniform("ambient_light_color", osg::Vec3f(settings_.ambient_light_color_[0], settings_.ambient_light_color_[1], settings_.ambient_light_color_[2])));
		stateset->addUniform(new osg::Uniform("point_light_color", osg::Vec3f(settings_.point_light_color_[0], settings_.point_light_color_[1], settings_.point_light_color_[2])));
	}
}


void LamurePointCloudPlugin::set_stateset_uniforms(osg::ref_ptr<osg::StateSet> stateset)
{
	if (auto u = stateset->getUniform("win_size"))					u->set(osg::Vec2f(traits->width, traits->height));
	if (auto u = stateset->getUniform("near_plane"))				u->set(settings_.near_plane_);
	if (auto u = stateset->getUniform("far_plane"))					u->set(settings_.far_plane_);
	if (auto u = stateset->getUniform("point_size_factor"))			u->set(settings_.lod_point_scale_);
	if (auto u = stateset->getUniform("show_normals"))				u->set((bool)settings_.show_normals_);
	if (auto u = stateset->getUniform("show_accuracy"))				u->set((bool)settings_.show_accuracy_);
	if (auto u = stateset->getUniform("show_radius_deviation"))		u->set((bool)settings_.show_radius_deviation_);
	if (auto u = stateset->getUniform("show_output_sensitivity"))	u->set((bool)settings_.show_output_sensitivity_);
	if (auto u = stateset->getUniform("channel"))					u->set(settings_.channel_);
	if (auto u = stateset->getUniform("heatmap"))					u->set((bool)settings_.heatmap_);
	if (auto u = stateset->getUniform("face_eye"))					u->set(false);
	if (auto u = stateset->getUniform("model_radius_scale"))		u->set(settings_.scale_radius_);
	if (auto u = stateset->getUniform("max_radius"))				u->set(settings_.max_radius_);
	if (auto u = stateset->getUniform("heatmap_min"))				u->set(settings_.heatmap_min_);
	if (auto u = stateset->getUniform("heatmap_max"))				u->set(settings_.heatmap_max_);
	if (auto u = stateset->getUniform("heatmap_min_color"))			u->set(osg::Vec3f(settings_.heatmap_color_min_[0],
		settings_.heatmap_color_min_[1],
		settings_.heatmap_color_min_[2]));
	if (auto u = stateset->getUniform("heatmap_max_color"))			u->set(osg::Vec3f(settings_.heatmap_color_max_[0],
		settings_.heatmap_color_max_[1],
		settings_.heatmap_color_max_[2]));

	if (settings_.enable_lighting_) {
		if (auto u = stateset->getUniform("use_material_color"))	u->set((bool)settings_.use_material_color_);
		if (auto u = stateset->getUniform("material_diffuse"))		u->set(osg::Vec3f(settings_.material_diffuse_[0],
			settings_.material_diffuse_[1],
			settings_.material_diffuse_[2]));
		if (auto u = stateset->getUniform("material_specular"))		u->set(osg::Vec3f(settings_.material_specular_[0],
			settings_.material_specular_[1],
			settings_.material_specular_[2]));
		if (auto u = stateset->getUniform("ambient_light_color"))	u->set(osg::Vec3f(settings_.ambient_light_color_[0],
			settings_.ambient_light_color_[1],
			settings_.ambient_light_color_[2]));
		if (auto u = stateset->getUniform("point_light_color"))		u->set(osg::Vec3f(settings_.point_light_color_[0],
			settings_.point_light_color_[1],
			settings_.point_light_color_[2]));
	}
}


void LamurePointCloudPlugin::set_lamure_uniforms(scm::gl::program_ptr shader) {
	shader->uniform("win_size", scm::math::vec2f(traits->width, traits->height));
	shader->uniform("near_plane", settings_.near_plane_);
	shader->uniform("far_plane", settings_.far_plane_);
	shader->uniform("point_size_factor", settings_.lod_point_scale_);
	shader->uniform("show_normals", (bool)settings_.show_normals_);
	shader->uniform("show_accuracy", (bool)settings_.show_accuracy_);
	shader->uniform("show_radius_deviation", (bool)settings_.show_radius_deviation_);
	shader->uniform("show_output_sensitivity", (bool)settings_.show_output_sensitivity_);
	shader->uniform("channel", settings_.channel_);
	shader->uniform("heatmap", (bool)settings_.heatmap_);
	shader->uniform("face_eye", false);
	shader->uniform("model_radius_scale", settings_.scale_radius_);
	shader->uniform("max_radius", settings_.max_radius_);
	shader->uniform("heatmap_min", settings_.heatmap_min_);
	shader->uniform("heatmap_max", settings_.heatmap_max_);
	shader->uniform("heatmap_min_color", settings_.heatmap_color_min_);
	shader->uniform("heatmap_max_color", settings_.heatmap_color_max_);
	if (settings_.enable_lighting_) {
		shader->uniform("use_material_color", settings_.use_material_color_);
		shader->uniform("material_diffuse", settings_.material_diffuse_);
		shader->uniform("material_specular", settings_.material_specular_);
		shader->uniform("ambient_light_color", settings_.ambient_light_color_);
		shader->uniform("point_light_color", settings_.point_light_color_);
	}
}


void LamurePointCloudPlugin::set_gl_uniforms(GLuint program) {
	glUniform1f(glGetUniformLocation(program, "model_radius_scale"), settings_.scale_radius_);
	glUniform1f(glGetUniformLocation(program, "max_radius"), settings_.max_radius_);

	glUniform2f(glGetUniformLocation(program, "win_size"), settings_.width_, settings_.height_);
	glUniform1f(glGetUniformLocation(program, "near_plane"), static_cast<float>(settings_.near_plane_));
	glUniform1f(glGetUniformLocation(program, "far_plane"), static_cast<float>(settings_.far_plane_));
	glUniform1f(glGetUniformLocation(program, "point_size_factor"), settings_.lod_point_scale_);

	glUniform1i(glGetUniformLocation(program, "show_normals"), settings_.show_normals_);
	glUniform1i(glGetUniformLocation(program, "show_accuracy"), static_cast<int>(settings_.show_accuracy_));
	glUniform1i(glGetUniformLocation(program, "show_output_sensitivity"), static_cast<int>(settings_.show_output_sensitivity_));
	glUniform1i(glGetUniformLocation(program, "channel"), settings_.channel_);
	glUniform1i(glGetUniformLocation(program, "heatmap"), static_cast<int>(settings_.heatmap_));

	glUniform1f(glGetUniformLocation(program, "heatmap_min"), settings_.heatmap_min_);
	glUniform1f(glGetUniformLocation(program, "heatmap_max"), settings_.heatmap_max_);

	glUniform3f(glGetUniformLocation(program, "heatmap_min_color"),
		settings_.heatmap_color_min_[0],
		settings_.heatmap_color_min_[1],
		settings_.heatmap_color_min_[2]);

	glUniform3f(glGetUniformLocation(program, "heatmap_max_color"),
		settings_.heatmap_color_max_[0],
		settings_.heatmap_color_max_[1],
		settings_.heatmap_color_max_[2]);

	if (settings_.enable_lighting_) {
		glUniform1i(glGetUniformLocation(program, "use_material_color"), static_cast<int>(settings_.use_material_color_));
		glUniform3f(glGetUniformLocation(program, "material_diffuse"),
			settings_.material_diffuse_[0],
			settings_.material_diffuse_[1],
			settings_.material_diffuse_[2]);

		glUniform4f(glGetUniformLocation(program, "material_specular"),
			settings_.material_specular_[0],
			settings_.material_specular_[1],
			settings_.material_specular_[2],
			settings_.material_specular_[3]);

		glUniform3f(glGetUniformLocation(program, "ambient_light_color"),
			settings_.ambient_light_color_[0],
			settings_.ambient_light_color_[1],
			settings_.ambient_light_color_[2]);

		glUniform4f(glGetUniformLocation(program, "point_light_color"),
			settings_.point_light_color_[0],
			settings_.point_light_color_[1],
			settings_.point_light_color_[2],
			settings_.point_light_color_[3]);
	}
}


std::string const LamurePointCloudPlugin::strip_whitespace(std::string const& in_string) {
	return boost::regex_replace(in_string, boost::regex("^ +| +$|( ) +"), "$1");
}


scm::gl::data_format get_tex_format() {
	switch (vt::VTConfig::get_instance().get_format_texture()) {
	case vt::VTConfig::R8:
		return scm::gl::FORMAT_R_8;
	case vt::VTConfig::RGB8:
		return scm::gl::FORMAT_RGB_8;
	case vt::VTConfig::RGBA8:
	default:
		return scm::gl::FORMAT_RGBA_8;
	}
}


string LamurePointCloudPlugin::getConfigEntry(string scope) {
	std::cout << "getConfigEntry(scope): ";
	covise::coCoviseConfig::ScopeEntries entries = covise::coCoviseConfig::getScopeEntries(scope);
	for (const auto& entry : entries)
	{
		return entry.second;
	}
	return "";
}


string LamurePointCloudPlugin::getConfigEntry(string scope, string name) {
	std::cout << "getConfigEntry(scope, name): ";
	covise::coCoviseConfig::ScopeEntries entries = covise::coCoviseConfig::getScopeEntries(scope);
	for (const auto& entry : entries) {
		std::cout << entry.first << " " << entry.second << " ";
		if (name == entry.first)
		{
			return entry.second;
		}
	}
	return "";
}


const char* LamurePointCloudPlugin::stringToConstChar(string str) {
	const char* cstr = str.c_str();
	return cstr;
}


LamurePointCloudPlugin* LamurePointCloudPlugin::instance()
{
	return plugin;
}


size_t LamurePointCloudPlugin::query_video_memory_in_mb() {
	int size_in_kb;
	glGetIntegerv(GL_GPU_MEMORY_INFO_CURRENT_AVAILABLE_VIDMEM_NVX, &size_in_kb);
	return size_t(size_in_kb) / 1024;
}


static bool GLLogError()
{
	while (GLenum error = glGetError())
	{
		std::cout << "[OpenGL Error] (" << error << ")" << std::endl;
	}
	return true;
}


void LamurePointCloudPlugin::load_settings(std::string const& filename) {
	std::cout << "load_settings()" << std::endl;
	std::cout << filename << std::endl;
	std::ifstream lmr_file(filename.c_str());
	if (!lmr_file.is_open()) {
		std::cout << "could not open lmr file" << std::endl;
		exit(-1);
	}
	else {
		lamure::model_t model_id = 0;
		std::string line;
		while (std::getline(lmr_file, line)) {
			if (line.length() >= 2) {
				if (line[0] == '#') {
					continue;
				}
				auto colon = line.find_first_of(':');
				if (colon == std::string::npos) {
					scm::math::mat4d transform = scm::math::mat4d::identity();
					std::string model;

					std::istringstream line_ss(line);
					line_ss >> model;

					settings_.models_.push_back(model);
					settings_.transforms_[model_id] = scm::math::mat4d::identity();
					settings_.aux_[model_id] = "";
					++model_id;
				}
				else {

					std::string key = line.substr(0, colon);

					if (key[0] == '@') {
						auto ws = line.find_first_of(' ');
						uint32_t address = atoi(strip_whitespace(line.substr(1, ws - 1)).c_str());
						key = strip_whitespace(line.substr(ws + 1, colon - (ws + 1)));
						std::string value = strip_whitespace(line.substr(colon + 1));

						if (key == "tf") {
							settings_.transforms_[address] = load_matrix(value);
							std::cout << "found transform for model id " << address << std::endl;
						}
						else if (key == "aux") {
							settings_.aux_[address] = value;
							std::cout << "found aux data for model id " << address << std::endl;
						}
						else {
							std::cout << "unrecognized key: " << key << std::endl;
							exit(-1);
						}
						continue;
					}

					key = strip_whitespace(key);
					std::string value = strip_whitespace(line.substr(colon + 1));

					if (key == "width") {
						settings_.width_ = std::max(atoi(value.c_str()), 64);
					}
					else if (key == "height") {
						settings_.height_ = std::max(atoi(value.c_str()), 64);
					}
					else if (key == "frame_div") {
						settings_.frame_div_ = std::max(atoi(value.c_str()), 1);
					}
					else if (key == "vram") {
						settings_.vram_ = std::max(atoi(value.c_str()), 8);
					}
					else if (key == "ram") {
						settings_.ram_ = std::max(atoi(value.c_str()), 8);
					}
					else if (key == "upload") {
						settings_.upload_ = std::max(atoi(value.c_str()), 8);
					}
					else if (key == "near") {
						settings_.near_plane_ = std::max(atof(value.c_str()), 0.0);
					}
					else if (key == "far") {
						settings_.far_plane_ = std::max(atof(value.c_str()), 0.1);
					}
					else if (key == "fov") {
						settings_.fov_ = std::max(atof(value.c_str()), 9.0);
					}
					else if (key == "splatting") {
						settings_.splatting_ = (bool)std::max(atoi(value.c_str()), 0);
					}
					else if (key == "gamma_correction") {
						settings_.gamma_correction_ = (bool)std::max(atoi(value.c_str()), 0);
					}
					else if (key == "gui") {
						settings_.gui_ = std::max(atoi(value.c_str()), 0);
					}
					else if (key == "speed") {
						settings_.travel_speed_ = std::min(std::max(atof(value.c_str()), 0.0), 400.0);
					}
					else if (key == "pvs_culling") {
						settings_.pvs_culling_ = (bool)std::max(atoi(value.c_str()), 0);
					}
					else if (key == "use_pvs") {
						settings_.use_pvs_ = (bool)std::max(atoi(value.c_str()), 0);
					}
					else if (key == "lod_point_scale") {
						settings_.lod_point_scale_ = std::min(std::max(atof(value.c_str()), 0.0), 10.0);
					}
					else if (key == "aux_point_size") {
						settings_.aux_point_size_ = std::min(std::max(atof(value.c_str()), 0.00001), 1.0);
					}
					else if (key == "aux_point_distance") {
						settings_.aux_point_distance_ = std::min(std::max(atof(value.c_str()), 0.00001), 1.0);
					}
					else if (key == "aux_focal_length") {
						settings_.aux_focal_length_ = std::min(std::max(atof(value.c_str()), 0.001), 10.0);
					}
					else if (key == "max_brush_size") {
						settings_.max_brush_size_ = std::min(std::max(atoi(value.c_str()), 64), 1024 * 1024);
					}
					else if (key == "lod_error") {
						settings_.lod_error_ = std::min(std::max(atof(value.c_str()), 0.0), 10.0);
					}
					else if (key == "provenance") {
						settings_.provenance_ = (bool)std::max(atoi(value.c_str()), 0);
					}
					else if (key == "create_aux_resources") {
						settings_.create_aux_resources_ = (bool)std::max(atoi(value.c_str()), 0);
					}
					else if (key == "show_normals") {
						settings_.show_normals_ = std::max(atoi(value.c_str()), 0);
					}
					else if (key == "show_accuracy") {
						settings_.show_accuracy_ = (bool)std::max(atoi(value.c_str()), 0);
					}
					else if (key == "show_radius_deviation") {
						settings_.show_radius_deviation_ = (bool)std::max(atoi(value.c_str()), 0);
					}
					else if (key == "show_output_sensitivity") {
						settings_.show_output_sensitivity_ = (bool)std::max(atoi(value.c_str()), 0);
					}
					else if (key == "show_sparse") {
						settings_.show_sparse_ = (bool)std::max(atoi(value.c_str()), 0);
					}
					else if (key == "show_views") {
						settings_.show_views_ = (bool)std::max(atoi(value.c_str()), 0);
					}
					else if (key == "show_photos") {
						settings_.show_photos_ = (bool)std::max(atoi(value.c_str()), 0);
					}
					else if (key == "show_octrees") {
						settings_.show_octrees_ = (bool)std::max(atoi(value.c_str()), 0);
					}
					else if (key == "show_bvhs") {
						settings_.show_bvhs_ = (bool)std::max(atoi(value.c_str()), 0);
					}
					else if (key == "show_pvs") {
						settings_.show_pvs_ = (bool)std::max(atoi(value.c_str()), 0);
					}
					else if (key == "channel") {
						settings_.channel_ = std::max(atoi(value.c_str()), 0);
					}
					else if (key == "enable_lighting") {
						settings_.enable_lighting_ = (bool)std::min(std::max(atoi(value.c_str()), 0), 1);
					}
					else if (key == "use_material_color") {
						settings_.use_material_color_ = (bool)std::min(std::max(atoi(value.c_str()), 0), 1);
					}
					else if (key == "material_diffuse_r") {
						settings_.material_diffuse_.x = std::max(atof(value.c_str()), 0.0);
					}
					else if (key == "material_diffuse_g") {
						settings_.material_diffuse_.y = std::max(atof(value.c_str()), 0.0);
					}
					else if (key == "material_diffuse_b") {
						settings_.material_diffuse_.z = std::max(atof(value.c_str()), 0.0);
					}
					else if (key == "material_specular_r") {
						settings_.material_specular_.x = std::max(atof(value.c_str()), 0.0);
					}
					else if (key == "material_specular_g") {
						settings_.material_specular_.y = std::max(atof(value.c_str()), 0.0);
					}
					else if (key == "material_specular_b") {
						settings_.material_specular_.z = std::max(atof(value.c_str()), 0.0);
					}
					else if (key == "material_specular_exponent") {
						settings_.material_specular_.w = std::min(std::max(atof(value.c_str()), 0.0), 10000.0);
					}
					else if (key == "ambient_light_color_r") {
						settings_.ambient_light_color_.r = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
					}
					else if (key == "ambient_light_color_g") {
						settings_.ambient_light_color_.g = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
					}
					else if (key == "ambient_light_color_b") {
						settings_.ambient_light_color_.b = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
					}
					else if (key == "point_light_color_r") {
						settings_.point_light_color_.r = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
					}
					else if (key == "point_light_color_g") {
						settings_.point_light_color_.g = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
					}
					else if (key == "point_light_color_b") {
						settings_.point_light_color_.b = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
					}
					else if (key == "point_light_intensity") {
						settings_.point_light_color_.w = std::min(std::max(atof(value.c_str()), 0.0), 10000.0);
					}
					else if (key == "background_color_r") {
						//settings_.background_color_.x = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
						settings_.background_color_.x = 0.1f;
					}
					else if (key == "background_color_g") {
						//settings_.background_color_.y = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
						settings_.background_color_.y = 0.1f;
					}
					else if (key == "background_color_b") {
						//settings_.background_color_.z = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
						settings_.background_color_.z = 0.1f;
					}
					else if (key == "heatmap") {
						settings_.heatmap_ = (bool)std::max(atoi(value.c_str()), 0);
					}
					else if (key == "heatmap_min") {
						settings_.heatmap_min_ = std::max(atof(value.c_str()), 0.0);
					}
					else if (key == "heatmap_max") {
						settings_.heatmap_max_ = std::max(atof(value.c_str()), 0.0);
					}
					else if (key == "heatmap_min_r") {
						settings_.heatmap_color_min_.x = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
					}
					else if (key == "heatmap_min_g") {
						settings_.heatmap_color_min_.y = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
					}
					else if (key == "heatmap_min_b") {
						settings_.heatmap_color_min_.z = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
					}
					else if (key == "heatmap_max_r") {
						settings_.heatmap_color_max_.x = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
					}
					else if (key == "heatmap_max_g") {
						settings_.heatmap_color_max_.y = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
					}
					else if (key == "heatmap_max_b") {
						settings_.heatmap_color_max_.z = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
					}
					else if (key == "atlas_file") {
						settings_.atlas_file_ = value;
					}
					else if (key == "json") {
						settings_.json_ = value;
					}
					else if (key == "pvs") {
						settings_.pvs_ = value;
					}
					else if (key == "selection") {
						settings_.selection_ = value;
					}
					else if (key == "background_image") {
						settings_.background_image_ = value;
					}
					else if (key == "use_view_tf") {
						settings_.use_view_tf_ = std::max(atoi(value.c_str()), 0);
					}
					else if (key == "view_tf") {
						settings_.view_tf_ = load_matrix(value);
					}
					else if (key == "max_radius") {
						settings_.max_radius_ = std::max(atof(value.c_str()), 0.1);
					}
					else if (key == "scale_radius") {
						settings_.scale_radius_ = std::max(atof(value.c_str()), 0.1);
					}
					else {
						std::cout << "unrecognized key: " << key << std::endl;
						exit(-1);
					}
					std::cout << key << " : " << value << std::endl;
				}
			}
		}
		lmr_file.close();
	}

	if (!settings_.models_.empty()) {
		std::string model_path = settings_.models_[0];
		std::string directory;
		size_t last_slash_idx = model_path.find_last_of("/\\");
		if (std::string::npos != last_slash_idx) {
			directory = model_path.substr(0, last_slash_idx + 1);
		}
		else {
			directory = "";
		}
		std::string base_filename = model_path.substr(last_slash_idx + 1);
		size_t dot_idx = base_filename.rfind('.');
		if (std::string::npos != dot_idx) {
			base_filename = base_filename.substr(0, dot_idx);
		}
		std::string json_filename = directory + base_filename + ".json";
		std::ifstream json_file_check(json_filename);
		if (json_file_check.good()) {
			json_file_check.close();
			settings_.json_ = json_filename;
			std::cout << "Found JSON file: " << settings_.json_ << std::endl;
		}
		else {
			json_file_check.close();
			settings_.json_ = "";
			std::cout << "No JSON file found for the first model." << std::endl;
		}
	}
	else {
		settings_.json_ = "";
		std::cout << "No models found in settings_.models_." << std::endl;
	}
	if (settings_.provenance_ != 0) {
		if (settings_.json_.size() > 0) {
			if (settings_.json_.substr(settings_.json_.size() - 5) != ".json") {
				std::cout << "unsupported json file" << std::endl;
				exit(-1);
			}
		}
	}
	if (settings_.models_.empty()) {
		std::cout << "error: no model filename specified" << std::endl;
		exit(-1);
	}
	if (settings_.pvs_.size() > 0) {
		if (settings_.pvs_.substr(settings_.pvs_.size() - 4) != ".pvs") {
			std::cout << "unsupported pvs file" << std::endl;
			exit(-1);
		}
	}
}


scm::math::mat4d LamurePointCloudPlugin::load_matrix(const std::string& filename) {
	std::ifstream file(filename.c_str());
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


bool LamurePointCloudPlugin::parse_prefix(std::string& in_string, std::string const& prefix) {
	uint32_t num_prefix_characters = prefix.size();
	bool prefix_found
		= (!(in_string.size() < num_prefix_characters)
			&& strncmp(in_string.c_str(), prefix.c_str(), num_prefix_characters) == 0);
	if (prefix_found) {
		in_string = in_string.substr(num_prefix_characters);
		in_string = strip_whitespace(in_string);
	}
	return prefix_found;
}


void LamurePointCloudPlugin::readMenuConfigData(const char* menu, vector<ImageFileEntry>& menulist, ui::Group* subMenu)
{
	covise::coCoviseConfig::ScopeEntries entries = covise::coCoviseConfig::getScopeEntries(menu);
	for (const auto& entry : entries)
	{
		ui::Button* temp = new ui::Button(subMenu, entry.second);
		temp->setCallback([this, entry](bool state)
			{
				if (state)
				std::printf("createGeodes(planetTrans, entry.second)");
			});
		menulist.push_back(ImageFileEntry(entry.first.c_str(), entry.second.c_str(), (ui::Element*)temp));
	}
}


void LamurePointCloudPlugin::selectedMenuButton(ui::Element* menuItem)
{
	std::string filename;
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


void LamurePointCloudPlugin::lines_from_min_max(const scm::math::vec3f& min_vertex, const scm::math::vec3f& max_vertex, std::vector<scm::math::vec3f>& lines) {

	lines.push_back(scm::math::vec3f(min_vertex.x, min_vertex.y, min_vertex.z));
	lines.push_back(scm::math::vec3f(max_vertex.x, min_vertex.y, min_vertex.z));

	lines.push_back(scm::math::vec3f(max_vertex.x, min_vertex.y, min_vertex.z));
	lines.push_back(scm::math::vec3f(max_vertex.x, min_vertex.y, max_vertex.z));

	lines.push_back(scm::math::vec3f(max_vertex.x, min_vertex.y, max_vertex.z));
	lines.push_back(scm::math::vec3f(min_vertex.x, min_vertex.y, max_vertex.z));

	lines.push_back(scm::math::vec3f(min_vertex.x, min_vertex.y, max_vertex.z));
	lines.push_back(scm::math::vec3f(min_vertex.x, min_vertex.y, min_vertex.z));


	lines.push_back(scm::math::vec3f(min_vertex.x, max_vertex.y, min_vertex.z));
	lines.push_back(scm::math::vec3f(max_vertex.x, max_vertex.y, min_vertex.z));

	lines.push_back(scm::math::vec3f(max_vertex.x, max_vertex.y, min_vertex.z));
	lines.push_back(scm::math::vec3f(max_vertex.x, max_vertex.y, max_vertex.z));

	lines.push_back(scm::math::vec3f(max_vertex.x, max_vertex.y, max_vertex.z));
	lines.push_back(scm::math::vec3f(min_vertex.x, max_vertex.y, max_vertex.z));

	lines.push_back(scm::math::vec3f(min_vertex.x, max_vertex.y, max_vertex.z));
	lines.push_back(scm::math::vec3f(min_vertex.x, max_vertex.y, min_vertex.z));


	lines.push_back(scm::math::vec3f(min_vertex.x, min_vertex.y, min_vertex.z));
	lines.push_back(scm::math::vec3f(min_vertex.x, max_vertex.y, min_vertex.z));

	lines.push_back(scm::math::vec3f(max_vertex.x, min_vertex.y, min_vertex.z));
	lines.push_back(scm::math::vec3f(max_vertex.x, max_vertex.y, min_vertex.z));

	lines.push_back(scm::math::vec3f(max_vertex.x, min_vertex.y, max_vertex.z));
	lines.push_back(scm::math::vec3f(max_vertex.x, max_vertex.y, max_vertex.z));

	lines.push_back(scm::math::vec3f(min_vertex.x, min_vertex.y, max_vertex.z));
	lines.push_back(scm::math::vec3f(min_vertex.x, max_vertex.y, max_vertex.z));
}


void LamurePointCloudPlugin::draw_all_models(const lamure::context_t context_id, const lamure::view_t view_id, scm::math::mat4d view_matrix, scm::math::mat4d projection_matrix, scm::gl::program_ptr shader) {

	//lamure::ren::controller* controller = lamure::ren::controller::get_instance();
	//lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
	//lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
	//lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();

	//context_->apply();

	//// ab hier: draw_all_models
	//			// bind vertex_buffer
	//if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
	//	context_->bind_vertex_array(controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, device_, data_provenance_));
	//}
	//else {
	//	context_->bind_vertex_array(controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, device_));
	//}
	//rendered_splats_ = 0;
	//rendered_nodes_ = 0;
	//for (uint16_t model_id = 0; model_id < num_models_; ++model_id) {
	//	lamure::context_t context_id = controller->deduce_context_id(lmr_ctx);
	//	lamure::ren::cut& cut = cuts->get_cut(context_id, lmr_ctx, model_id);
	//	std::vector<lamure::ren::cut::node_slot_aggregate> renderable = cut.complete_set();
	//	const lamure::ren::bvh* bvh = database->get_model(model_id)->get_bvh();
	//	size_t surfels_per_node = database->get_primitives_per_node();
	//	std::vector<scm::gl::boxf>const& bounding_box_vector = bvh->get_bounding_boxes();
	//	scm::math::mat4d model_matrix = model_info_.model_transformations_[model_id];
	//	scm::gl::frustum frustum_by_model = scm_camera_->get_frustum_by_model(scm::math::mat4f(model_matrix));
	//	//uniforms per model
	//	scm::math::mat4d projection_matrix = scm::math::mat4d(view_matrix);
	//	scm::math::mat4d view_matrix = projection_matrix;
	//	scm::math::mat4d model_view_matrix = view_matrix * model_matrix;
	//	scm::math::mat4d model_view_projection_matrix = projection_matrix * model_view_matrix;
	//	shader->uniform("mvp_matrix", scm::math::mat4f(model_view_projection_matrix));
	//	shader->uniform("model_matrix", scm::math::mat4f(model_matrix));
	//	shader->uniform("model_view_matrix", scm::math::mat4f(model_view_matrix));
	//	shader->uniform("inv_mv_matrix", scm::math::mat4f(scm::math::transpose(scm::math::inverse(model_view_matrix))));
	//	const scm::math::mat4d viewport_scale = scm::math::make_scale(traits->width * 0.5, traits->height * 0.5, 0.5);
	//	const scm::math::mat4d viewport_translate = scm::math::make_translation(1.0, 1.0, 1.0);
	//	const scm::math::mat4d model_to_screen = viewport_scale * viewport_translate;
	//	shader->uniform("model_to_screen_matrix", scm::math::mat4f(model_to_screen));
	//	shader->uniform("model_radius_scale", settings_.scale_radius_);
	//	context_->apply_uniform_buffer_bindings();
	//	context_->bind_program(shader);
	//	context_->set_blend_state(color_no_blending_state_);
	//	context_->set_depth_stencil_state(depth_state_less_);
	//	context_->apply_state_objects();
	//	context_->apply_program();
	//	glEnable(GL_PROGRAM_POINT_SIZE);
	//	glEnable(GL_POINT_SMOOTH);
	//	bool draw = true;
	//	for (auto const& node_slot_aggregate : renderable) {
	//		uint32_t node_culling_result = scm_camera_->cull_against_frustum(frustum_by_model, bounding_box_vector[node_slot_aggregate.node_id_]);
	//		if (node_culling_result != 1) {
	//			if (draw) {
	//				//glDrawArrays(GL_POINTS, (node_slot_aggregate.slot_id_) * (GLsizei)surfels_per_node, surfels_per_node);
	//				context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, (node_slot_aggregate.slot_id_) * (GLsizei)surfels_per_node, surfels_per_node);
	//				rendered_splats_ += surfels_per_node;
	//				++rendered_nodes_;
	//			}
	//		}
	//	}
	//}
}


int LamurePointCloudPlugin::unloadLMR(const char* filename, const char* covise_key)
{
	return 1;
}


LamurePointCloudPlugin::~LamurePointCloudPlugin()
{
	fprintf(stderr, "LamurePlugin::~LamurePlugin\n");
	coVRFileManager::instance()->unregisterFileHandler(&handler);
	cover->getObjectsRoot()->removeChild(LamureGroup);
}


void debug_callback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* param) {
	switch (severity) {
	case GL_DEBUG_SEVERITY_HIGH: {
		fprintf(stderr, "GL_DEBUG_SEVERITY_HIGH: %s type = 0x%x, message = %s\n", (type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : ""), type, message);
	}
							   break;
	case GL_DEBUG_SEVERITY_MEDIUM: {
		fprintf(stderr, "GL_DEBUG_SEVERITY_MEDIUM: %s type = 0x%x, message = %s\n", (type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : ""), type, message);
	}
								 break;
	case GL_DEBUG_SEVERITY_LOW: {
		fprintf(stderr, "GL_DEBUG_SEVERITY_LOW: %s type = 0x%x, message = %s\n", (type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : ""), type, message);
	}
							  break;
	default:
		break;
	}
}


void LamurePointCloudPlugin::draw_brush(scm::gl::program_ptr shader) {
	if (selection_.brush_end_ > 0) {
		set_lamure_uniforms(shader);
		scm::math::mat4d model_matrix = scm::math::mat4d::identity();
		scm::math::mat4d projection_matrix = scm::math::mat4d(scm_camera_->get_projection_matrix());
		scm::math::mat4d view_matrix = scm_camera_->get_high_precision_view_matrix();
		scm::math::mat4d model_view_matrix = view_matrix * model_matrix;
		scm::math::mat4d model_view_projection_matrix = projection_matrix * model_view_matrix;

		shader->uniform("mvp_matrix", scm::math::mat4f(model_view_projection_matrix));
		shader->uniform("model_matrix", scm::math::mat4f(model_matrix));
		shader->uniform("model_view_matrix", scm::math::mat4f(model_view_matrix));
		shader->uniform("inv_mv_matrix", scm::math::mat4f(scm::math::transpose(scm::math::inverse(model_view_matrix))));

		shader->uniform("point_size_factor", settings_.aux_point_scale_);

		shader->uniform("model_to_screen_matrix", scm::math::mat4f::identity());
		shader->uniform("model_radius_scale", 1.0f);

		shader->uniform("show_normals", false);
		shader->uniform("show_accuracy", false);
		shader->uniform("show_radius_deviation", false);
		shader->uniform("show_output_sensitivity", false);
		shader->uniform("channel", 0);

		shader->uniform("face_eye", false);

		context_->bind_vertex_array(brush_resource_.array_);
		context_->apply();
		context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 0, selection_.brush_end_);
	}
}


void LamurePointCloudPlugin::lines_from_min_max_buffered(const scm::math::vec3f& min_vertex, const scm::math::vec3f& max_vertex, osg::ref_ptr<osg::Vec3Array>& lines) {

	lines->push_back(osg::Vec3f(min_vertex.x, min_vertex.y, min_vertex.z));
	lines->push_back(osg::Vec3f(max_vertex.x, min_vertex.y, min_vertex.z));

	lines->push_back(osg::Vec3f(max_vertex.x, min_vertex.y, min_vertex.z));
	lines->push_back(osg::Vec3f(max_vertex.x, min_vertex.y, max_vertex.z));

	lines->push_back(osg::Vec3f(max_vertex.x, min_vertex.y, max_vertex.z));
	lines->push_back(osg::Vec3f(min_vertex.x, min_vertex.y, max_vertex.z));

	lines->push_back(osg::Vec3f(min_vertex.x, min_vertex.y, max_vertex.z));
	lines->push_back(osg::Vec3f(min_vertex.x, min_vertex.y, min_vertex.z));


	lines->push_back(osg::Vec3f(min_vertex.x, max_vertex.y, min_vertex.z));
	lines->push_back(osg::Vec3f(max_vertex.x, max_vertex.y, min_vertex.z));

	lines->push_back(osg::Vec3f(max_vertex.x, max_vertex.y, min_vertex.z));
	lines->push_back(osg::Vec3f(max_vertex.x, max_vertex.y, max_vertex.z));

	lines->push_back(osg::Vec3f(max_vertex.x, max_vertex.y, max_vertex.z));
	lines->push_back(osg::Vec3f(min_vertex.x, max_vertex.y, max_vertex.z));

	lines->push_back(osg::Vec3f(min_vertex.x, max_vertex.y, max_vertex.z));
	lines->push_back(osg::Vec3f(min_vertex.x, max_vertex.y, min_vertex.z));


	lines->push_back(osg::Vec3f(min_vertex.x, min_vertex.y, min_vertex.z));
	lines->push_back(osg::Vec3f(min_vertex.x, max_vertex.y, min_vertex.z));

	lines->push_back(osg::Vec3f(max_vertex.x, min_vertex.y, min_vertex.z));
	lines->push_back(osg::Vec3f(max_vertex.x, max_vertex.y, min_vertex.z));

	lines->push_back(osg::Vec3f(max_vertex.x, min_vertex.y, max_vertex.z));
	lines->push_back(osg::Vec3f(max_vertex.x, max_vertex.y, max_vertex.z));

	lines->push_back(osg::Vec3f(min_vertex.x, min_vertex.y, max_vertex.z));
	lines->push_back(osg::Vec3f(min_vertex.x, max_vertex.y, max_vertex.z));
}


void LamurePointCloudPlugin::create_aux_representation() {
	for (const auto& aux_file : settings_.aux_) {
		if (aux_file.second != "") {

			uint32_t model_id = aux_file.first;

			std::cout << "aux: " << aux_file.second << std::endl;
			lamure::prov::aux aux(aux_file.second);

			provenance_[model_id].num_views_ = aux.get_num_views();
			std::cout << "aux: " << aux.get_num_views() << " views" << std::endl;
			std::cout << "aux: " << aux.get_num_sparse_points() << " points" << std::endl;
			std::cout << "aux: " << aux.get_atlas().atlas_width_ << ", " << aux.get_atlas().atlas_height_ << " is it rotated? : " << aux.get_atlas().rotated_ << std::endl;
			std::cout << "aux: " << aux.get_num_atlas_tiles() << " atlas tiles" << std::endl;

			std::vector<xyz> points_to_upload;

			for (uint32_t i = 0; i < aux.get_num_views(); ++i) {
				const auto& view = aux.get_view(i);
				points_to_upload.push_back(
					xyz{ view.position_,
					(uint8_t)255, (uint8_t)240, (uint8_t)0, (uint8_t)255,
					settings_.aux_point_size_,
					scm::math::vec3f(1.0, 0.0, 0.0) } //placeholder
				);
				settings_.views_[model_id].push_back(view);
			}

			for (uint32_t i = 0; i < aux.get_num_sparse_points(); ++i) {
				const auto& point = aux.get_sparse_point(i);
				points_to_upload.push_back(
					xyz{ point.pos_,
					point.r_, point.g_, point.b_, point.a_,
					settings_.aux_point_size_,
					scm::math::vec3f(1.0, 0.0, 0.0) } //placeholder
				);
			}

			resource points_resource;
			points_resource.num_primitives_ = points_to_upload.size();
			points_resource.buffer_.reset();
			points_resource.array_.reset();

			points_resource.buffer_ = device_->create_buffer(
				scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, sizeof(xyz) * points_to_upload.size(), &points_to_upload[0]);
			points_resource.array_ = device_->create_vertex_array(scm::gl::vertex_format
			(0, 0, scm::gl::TYPE_VEC3F, sizeof(xyz))
				(0, 1, scm::gl::TYPE_UBYTE, sizeof(xyz), scm::gl::INT_FLOAT_NORMALIZE)
				(0, 2, scm::gl::TYPE_UBYTE, sizeof(xyz), scm::gl::INT_FLOAT_NORMALIZE)
				(0, 3, scm::gl::TYPE_UBYTE, sizeof(xyz), scm::gl::INT_FLOAT_NORMALIZE)
				(0, 4, scm::gl::TYPE_UBYTE, sizeof(xyz), scm::gl::INT_FLOAT_NORMALIZE)
				(0, 5, scm::gl::TYPE_FLOAT, sizeof(xyz))
				(0, 6, scm::gl::TYPE_VEC3F, sizeof(xyz)),
				boost::assign::list_of(points_resource.buffer_));

			sparse_res_[model_id] = points_resource;

			//init octree
			settings_.octrees_[model_id] = aux.get_octree();
			std::cout << "Octree loaded (" << settings_.octrees_[model_id]->get_num_nodes() << " nodes)" << std::endl;

			//init octree buffers
			resource octree_resource;
			octree_resource.buffer_.reset();
			octree_resource.array_.reset();

			std::vector<scm::math::vec3f> octree_lines_to_upload;
			for (uint64_t i = 0; i < settings_.octrees_[model_id]->get_num_nodes(); ++i) {
				const auto& node = settings_.octrees_[model_id]->get_node(i);

				lines_from_min_max(node.get_min(), node.get_max(), octree_lines_to_upload);
			}

			octree_resource.buffer_ = device_->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
				scm::gl::USAGE_STATIC_DRAW, (sizeof(float) * 3) * octree_lines_to_upload.size(), &octree_lines_to_upload[0]);
			octree_resource.array_ = device_->create_vertex_array(scm::gl::vertex_format
			(0, 0, scm::gl::TYPE_VEC3F, sizeof(float) * 3),
				boost::assign::list_of(octree_resource.buffer_));

			octree_resource.num_primitives_ = octree_lines_to_upload.size();
			octree_res_[model_id] = octree_resource;

			auto root_bb = lamure::ren::model_database::get_instance()->get_model(model_id)->get_bvh()->get_bounding_boxes()[0];
			auto root_bb_min = scm::math::mat4f(model_info_.model_transformations_[model_id]) * root_bb.min_vertex();
			auto root_bb_max = scm::math::mat4f(model_info_.model_transformations_[model_id]) * root_bb.max_vertex();
			auto model_dim = scm::math::length(root_bb_max - root_bb_min);

			//for image planes
			if (!settings_.atlas_file_.empty()) {
				if (aux.get_num_atlas_tiles() != aux.get_num_views()) {
					throw std::runtime_error(
						"Number of atlas_tiles (" + std::to_string(aux.get_num_atlas_tiles()) + ") "
						+ "does not match number of views (" + std::to_string(aux.get_num_views()) + ")");
				}

				std::vector<vertex> triangles_to_upload;
				for (uint32_t i = 0; i < aux.get_num_views(); ++i) {
					const auto& view = aux.get_view(i);
					const auto& atlas_tile = aux.get_atlas_tile(i);

					float aspect_ratio = view.image_height_ / (float)view.image_width_;
					float img_w_half = (settings_.aux_focal_length_) * 0.5f;
					float img_h_half = img_w_half * aspect_ratio;
					float focal_length = settings_.aux_focal_length_;

					float atlas_width = aux.get_atlas().atlas_width_;
					float atlas_height = aux.get_atlas().atlas_height_;

					// scale factor from image space to vt atlas space
					float factor = get_atlas_scale_factor();

					// positions in vt atlas space coordinate system
					float tile_height = (float)atlas_tile.width_ / atlas_width * factor;
					float tile_width = (float)atlas_tile.width_ / atlas_height * factor;

					float tile_pos_x = (float)atlas_tile.x_ / atlas_height * factor;
					float tile_pos_y = (float)atlas_tile.y_ / atlas_tile.height_ * tile_height + (1 - factor);


					vertex p1;
					p1.pos_ = view.transform_ * scm::math::vec3f(-img_w_half, img_h_half, -focal_length);
					p1.uv_ = scm::math::vec2f(tile_pos_x + tile_width, tile_pos_y);

					vertex p2;
					p2.pos_ = view.transform_ * scm::math::vec3f(img_w_half, img_h_half, -focal_length);
					p2.uv_ = scm::math::vec2f(tile_pos_x, tile_pos_y);

					vertex p3;
					p3.pos_ = view.transform_ * scm::math::vec3f(-img_w_half, -img_h_half, -focal_length);
					p3.uv_ = scm::math::vec2f(tile_pos_x + tile_width, tile_pos_y + tile_height);

					vertex p4;
					p4.pos_ = view.transform_ * scm::math::vec3f(img_w_half, -img_h_half, -focal_length);
					p4.uv_ = scm::math::vec2f(tile_pos_x, tile_pos_y + tile_height);

					// left quad triangle
					triangles_to_upload.push_back(p1);
					triangles_to_upload.push_back(p4);
					triangles_to_upload.push_back(p3);

					// right quad triangle
					triangles_to_upload.push_back(p2);
					triangles_to_upload.push_back(p4);
					triangles_to_upload.push_back(p1);
				}

				//init triangle buffer
				resource triangles_resource;
				triangles_resource.buffer_.reset();
				triangles_resource.array_.reset();

				triangles_resource.buffer_ = device_->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
					scm::gl::USAGE_STATIC_DRAW,
					(sizeof(vertex)) * triangles_to_upload.size(),
					&triangles_to_upload[0]);

				triangles_resource.array_ = device_->create_vertex_array(scm::gl::vertex_format
				(0, 0, scm::gl::TYPE_VEC3F, sizeof(vertex))
					(0, 1, scm::gl::TYPE_VEC2F, sizeof(vertex)),
					boost::assign::list_of(triangles_resource.buffer_));

				triangles_resource.num_primitives_ = triangles_to_upload.size();

				image_plane_res_[model_id] = triangles_resource;
			}

			//init line buffers
			resource lines_resource;
			lines_resource.buffer_.reset();
			lines_resource.array_.reset();

			std::vector<scm::math::vec3f> lines_to_upload;
			for (uint32_t i = 0; i < aux.get_num_views(); ++i) {
				const auto& view = aux.get_view(i);

				float aspect_ratio = view.image_height_ / (float)view.image_width_;
				float img_w_half = (settings_.aux_focal_length_) * 0.5f;
				float img_h_half = img_w_half * aspect_ratio;
				float focal_length = settings_.aux_focal_length_;

				lines_to_upload.push_back(view.transform_ * scm::math::vec3f(-img_w_half, img_h_half, -focal_length));
				lines_to_upload.push_back(view.transform_ * scm::math::vec3f(img_w_half, img_h_half, -focal_length));

				lines_to_upload.push_back(view.transform_ * scm::math::vec3f(img_w_half, img_h_half, -focal_length));
				lines_to_upload.push_back(view.transform_ * scm::math::vec3f(img_w_half, -img_h_half, -focal_length));

				lines_to_upload.push_back(view.transform_ * scm::math::vec3f(img_w_half, -img_h_half, -focal_length));
				lines_to_upload.push_back(view.transform_ * scm::math::vec3f(-img_w_half, -img_h_half, -focal_length));

				lines_to_upload.push_back(view.transform_ * scm::math::vec3f(-img_w_half, -img_h_half, -focal_length));
				lines_to_upload.push_back(view.transform_ * scm::math::vec3f(-img_w_half, img_h_half, -focal_length));

				lines_to_upload.push_back(view.transform_ * scm::math::vec3f(0.f));
				lines_to_upload.push_back(view.transform_ * scm::math::vec3f(-img_w_half, img_h_half, -focal_length));

				lines_to_upload.push_back(view.transform_ * scm::math::vec3f(0.f));
				lines_to_upload.push_back(view.transform_ * scm::math::vec3f(img_w_half, img_h_half, -focal_length));

				lines_to_upload.push_back(view.transform_ * scm::math::vec3f(0.f));
				lines_to_upload.push_back(view.transform_ * scm::math::vec3f(img_w_half, -img_h_half, -focal_length));

				lines_to_upload.push_back(view.transform_ * scm::math::vec3f(0.f));
				lines_to_upload.push_back(view.transform_ * scm::math::vec3f(-img_w_half, -img_h_half, -focal_length));

			}

			lines_resource.buffer_ = device_->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
				scm::gl::USAGE_STATIC_DRAW, (sizeof(float) * 3) * lines_to_upload.size(), &lines_to_upload[0]);
			lines_resource.array_ = device_->create_vertex_array(scm::gl::vertex_format
			(0, 0, scm::gl::TYPE_VEC3F, sizeof(float) * 3),
				boost::assign::list_of(lines_resource.buffer_));

			lines_resource.num_primitives_ = lines_to_upload.size();

			//frustum_resource_ = lines_resource;
		}
	}
}


void LamurePointCloudPlugin::draw_resources(const lamure::context_t context_id, const lamure::view_t view_id) {

	if (settings_.show_bvhs_) {
		lamure::ren::controller* controller = lamure::ren::controller::get_instance();
		lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
		lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
		lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();

		context_->bind_program(vis_line_shader_);

		scm::math::mat4f projection_matrix = scm::math::mat4f(scm_camera_->get_projection_matrix());
		scm::math::mat4f view_matrix = scm_camera_->get_view_matrix();

		vis_line_shader_->uniform("view_matrix", view_matrix);
		vis_line_shader_->uniform("projection_matrix", projection_matrix);
		for (uint16_t model_id = 0; model_id < num_models_; ++model_id) {
			if (selection_.selected_model_ != -1) {
				model_id = selection_.selected_model_;
			}

			bool draw = true;
			lamure::model_t m_id = controller->deduce_model_id(std::to_string(model_id));
			lamure::ren::cut& cut = cuts->get_cut(context_id, view_id, m_id);
			std::vector<lamure::ren::cut::node_slot_aggregate> renderable = cut.complete_set();
			const lamure::ren::bvh* bvh = database->get_model(m_id)->get_bvh();
			if (bvh->get_primitive() != lamure::ren::bvh::primitive_type::POINTCLOUD) {
				if (selection_.selected_model_ != -1) break;
				else draw = false;
			}

			if (draw) {
				//uniforms per model
				scm::math::mat4d model_matrix = model_info_.model_transformations_[model_id];
				vis_line_shader_->uniform("model_matrix", scm::math::mat4f(model_matrix));

				std::vector<scm::gl::boxf>const& bounding_box_vector = bvh->get_bounding_boxes();
				scm::gl::frustum frustum_ = scm_camera_->get_frustum();

				auto bvh_res = bvh_res_[model_id];
				if (bvh_res.num_primitives_ > 0) {
					context_->bind_vertex_array(bvh_res.array_);
					context_->apply();
					for (auto const& node_slot_aggregate : renderable) {
						uint32_t node_culling_result = scm_camera_->cull_against_frustum(frustum_, bounding_box_vector[node_slot_aggregate.node_id_]);
						if (node_culling_result != 1) {
							if (settings_.use_pvs_ && pvs->is_activated() && settings_.pvs_culling_
								&& !lamure::pvs::pvs_database::get_instance()->get_viewer_visibility(model_id, node_slot_aggregate.node_id_)) {
								continue;
							}
							context_->draw_arrays(scm::gl::PRIMITIVE_LINE_LIST, node_slot_aggregate.node_id_ * 24, 24);
						}
					}
				}
			}
			if (selection_.selected_model_ != -1) {
				break;
			}
		}
	}
	if (sparse_res_.size() > 0) {
		if ((settings_.show_sparse_ || settings_.show_views_) && sparse_res_.size() > 0) {
			context_->bind_program(vis_xyz_shader_);
			context_->set_blend_state(color_no_blending_state_);
			context_->set_depth_stencil_state(depth_state_less_);

			LamurePointCloudPlugin::instance()->set_lamure_uniforms(vis_xyz_shader_);

			scm::math::mat4d model_matrix = scm::math::mat4d::identity();
			scm::math::mat4d projection_matrix = scm::math::mat4d(scm_camera_->get_projection_matrix());
			scm::math::mat4d view_matrix = scm_camera_->get_high_precision_view_matrix();
			scm::math::mat4d model_view_matrix = view_matrix * model_matrix;
			scm::math::mat4d model_view_projection_matrix = projection_matrix * model_view_matrix;

			vis_xyz_shader_->uniform("mvp_matrix", scm::math::mat4f(model_view_projection_matrix));
			vis_xyz_shader_->uniform("model_matrix", scm::math::mat4f(model_matrix));
			vis_xyz_shader_->uniform("model_view_matrix", scm::math::mat4f(model_view_matrix));
			vis_xyz_shader_->uniform("inv_mv_matrix", scm::math::mat4f(scm::math::transpose(scm::math::inverse(model_view_matrix))));
			vis_xyz_shader_->uniform("point_size_factor", settings_.aux_point_scale_);
			vis_xyz_shader_->uniform("model_to_screen_matrix", scm::math::mat4f::identity());
			vis_xyz_shader_->uniform("model_radius_scale", 1.f);

			scm::math::mat4f inv_view = scm::math::inverse(scm::math::mat4f(view_matrix));
			scm::math::vec3f eye = scm::math::vec3f(inv_view[12], inv_view[13], inv_view[14]);

			vis_xyz_shader_->uniform("eye", eye);
			vis_xyz_shader_->uniform("face_eye", true);
			vis_xyz_shader_->uniform("show_normals", false);
			vis_xyz_shader_->uniform("show_accuracy", false);
			vis_xyz_shader_->uniform("show_radius_deviation", false);
			vis_xyz_shader_->uniform("show_output_sensitivity", false);
			vis_xyz_shader_->uniform("channel", 0);

			for (uint16_t model_id = 0; model_id < num_models_; ++model_id) {
				if (selection_.selected_model_ != -1) {
					model_id = selection_.selected_model_;
				}
				auto s_res = sparse_res_[model_id];
				if (s_res.num_primitives_ > 0) {
					context_->bind_vertex_array(s_res.array_);
					context_->apply();
					uint16_t num_views = provenance_[model_id].num_views_;
					if (settings_.show_views_) {
						if (selection_.selected_views_.empty()) {
							context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 0, num_views);
						}
						else {
							for (const auto view : selection_.selected_views_) {
								context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, view, 1);
							}
						}
					}
					if (settings_.show_sparse_) {
						context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, num_views, s_res.num_primitives_ - num_views);
					}
				}
				if (selection_.selected_model_ != -1) {
					break;
				}
			}
		}
		// draw image_plane resources with vt system
		if (settings_.show_photos_ && !settings_.atlas_file_.empty()) {
			context_->bind_program(vis_vt_shader_);
			uint64_t color_cut_id = (((uint64_t)vt_.texture_id_) << 32) | ((uint64_t)vt_.view_id_ << 16) | ((uint64_t)vt_.context_id_);
			uint32_t max_depth_level_color = (*vt::CutDatabase::get_instance().get_cut_map())[color_cut_id]->get_atlas()->getDepth() - 1;
			scm::math::mat4f view_matrix = scm_camera_->get_view_matrix();
			scm::math::mat4f projection_matrix = scm::math::mat4f(scm_camera_->get_projection_matrix());
			vis_vt_shader_->uniform("model_view_matrix", view_matrix);
			vis_vt_shader_->uniform("projection_matrix", projection_matrix);
			vis_vt_shader_->uniform("physical_texture_dim", vt_.physical_texture_size_);
			vis_vt_shader_->uniform("max_level", max_depth_level_color);
			vis_vt_shader_->uniform("tile_size", scm::math::vec2((uint32_t)vt::VTConfig::get_instance().get_size_tile()));
			vis_vt_shader_->uniform("tile_padding", scm::math::vec2((uint32_t)vt::VTConfig::get_instance().get_size_padding()));
			vis_vt_shader_->uniform("enable_hierarchy", vt_.enable_hierarchy_);
			vis_vt_shader_->uniform("toggle_visualization", vt_.toggle_visualization_);

			for (uint32_t i = 0; i < vt_.index_texture_hierarchy_.size(); ++i) {
				std::string texture_string = "hierarchical_idx_textures";
				vis_vt_shader_->uniform(texture_string, i, int((i)));
			}
			vis_vt_shader_->uniform("physical_texture_array", 17);
			context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(traits->width, traits->height)));
			context_->set_depth_stencil_state(depth_state_less_);
			context_->set_rasterizer_state(no_backface_culling_rasterizer_state_);
			context_->set_blend_state(color_no_blending_state_);
			context_->sync();
			//apply_vt_cut_update();

			for (uint16_t i = 0; i < vt_.index_texture_hierarchy_.size(); ++i) {
				context_->bind_texture(vt_.index_texture_hierarchy_.at(i), vt_filter_nearest_, i);
			}
			context_->bind_texture(vt_.physical_texture_, vt_filter_linear_, 17);
			context_->bind_storage_buffer(vt_.feedback_lod_storage_, 0);
			context_->bind_storage_buffer(vt_.feedback_count_storage_, 1);
			context_->apply();
			for (uint16_t model_id = 0; model_id < num_models_; ++model_id) {
				if (selection_.selected_model_ != -1) {
					model_id = selection_.selected_model_;
				}
				auto t_res = image_plane_res_[model_id];

				if (t_res.num_primitives_ > 0) {
					context_->bind_vertex_array(t_res.array_);
					context_->apply();
					if (selection_.selected_views_.empty()) {
						context_->draw_arrays(scm::gl::PRIMITIVE_TRIANGLE_LIST, 0, t_res.num_primitives_);
					}
					else {
						for (const auto view : selection_.selected_views_) {
							context_->draw_arrays(scm::gl::PRIMITIVE_TRIANGLE_LIST, view * 6, 6);
						}
					}
				}
				if (selection_.selected_model_ != -1) {
					break;
				}
			}
			context_->sync();
			//collect_vt_feedback();
		}
	}
	if (settings_.pvs_ != "" && settings_.show_pvs_) {
		if (pvs_resource_.num_primitives_ > 0) {
			context_->bind_program(vis_line_shader_);

			scm::math::mat4f projection_matrix = scm::math::mat4f(scm_camera_->get_projection_matrix());
			scm::math::mat4f view_matrix = scm_camera_->get_view_matrix();
			vis_line_shader_->uniform("model_matrix", scm::math::mat4f::identity());
			vis_line_shader_->uniform("view_matrix", view_matrix);
			vis_line_shader_->uniform("projection_matrix", projection_matrix);

			context_->bind_vertex_array(pvs_resource_.array_);
			context_->apply();
			context_->draw_arrays(scm::gl::PRIMITIVE_LINE_LIST, 0, pvs_resource_.num_primitives_);
		}
	}
}


float LamurePointCloudPlugin::get_atlas_scale_factor() {
	auto atlas = new vt::pre::AtlasFile(settings_.atlas_file_.c_str());
	uint64_t image_width = atlas->getImageWidth();
	uint64_t image_height = atlas->getImageHeight();

	// tile's width and height without padding
	uint64_t tile_inner_width = atlas->getInnerTileWidth();
	uint64_t tile_inner_height = atlas->getInnerTileHeight();

	// Quadtree depth counter, ranges from 0 to depth-1
	uint64_t depth = atlas->getDepth();

	double factor_u = (double)image_width / (tile_inner_width * std::pow(2, depth - 1));
	double factor_v = (double)image_height / (tile_inner_height * std::pow(2, depth - 1));

	return std::max(factor_u, factor_v);
}


/*
void LamurePointCloudPlugin::ImplGlfwGL3_RenderDrawLists(draw_data) {
	ImGuiIO& io = ImGui::GetIO();
	int fb_width = (int)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
	int fb_height = (int)(io.DisplaySize.y * io.DisplayFramebufferScale.y);
	if (fb_width == 0 || fb_height == 0)
		return;
	draw_data->ScaleClipRects(io.DisplayFramebufferScale);
	// Backup GL state
	GLenum last_active_texture; glGetIntegerv(GL_ACTIVE_TEXTURE, (GLint*)&last_active_texture);
	glActiveTexture(GL_TEXTURE0);
	GLint last_program; glGetIntegerv(GL_CURRENT_PROGRAM, &last_program);
	GLint last_texture; glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
	GLint last_sampler; glGetIntegerv(GL_SAMPLER_BINDING, &last_sampler);
	GLint last_array_buffer; glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &last_array_buffer);
	GLint last_element_array_buffer; glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, &last_element_array_buffer);
	GLint last_vertex_array; glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &last_vertex_array);
	GLint last_polygon_mode[2]; glGetIntegerv(GL_POLYGON_MODE, last_polygon_mode);
	GLint last_viewport[4]; glGetIntegerv(GL_VIEWPORT, last_viewport);
	GLint last_scissor_box[4]; glGetIntegerv(GL_SCISSOR_BOX, last_scissor_box);
	GLenum last_blend_src_rgb; glGetIntegerv(GL_BLEND_SRC_RGB, (GLint*)&last_blend_src_rgb);
	GLenum last_blend_dst_rgb; glGetIntegerv(GL_BLEND_DST_RGB, (GLint*)&last_blend_dst_rgb);
	GLenum last_blend_src_alpha; glGetIntegerv(GL_BLEND_SRC_ALPHA, (GLint*)&last_blend_src_alpha);
	GLenum last_blend_dst_alpha; glGetIntegerv(GL_BLEND_DST_ALPHA, (GLint*)&last_blend_dst_alpha);
	GLenum last_blend_equation_rgb; glGetIntegerv(GL_BLEND_EQUATION_RGB, (GLint*)&last_blend_equation_rgb);
	GLenum last_blend_equation_alpha; glGetIntegerv(GL_BLEND_EQUATION_ALPHA, (GLint*)&last_blend_equation_alpha);
	GLboolean last_enable_blend = glIsEnabled(GL_BLEND);
	GLboolean last_enable_cull_face = glIsEnabled(GL_CULL_FACE);
	GLboolean last_enable_depth_test = glIsEnabled(GL_DEPTH_TEST);
	GLboolean last_enable_scissor_test = glIsEnabled(GL_SCISSOR_TEST);

	// Setup render state: alpha-blending enabled, no face culling, no depth testing, scissor enabled, polygon fill
	glEnable(GL_BLEND);
	glBlendEquation(GL_FUNC_ADD);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_SCISSOR_TEST);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Setup viewport, orthographic projection matrix
	glViewport(0, 0, (GLsizei)fb_width, (GLsizei)fb_height);
	const float ortho_projection[4][4] =
	{
		{ 2.0f / io.DisplaySize.x, 0.0f,                   0.0f, 0.0f },
		{ 0.0f,                  2.0f / -io.DisplaySize.y, 0.0f, 0.0f },
		{ 0.0f,                  0.0f,                  -1.0f, 0.0f },
		{-1.0f,                  1.0f,                   0.0f, 1.0f },
	};
	glUseProgram(g_ShaderHandle);
	glUniform1i(g_AttribLocationTex, 0);
	glUniformMatrix4fv(g_AttribLocationProjMtx, 1, GL_FALSE, &ortho_projection[0][0]);
	glBindVertexArray(g_VaoHandle);
	glBindSampler(0, 0); // Rely on combined texture/sampler state.

	for (int n = 0; n < draw_data->CmdListsCount; n++)
	{
		const ImDrawList* cmd_list = draw_data->CmdLists[n];
		const ImDrawIdx* idx_buffer_offset = 0;

		glBindBuffer(GL_ARRAY_BUFFER, g_VboHandle);
		glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)cmd_list->VtxBuffer.Size * sizeof(ImDrawVert), (const GLvoid*)cmd_list->VtxBuffer.Data, GL_STREAM_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g_ElementsHandle);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)cmd_list->IdxBuffer.Size * sizeof(ImDrawIdx), (const GLvoid*)cmd_list->IdxBuffer.Data, GL_STREAM_DRAW);

		for (int cmd_i = 0; cmd_i < cmd_list->CmdBuffer.Size; cmd_i++)
		{
			const ImDrawCmd* pcmd = &cmd_list->CmdBuffer[cmd_i];
			if (pcmd->UserCallback)
			{
				pcmd->UserCallback(cmd_list, pcmd);
			}
			else
			{
				glBindTexture(GL_TEXTURE_2D, (GLuint)(intptr_t)pcmd->TextureId);
				glScissor((int)pcmd->ClipRect.x, (int)(fb_height - pcmd->ClipRect.w), (int)(pcmd->ClipRect.z - pcmd->ClipRect.x), (int)(pcmd->ClipRect.w - pcmd->ClipRect.y));
				glDrawElements(GL_TRIANGLES, (GLsizei)pcmd->ElemCount, sizeof(ImDrawIdx) == 2 ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT, idx_buffer_offset);
			}
			idx_buffer_offset += pcmd->ElemCount;
		}
	}
	// Restore modified GL state
	glUseProgram(last_program);
	glBindTexture(GL_TEXTURE_2D, last_texture);
	glBindSampler(0, last_sampler);
	glActiveTexture(last_active_texture);
	glBindVertexArray(last_vertex_array);
	glBindBuffer(GL_ARRAY_BUFFER, last_array_buffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, last_element_array_buffer);
	glBlendEquationSeparate(last_blend_equation_rgb, last_blend_equation_alpha);
	glBlendFuncSeparate(last_blend_src_rgb, last_blend_dst_rgb, last_blend_src_alpha, last_blend_dst_alpha);
	if (last_enable_blend) glEnable(GL_BLEND); else glDisable(GL_BLEND);
	if (last_enable_cull_face) glEnable(GL_CULL_FACE); else glDisable(GL_CULL_FACE);
	if (last_enable_depth_test) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
	if (last_enable_scissor_test) glEnable(GL_SCISSOR_TEST); else glDisable(GL_SCISSOR_TEST);
	glPolygonMode(GL_FRONT_AND_BACK, last_polygon_mode[0]);
	glViewport(last_viewport[0], last_viewport[1], (GLsizei)last_viewport[2], (GLsizei)last_viewport[3]);
	glScissor(last_scissor_box[0], last_scissor_box[1], (GLsizei)last_scissor_box[2], (GLsizei)last_scissor_box[3])
}

void LamurePointCloudPlugin::ImplGlfwGL3_CreateDeviceObjects() {
// Backup GL state
	GLint last_texture, last_array_buffer, last_vertex_array;
	glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
	glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &last_array_buffer);
	glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &last_vertex_array);

	const GLchar *vertex_shader =
		"#version 330\n"
		"uniform mat4 ProjMtx;\n"
		"in vec2 Position;\n"
		"in vec2 UV;\n"
		"in vec4 Color;\n"
		"out vec2 Frag_UV;\n"
		"out vec4 Frag_Color;\n"
		"void main()\n"
		"{\n"
		"	Frag_UV = UV;\n"
		"	Frag_Color = Color;\n"
		"	gl_Position = ProjMtx * vec4(Position.xy,0,1);\n"
		"}\n";

	const GLchar* fragment_shader =
		"#version 330\n"
		"uniform sampler2D Texture;\n"
		"in vec2 Frag_UV;\n"
		"in vec4 Frag_Color;\n"
		"out vec4 Out_Color;\n"
		"void main()\n"
		"{\n"
		"	Out_Color = Frag_Color * texture( Texture, Frag_UV.st);\n"
		"}\n";

	g_ShaderHandle = glCreateProgram();
	g_VertHandle = glCreateShader(GL_VERTEX_SHADER);
	g_FragHandle = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(g_VertHandle, 1, &vertex_shader, 0);
	glShaderSource(g_FragHandle, 1, &fragment_shader, 0);
	glCompileShader(g_VertHandle);
	glCompileShader(g_FragHandle);
	glAttachShader(g_ShaderHandle, g_VertHandle);
	glAttachShader(g_ShaderHandle, g_FragHandle);
	glLinkProgram(g_ShaderHandle);

	g_AttribLocationTex = glGetUniformLocation(g_ShaderHandle, "Texture");
	g_AttribLocationProjMtx = glGetUniformLocation(g_ShaderHandle, "ProjMtx");
	g_AttribLocationPosition = glGetAttribLocation(g_ShaderHandle, "Position");
	g_AttribLocationUV = glGetAttribLocation(g_ShaderHandle, "UV");
	g_AttribLocationColor = glGetAttribLocation(g_ShaderHandle, "Color");

	glGenBuffers(1, &g_VboHandle);
	glGenBuffers(1, &g_ElementsHandle);

	glGenVertexArrays(1, &g_VaoHandle);
	glBindVertexArray(g_VaoHandle);
	glBindBuffer(GL_ARRAY_BUFFER, g_VboHandle);
	glEnableVertexAttribArray(g_AttribLocationPosition);
	glEnableVertexAttribArray(g_AttribLocationUV);
	glEnableVertexAttribArray(g_AttribLocationColor);

#define OFFSETOF(TYPE, ELEMENT) ((size_t)&(((TYPE *)0)->ELEMENT))
	glVertexAttribPointer(g_AttribLocationPosition, 2, GL_FLOAT, GL_FALSE, sizeof(ImDrawVert), (GLvoid*)OFFSETOF(ImDrawVert, pos));
	glVertexAttribPointer(g_AttribLocationUV, 2, GL_FLOAT, GL_FALSE, sizeof(ImDrawVert), (GLvoid*)OFFSETOF(ImDrawVert, uv));
	glVertexAttribPointer(g_AttribLocationColor, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(ImDrawVert), (GLvoid*)OFFSETOF(ImDrawVert, col));
#undef OFFSETOF

	ImGui_ImplGlfwGL3_CreateFontsTexture();

	// Restore modified GL state
	glBindTexture(GL_TEXTURE_2D, last_texture);
	glBindBuffer(GL_ARRAY_BUFFER, last_array_buffer);
	glBindVertexArray(last_vertex_array);

	return true;

}
*/

/*osg::View* view = renderInfo.getView();
osg::Camera* cam = view->getCamera();
osg::Matrix objectsXform = cover->getObjectsXform()->getMatrix();
osg::Matrix objectsScale = cover->getObjectsScale()->getMatrix();
osg::Matrix viewerMat = cover->getViewerMat();
osg::Matrix scale_trans = VRSceneGraph::instance()->getScaleTransform()->getMatrix();
float scene_scale = cover->getScale();
osg::BoundingBox bb_gl_grp = cover->getBBox(_gl_grp);
osg::Matrix base_mat = cover->getBaseMat();
float viewer_screen_distance = cover->getViewerScreenDistance();
int acp = cover->getActiveClippingPlane();
osg::ClipPlane* cp = cover->getClipPlane(cover->getActiveClippingPlane());
glMatrixMode(GL_TEXTURE);
std::cout << "matConv4F(VRViewer::instance()->getViewerMat())" << std::endl;
std::cout << matConv4F(VRViewer::instance()->getViewerMat()) << std::endl << std::endl;
std::cout << "vecConv3F(VRViewer::instance()->getViewerPos())" << std::endl;
std::cout << vecConv3F(VRViewer::instance()->getViewerPos()) << std::endl << std::endl;
std::cout << "viewer_screen_distance" << std::endl;
std::cout << viewer_screen_distance << std::endl << std::endl;
std::cout << "matConv4F(scale_trans)" << std::endl;
std::cout << matConv4F(scale_trans) << std::endl << std::endl;
std::cout << "scene_scale" << std::endl;
std::cout << scene_scale << std::endl << std::endl;
std::cout << "base_mat" << std::endl;
std::cout << matConv4F(base_mat) << std::endl << std::endl;
std::cout << "matConv4F(objectsXform)" << std::endl;
std::cout << matConv4F(objectsXform) << std::endl << std::endl;
std::cout << "matConv4F(objectsScale)" << std::endl;
std::cout << matConv4F(objectsScale) << std::endl << std::endl;
std::cout << "matConv4F(viewerMat)" << std::endl;
std::cout << matConv4F(viewerMat) << std::endl << std::endl;
//osgViewer::View* view;
//osg::View *view = renderInfo.getView();
scm::math::mat4f model_matrix = scm::math::mat4f::identity();
auto model = model_matrix.data_array;
scm::math::mat4f view_matrix = matConv4F(osg_camera_->getViewMatrix());
auto view = view_matrix.data_array;
scm::math::mat4f projection_matrix = matConv4F(osg_camera_->getProjectionMatrix());
auto projection = projection_matrix.data_array;
GLuint vboId;
glGenBuffers(1, &vboId);
glBindBuffer(GL_ARRAY_BUFFER, vboId);
glBufferData(GL_ARRAY_BUFFER, sizeof(position), lines, GL_STATIC_DRAW);
glEnableVertexAttribArray(0);
glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), 0);
unsigned int vis_line_shader = glCreateProgram();
unsigned int vs = CompileShader(GL_VERTEX_SHADER, vis_line_vs_source, 0);
unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, vis_line_fs_source, 0);
glAttachShader(vis_line_shader, vs);
glAttachShader(vis_line_shader, fs);
glLinkProgram(vis_line_shader);
glValidateProgram(vis_line_shader);
glDeleteProgram(vs);
glDeleteProgram(fs);
context_->bind_program(vis_line_shader_);
scm::math::mat4f projection_matrix = scm::math::mat4f(camera_->get_projection_matrix());
scm::math::mat4f view_matrix = camera_->get_view_matrix();
vis_line_shader_->uniform("model_matrix", scm::math::mat4f::identity());
vis_line_shader_->uniform("view_matrix", view_matrix);
vis_line_shader_->uniform("projection_matrix", projection_matrix);
GLint num_uniforms;
glGetProgramiv(vis_line_shader_->program_id(), GL_ACTIVE_UNIFORMS, &num_uniforms);
GLchar uniform_name[256];
GLsizei length;
GLint size;
GLenum type;
for (int i = 0; i < num_uniforms; i++)
{
	glGetActiveUniform(vis_line_shader_->program_id(), i, sizeof(uniform_name), &length, &size, &type, uniform_name);
	std::cout << uniform_name << std::endl;
}
glUseProgram(vis_line_shader_->program_id());
glDrawArrays(GL_LINE, 0, lines->size());
glDisableVertexAttribArray(0);*/

/*
GLuint vboId;
glGenBuffers(1, &vboId);
glBindBuffer(GL_ARRAY_BUFFER, vboId);
glBufferData(GL_ARRAY_BUFFER, sizeof(position), &position[0], GL_STREAM_DRAW);
glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
glEnableVertexAttribArray(0);

std::string vertexShader =
	"#version 420 core\n"
	"layout(location = 0) in vec3 position;\n"
	"uniform mat4 mts;\n"
	"uniform mat4 model;\n"
	"uniform mat4 view;\n"
	"uniform mat4 projection;\n"
	"void main()\n"
	"{\n"
	"   gl_Position = projection * view * vec4(position, 1.0);\n"
	"}\n";
std::cout << "vertexShader" << std::endl;

std::string fragmentShader =
	"#version 420 core\n"
	"out vec4 color;\n"
	"void main()\n"
	"{\n"
	"   color = vec4(0.2, 0.1, 0.3, 0.8);\n"
	"}\n";
std::cout << "fragmentShader" << std::endl;

unsigned int program = glCreateProgram();
unsigned int vs = CompileShader(GL_VERTEX_SHADER, vertexShader, 0);
unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, fragmentShader, 0);
glAttachShader(program, vs);
glAttachShader(program, fs);
glLinkProgram(program);
glValidateProgram(program);
glDeleteProgram(vs);
glDeleteProgram(fs);
glUseProgram(program);

glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE, &vmf[0]);
glUniformMatrix4fv(glGetUniformLocation(program, "projection"), 1, GL_FALSE, &pmf[0]);
glDrawArrays(GL_TRIANGLES, 0, 3);
glDisableVertexAttribArray(0);*/

//void LamurePointCloudPlugin::create_bvh_resources() {
//    //create bvh representation
//    int num_idx_per_node = 72;
//    bvh_line_resource_.num_idx_per_node = num_idx_per_node;
//
//    lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
//    lamure::model_t num_models = database->num_models();
//
//    std::vector<std::string> lod_files;
//    std::vector<std::string> provenance_files;
//
//    for (lamure::model_t model_id = 0; model_id < num_models; ++model_id) {
//
//        std::string bvh_filename = database->get_model(model_id)->get_bvh()->get_filename();
//        std::string base_name = bvh_filename.substr(0, bvh_filename.find_last_of(".") + 1);
//        std::string file_extension = bvh_filename.substr(base_name.size());
//        std::string bvh_suffix = file_extension.substr(3);
//        std::string lod_file_name = base_name + "lod" + bvh_suffix;
//        std::string provenance_file_name = bvh_filename.substr(0, bvh_filename.size() - 3) + "prov";
//
//        lod_files.push_back(lod_file_name);
//        if (settings_.provenance_ != 0)
//        {
//            provenance_files.push_back(provenance_file_name);
//        }
//    }
//
//    char* local_cache = new char[database->get_slot_size()];
//    char* local_cache_provenance = nullptr;
//    if (settings_.provenance_ != 0) {
//        local_cache_provenance = new char[lamure::ren::policy::get_instance()->size_of_provenance()];
//    }
//
//
//    for (int model_id = 0; model_id < num_models_; model_id++) {
//
//        std::vector<std::vector<int>> idx_vec;
//
//        std::vector<std::array<float, 3>> vert_min;
//        std::vector<std::array<float, 3>> vert_max;
//
//        std::string bvh_filename = database->get_model(model_id)->get_bvh()->get_filename();
//        std::string base_name = bvh_filename.substr(0, bvh_filename.find_last_of(".") + 1);
//        std::string file_extension = bvh_filename.substr(base_name.size());
//        std::string bvh_suffix = file_extension.substr(3);
//        std::string lod_file_name = base_name + "lod" + bvh_suffix;
//        std::string provenance_file_name = bvh_filename.substr(0, bvh_filename.size() - 3) + "prov";
//
//        const auto& bvh_ = lamure::ren::model_database::get_instance()->get_model(model_id)->get_bvh();
//        const auto& bounding_boxes = bvh_->get_bounding_boxes();
//        int prim_per_node = bvh_->get_primitives_per_node();
//        int prim_size = bvh_->get_size_of_primitive();
//
//        lamure::ren::lod_stream access;
//        access.open(lod_files[model_id]);
//
//        for (int node_id = 0; node_id < bounding_boxes.size(); node_id++) {
//
//            size_t stride_in_bytes = database->get_node_size(model_id);
//            size_t offset_in_bytes = node_id * stride_in_bytes;
//
//            access.read(local_cache, offset_in_bytes, stride_in_bytes);
//
//            lamure::ren::dataset::serialized_surfel* surfels = (lamure::ren::dataset::serialized_surfel*)local_cache;
//
//            std::array<float, 3> pos_min = std::array<float, 3>{0.0f, 0.0f, 0.0f};
//            std::array<float, 3> pos_max = std::array<float, 3>{0.0f, 0.0f, 0.0f};
//            std::array<int, 3> idx_min = std::array<int, 3>{0, 0, 0};
//            std::array<int, 3> idx_max = std::array<int, 3>{0, 0, 0};
//
//            /*           for (int prim = 0; prim < prim_per_node; prim++) {
//
//                           min_.x = std::min(min_.x, point.x);
//                           min_.y = std::min(min_.y, point.y);
//                           min_.z = std::min(min_.z, point.z);
//
//                           max_.x = std::max(max_.x, point.x);
//                           max_.y = std::max(max_.y, point.y);
//                           max_.z = std::max(max_.z, point.z);
//
//                       }*/
//
//
//            for (int prim = 0; prim < prim_per_node; prim++) {
//                if (prim == 0) {
//                    idx_min[0] = prim;
//                    pos_min[0] = surfels[prim].x;
//                    idx_max[0] = prim;
//                    pos_max[0] = surfels[prim].y;
//                    idx_min[1] = prim;
//                    pos_min[1] = surfels[prim].z;
//                    idx_max[1] = prim;
//                    pos_max[1] = surfels[prim].x;
//                    idx_min[2] = prim;
//                    pos_min[2] = surfels[prim].y;
//                    idx_max[2] = prim;
//                    pos_max[2] = surfels[prim].z;
//                }
//                else {
//                    if (surfels[prim].x < pos_min[0]) {
//                        idx_min[0] = prim;
//                        pos_min[0] = surfels[prim].x;
//                    }
//                    else if (surfels[prim].x > pos_max[0]) {
//                        idx_max[0] = prim;
//                        pos_max[0] = surfels[prim].x;
//                    }
//
//                    if (surfels[prim].x < pos_min[1]) {
//                        idx_min[1] = prim;
//                        pos_min[1] = surfels[prim].y;
//                    }
//                    else if (surfels[prim].x > pos_max[1]) {
//                        idx_max[1] = prim;
//                        pos_max[1] = surfels[prim].y;
//                    }
//
//                    if (surfels[prim].x < pos_min[2]) {
//                        idx_min[2] = prim;
//                        pos_min[2] = surfels[prim].z;
//                    }
//                    else if (surfels[prim].x > pos_max[2]) {
//                        idx_max[2] = prim;
//                        pos_max[2] = surfels[prim].z;
//                    }
//                }
//            }
//
//            std::vector<int> vect =
//            {
//                idx_min[0], idx_min[1], idx_min[2],
//                idx_max[0], idx_min[1], idx_min[2],
//                idx_max[0], idx_min[1], idx_min[2],
//                idx_max[0], idx_min[1], idx_max[2],
//                idx_max[0], idx_min[1], idx_max[2],
//                idx_min[0], idx_min[1], idx_max[2],
//                idx_min[0], idx_min[1], idx_max[2],
//                idx_min[0], idx_min[1], idx_min[2],
//
//                idx_min[0], idx_max[1], idx_min[2],
//                idx_max[0], idx_max[1], idx_min[2],
//                idx_max[0], idx_max[1], idx_min[2],
//                idx_max[0], idx_max[1], idx_max[2],
//                idx_max[0], idx_max[1], idx_max[2],
//                idx_min[0], idx_max[1], idx_max[2],
//                idx_min[0], idx_max[1], idx_max[2],
//                idx_min[0], idx_max[1], idx_min[2],
//
//                idx_min[0], idx_min[1], idx_min[2],
//                idx_min[0], idx_max[1], idx_min[2],
//                idx_min[0], idx_min[1], idx_max[2],
//                idx_min[0], idx_max[1], idx_max[2],
//                idx_max[0], idx_min[1], idx_max[2],
//                idx_max[0], idx_max[1], idx_max[2],
//                idx_max[0], idx_min[1], idx_min[2],
//                idx_max[0], idx_max[1], idx_min[2]
//            };
//
//            idx_vec.push_back(vect);
//            vert_min.push_back(pos_min);
//            vert_max.push_back(pos_max);
//
//
//            std::cout << "node_id: " << node_id << std::endl;
//            std::cout << idx_min[0] << " " << idx_min[1] << " " << idx_min[2] << std::endl;
//            std::cout << pos_min[0] << " " << pos_min[1] << " " << pos_min[2] << " " << std::endl;
//            std::cout << bounding_boxes[node_id].min_vertex() << std::endl;
//            std::cout << idx_max[0] << " " << idx_max[1] << " " << idx_max[2] << std::endl;
//            std::cout << pos_max[0] << " " << pos_max[1] << " " << pos_max[2] << " " << std::endl;
//            std::cout << bounding_boxes[node_id].max_vertex() << std::endl;
//            std::cout << "" << std::endl;
//
//        }
//        access.close();
//
//        bvh_line_resource_.vert_min = vert_min;
//        bvh_line_resource_.vert_max = vert_max;
//        bvh_line_resource_.idx_vec = idx_vec;
//        bvh_line_resource_.num_primitives_ = 24 * bounding_boxes.size();
//        bvh_resources_[model_id] = bvh_line_resource_;
//
//        //GLuint elementbuffer;
//        //glGenBuffers(1, &elementbuffer);
//        //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
//        //glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(idx_serialized), &idx_serialized[0], GL_STREAM_DRAW);
//        //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
//        //bvh_line_resource.gl_id = elementbuffer;
//
//    }
//}


/*
void LamurePointCloudPlugin::create_aux_resources_buffered() {
	if (!settings_.create_aux_resources_) { return; }

	osg::Geometry* aux_geo = new osg::Geometry();
	plugin->geode->addDrawable(aux_geo);

	for (uint32_t model_id = 0; model_id < num_models_; ++model_id) {
		const auto& bounding_boxes = lamure::ren::model_database::get_instance()->get_model(model_id)->get_bvh()->get_bounding_boxes();
		aux_geo->setUseDisplayList(false);
		aux_geo->setUseVertexBufferObjects(true);
		aux_geo->setUseVertexArrayObject(false);
		osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_LINES, bounding_boxes.size() * 24);
		osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(bounding_boxes.size() * 8);

		for (uint64_t node_id = 0; node_id < bounding_boxes.size(); ++node_id) {
			scm::math::vec3f min_vertex = bounding_boxes[node_id].min_vertex();
			scm::math::vec3f max_vertex = bounding_boxes[node_id].max_vertex();
			(*vertices)[(node_id * 8) + 0] = osg::Vec3f(min_vertex.x, min_vertex.y, min_vertex.z);
			(*vertices)[(node_id * 8) + 1] = osg::Vec3f(max_vertex.x, min_vertex.y, min_vertex.z);
			(*vertices)[(node_id * 8) + 2] = osg::Vec3f(max_vertex.x, min_vertex.y, max_vertex.z);
			(*vertices)[(node_id * 8) + 3] = osg::Vec3f(min_vertex.x, min_vertex.y, max_vertex.z);
			(*vertices)[(node_id * 8) + 4] = osg::Vec3f(min_vertex.x, max_vertex.y, min_vertex.z);
			(*vertices)[(node_id * 8) + 5] = osg::Vec3f(max_vertex.x, max_vertex.y, min_vertex.z);
			(*vertices)[(node_id * 8) + 6] = osg::Vec3f(max_vertex.x, max_vertex.y, max_vertex.z);
			(*vertices)[(node_id * 8) + 7] = osg::Vec3f(min_vertex.x, max_vertex.y, max_vertex.z);

			(*indices)[(node_id * 24) + 0] = (node_id * 8) + 0;   (*indices)[(node_id * 24) + 1] = (node_id * 8) + 1;
			(*indices)[(node_id * 24) + 2] = (node_id * 8) + 1;   (*indices)[(node_id * 24) + 3] = (node_id * 8) + 2;
			(*indices)[(node_id * 24) + 4] = (node_id * 8) + 2;   (*indices)[(node_id * 24) + 5] = (node_id * 8) + 3;
			(*indices)[(node_id * 24) + 6] = (node_id * 8) + 3;   (*indices)[(node_id * 24) + 7] = (node_id * 8) + 0;
			(*indices)[(node_id * 24) + 8] = (node_id * 8) + 4;   (*indices)[(node_id * 24) + 9] = (node_id * 8) + 5;
			(*indices)[(node_id * 24) + 10] = (node_id * 8) + 5;  (*indices)[(node_id * 24) + 11] = (node_id * 8) + 6;
			(*indices)[(node_id * 24) + 12] = (node_id * 8) + 6;  (*indices)[(node_id * 24) + 13] = (node_id * 8) + 7;
			(*indices)[(node_id * 24) + 14] = (node_id * 8) + 7;  (*indices)[(node_id * 24) + 15] = (node_id * 8) + 4;
			(*indices)[(node_id * 24) + 16] = (node_id * 8) + 0;  (*indices)[(node_id * 24) + 17] = (node_id * 8) + 4;
			(*indices)[(node_id * 24) + 18] = (node_id * 8) + 1;  (*indices)[(node_id * 24) + 19] = (node_id * 8) + 5;
			(*indices)[(node_id * 24) + 20] = (node_id * 8) + 2;  (*indices)[(node_id * 24) + 21] = (node_id * 8) + 6;
			(*indices)[(node_id * 24) + 22] = (node_id * 8) + 3;  (*indices)[(node_id * 24) + 23] = (node_id * 8) + 7;
		}
		osg::StateSet* stateset = new osg::StateSet();
		aux_geo->setStateSet(stateset);
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
		colors->push_back(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
		aux_geo->setVertexArray(vertices.get());
		aux_geo->setColorArray(colors.get());
		aux_geo->setColorBinding(osg::Geometry::BIND_OVERALL);
		stateset->setAttributeAndModes(new osg::LineWidth(1.0f), osg::StateAttribute::ON);
		aux_geo->addPrimitiveSet(indices.get());
	}



	for (int model_id = 0; model_id < num_models_; ++model_id) {
		if (settings_.show_bvhs_) {
			//create bvh representation
			const auto& bounding_boxes = lamure::ren::model_database::get_instance()->get_model(model_id)->get_bvh()->get_bounding_boxes();

			aux_geo->setUseDisplayList(false);
			aux_geo->setUseVertexBufferObjects(true);
			aux_geo->setUseVertexArrayObject(false);

			osg::ref_ptr<osg::Vec3Array> lines = new osg::Vec3Array();
			for (uint64_t node_id = 0; node_id < bounding_boxes.size(); ++node_id) {
				const auto& node = bounding_boxes[node_id];
				lines_from_min_max_buffered(node.min_vertex(), node.max_vertex(), lines);
			}
			aux_geo->setVertexArray(lines.get());

			osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
			colors->push_back(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
			aux_geo->setColorArray(colors.get());
			aux_geo->setColorBinding(osg::Geometry::BIND_OVERALL);
			aux_geo->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(1.0f), osg::StateAttribute::ON);
			aux_geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, (int)(24 * bounding_boxes.size()), 0));
		}
		if (settings_.show_pvs_ && settings_.pvs_ != "") {
			std::string pvs_grid_file_path = settings_.pvs_;
			pvs_grid_file_path.resize(pvs_grid_file_path.length() - 3);
			pvs_grid_file_path = pvs_grid_file_path + "grid";

			lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();
			pvs->load_pvs_from_file(pvs_grid_file_path, settings_.pvs_, false);
			pvs->activate(settings_.use_pvs_);
			std::cout << "use pvs: " << (int)pvs->is_activated() << std::endl;
			if (pvs->get_visibility_grid() != nullptr) {

				aux_geo->setUseDisplayList(false);
				aux_geo->setUseVertexBufferObjects(true);
				aux_geo->setUseVertexArrayObject(false);

				osg::ref_ptr<osg::Vec3Array> lines = new osg::Vec3Array();

				for (size_t cell_id = 0; cell_id < pvs->get_visibility_grid()->get_cell_count(); ++cell_id) {
					const lamure::pvs::view_cell* cell = pvs->get_visibility_grid()->get_cell_at_index(cell_id);

					scm::math::vec3f min_vertex(cell->get_position_center() - (cell->get_size() * 0.5f));
					scm::math::vec3f max_vertex(cell->get_position_center() + (cell->get_size() * 0.5f));

					lines_from_min_max_buffered(min_vertex, max_vertex, lines);
				}

				aux_geo->setVertexArray(lines.get());

				osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
				colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
				aux_geo->setColorArray(colors.get());
				aux_geo->setColorBinding(osg::Geometry::BIND_OVERALL);
				aux_geo->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(1.0f), osg::StateAttribute::ON);
				aux_geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, (int)(pvs->get_visibility_grid()->get_cell_count() * 24)));

			}
			else {
				std::cout << "no pvs grid!" << std::endl;
			}
		}
	}


}*/
//for (uint32_t model_id = 0; model_id < num_models_; ++model_id) {
//    const auto& bounding_boxes = lamure::ren::model_database::get_instance()->get_model(model_id)->get_bvh()->get_bounding_boxes();
//    float* corners_ = LamurePointCloudPlugin::instance()->getSerializesBvhCorners(bounding_boxes);
//}
//coVRPluginList::instance()->preDraw(renderInfo);

//create bvh representation
//for (uint32_t model_id = 0; model_id < num_models_; ++model_id) {
//    const auto& bounding_boxes = lamure::ren::model_database::get_instance()->get_model(model_id)->get_bvh()->get_bounding_boxes();
//    osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_LINES, bounding_boxes.size() * 24);
//    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(bounding_boxes.size() * 8);
//    for (uint64_t node_id = 0; node_id < bounding_boxes.size(); ++node_id) {
//        scm::math::vec3f min_vertex = bounding_boxes[node_id].min_vertex();
//        scm::math::vec3f max_vertex = bounding_boxes[node_id].max_vertex();
//        (*vertices)[(node_id * 8) + 0] = osg::Vec3f(min_vertex.x, min_vertex.y, min_vertex.z);
//        (*vertices)[(node_id * 8) + 1] = osg::Vec3f(max_vertex.x, min_vertex.y, min_vertex.z);
//        (*vertices)[(node_id * 8) + 2] = osg::Vec3f(max_vertex.x, min_vertex.y, max_vertex.z);
//        (*vertices)[(node_id * 8) + 3] = osg::Vec3f(min_vertex.x, min_vertex.y, max_vertex.z);
//        (*vertices)[(node_id * 8) + 4] = osg::Vec3f(min_vertex.x, max_vertex.y, min_vertex.z);
//        (*vertices)[(node_id * 8) + 5] = osg::Vec3f(max_vertex.x, max_vertex.y, min_vertex.z);
//        (*vertices)[(node_id * 8) + 6] = osg::Vec3f(max_vertex.x, max_vertex.y, max_vertex.z);
//        (*vertices)[(node_id * 8) + 7] = osg::Vec3f(min_vertex.x, max_vertex.y, max_vertex.z);
//        (*indices)[(node_id * 12) + 0] = (node_id * 8) + 0;   (*indices)[(node_id * 12) + 1] = (node_id * 8) + 1;
//        (*indices)[(node_id * 12) + 2] = (node_id * 8) + 1;   (*indices)[(node_id * 12) + 3] = (node_id * 8) + 2;
//        (*indices)[(node_id * 12) + 4] = (node_id * 8) + 2;   (*indices)[(node_id * 12) + 5] = (node_id * 8) + 3;
//        (*indices)[(node_id * 12) + 6] = (node_id * 8) + 3;   (*indices)[(node_id * 12) + 7] = (node_id * 8) + 0;
//        (*indices)[(node_id * 12) + 8] = (node_id * 8) + 4;   (*indices)[(node_id * 12) + 9] = (node_id * 8) + 5;
//        (*indices)[(node_id * 12) + 10] = (node_id * 8) + 5;  (*indices)[(node_id * 12) + 11] = (node_id * 8) + 6;
//        (*indices)[(node_id * 12) + 12] = (node_id * 8) + 6;  (*indices)[(node_id * 12) + 13] = (node_id * 8) + 7;
//        (*indices)[(node_id * 12) + 14] = (node_id * 8) + 7;  (*indices)[(node_id * 12) + 15] = (node_id * 8) + 4;
//        (*indices)[(node_id * 12) + 16] = (node_id * 8) + 0;  (*indices)[(node_id * 12) + 17] = (node_id * 8) + 4;
//        (*indices)[(node_id * 12) + 18] = (node_id * 8) + 1;  (*indices)[(node_id * 12) + 19] = (node_id * 8) + 5;
//        (*indices)[(node_id * 12) + 20] = (node_id * 8) + 2;  (*indices)[(node_id * 12) + 21] = (node_id * 8) + 6;
//        (*indices)[(node_id * 12) + 22] = (node_id * 8) + 3;  (*indices)[(node_id * 12) + 23] = (node_id * 8) + 7;
//    }
//    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
//    colors->push_back(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
//    setVertexArray(vertices.get());
//    setColorArray(colors.get());
//    setColorBinding(osg::Geometry::BIND_OVERALL);
//    getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(1.0f), osg::StateAttribute::ON);
//    addPrimitiveSet(indices.get());
//}

//unsigned int program = glCreateProgram();
//unsigned int vs = CompileShader(GL_VERTEX_SHADER, vis_line_vs_source, 0);
//unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, vis_line_fs_source, 0);
//glAttachShader(program, vs);
//glAttachShader(program, fs);
//glLinkProgram(program);
//glValidateProgram(program);
//glDeleteProgram(vs);
//glDeleteProgram(fs);
//glUseProgram(program);

//GLuint ibo = bvh_res.gl_id;
//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
//context_->bind_program(vis_line_shader_);
//vis_line_shader_->uniform("mvp_matrix", scm::math::mat4f(model_view_projection_matrix));
//context_->apply_program();
//scm::gl::buffer_ptr index_buffer = device_->create_buffer(scm::gl::BIND_INDEX_BUFFER, scm::gl::USAGE_STREAM_DRAW, sizeof(bvh_res.idx_serialized), bvh_res.idx_serialized);
//context_->bind_index_buffer(index_buffer, scm::gl::PRIMITIVE_LINE_LIST, scm::gl::TYPE_INT, 0);
//context_->apply();

//context_->draw_elements(1, bvh_res.idx_serialized[node_slot_aggregate.node_id_ * bvh_res.idx_per_node], 12);
//glDrawElements(GL_LINES, 24, GL_STREAM_DRAW, bvh_res.idx_arr[node_slot_aggregate.node_id_]);
//glDrawElements(GL_LINES, bvh_res.idx_per_node, GL_STREAM_DRAW, bvh_res.idx_arr[node_slot_aggregate.node_id_]);

//GLuint ibo;
//glGenBuffers(1, &ibo);
//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
//glBufferData(GL_ELEMENT_ARRAY_BUFFER, bvh_res.idx_per_node * 3 * sizeof(int), bvh_res.idx_serialized, GL_STREAM_DRAW);
//float* mvp = scm::math::mat4f(model_view_projection_matrix).data_array;
//unsigned int program = glCreateProgram();
//unsigned int vs = CompileShader(GL_VERTEX_SHADER, vis_line_vs_source, 0);
//unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, vis_line_fs_source, 0);
//glAttachShader(program, vs);
//glAttachShader(program, fs);
//glLinkProgram(program);
//glValidateProgram(program);
//glDeleteProgram(vs);
//glDeleteProgram(fs);
//glUseProgram(program);
//glEnableVertexAttribArray(0);


// bind program, set state and set uniforms
//context_->bind_program(vis_line_shader_);
//_plugin->set_uniforms(vis_line_shader_);
//lamure::context_t context_id = controller->deduce_context_id(lmr_ctx);
//scm::math::mat4d projection_matrix = scm::math::mat4d(gl_projection_matrix_d);
//scm::math::mat4d view_matrix = gl_view_matrix_d;
//for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
//    bool draw = true;
//    auto bvh_res = bvh_resources_[model_id];
//    lamure::model_t m_id = controller->deduce_model_id(std::to_string(model_id));
//    lamure::ren::cut& cut = cuts->get_cut(lmr_ctx, lmr_ctx, m_id);
//    std::vector<lamure::ren::cut::node_slot_aggregate> renderable = cut.complete_set();
//    const lamure::ren::bvh* bvh = database->get_model(m_id)->get_bvh();
//    if (draw) {
//        //uniforms per model
//        scm::math::mat4d model_matrix = model_info_.model_transformations_[model_id];
//        scm::math::mat4d mvp = projection_matrix * view_matrix * model_matrix;
//        vis_line_shader_->uniform("mvp_matrix", scm::math::mat4f(mvp));
//        std::vector<scm::gl::boxf>const& bounding_box_vector = bvh->get_bounding_boxes();
//        scm::gl::frustum frustum_by_model = scm_camera_->get_frustum_by_model(scm::math::mat4f(model_matrix));
//        if (bvh_res.num_primitives_ > 0) {
//            context_->bind_vertex_array(bvh_res.array_);
//            context_->apply();
//            for (auto const& node_slot_aggregate : renderable) {
//                uint32_t node_culling_result = scm_camera_->cull_against_frustum(frustum_by_model, bounding_box_vector[node_slot_aggregate.node_id_]);
//                if (node_culling_result != 1) {
//                    context_->draw_arrays(scm::gl::PRIMITIVE_LINE_LIST, node_slot_aggregate.node_id_ * 24, 24);
//                }
//            }
//        }
//    }
//}