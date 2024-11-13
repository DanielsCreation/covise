
#define GLFW_EXPOSE_NATIVE_WIN32
//local
#include "LamurePointCloud.h"
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


lmr_camera* lamure_camera_;
lamure::ren::camera* scm_camera_;
osg::ref_ptr<osg::Camera>   osg_camera_;
osg::ref_ptr<osg::Camera>   rtt_camera_;

scm::gl::program_ptr vis_surfel_shader_;
scm::gl::program_ptr vis_line_bb_shader_;
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


struct pcl_resource {
	GLuint vbo_{ 0 };
	GLuint ibo_{ 0 };
	GLuint vao_{ 0 };
	GLuint program_{ 0 };
	std::array<unsigned short, 24> corners_idx_ = {
		0, 1, 2, 3, 4, 5, 6, 7,
		0, 2, 1, 3, 4, 6, 5, 7,
		0, 4, 1, 5, 2, 6, 3, 7,
	};
};
pcl_resource pcl_resource_;


struct box_resource {
	GLuint vbo_{ 0 };
	GLuint ibo_{ 0 };
	GLuint vao_{ 0 };
	GLuint program_{ 0 };
	std::array<unsigned short, 24> corners_idx_ = {
		0, 1, 2, 3, 4, 5, 6, 7,
		0, 2, 1, 3, 4, 6, 5, 7,
		0, 4, 1, 5, 2, 6, 3, 7,
	};
};
box_resource box_resource_;


struct plane_resource {
	GLuint vbo_{ 0 };
	GLuint ibo_{ 0 };
	GLuint vao_{ 0 };
	GLuint program_{ 0 };
	std::array<unsigned short, 6> corners_idx_ = {
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


struct resource {
	uint64_t num_primitives_{ 0 };
	scm::gl::buffer_ptr buffer_;
	scm::gl::vertex_array_ptr array_;
	std::vector<std::vector<float>> corners_;
};

resource brush_resource_;
resource pvs_resource_;
resource frustum_resource_;

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


double fps_ = 0.0;
uint64_t rendered_splats_ = 0;
uint64_t rendered_nodes_ = 0;


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

#define BACKUP_GL_STATE() \
GLenum last_active_texture; glGetIntegerv(GL_ACTIVE_TEXTURE, (GLint*)&last_active_texture); \
GLint last_program; glGetIntegerv(GL_CURRENT_PROGRAM, &last_program); \
GLint last_texture; glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture); \
GLint last_sampler; glGetIntegerv(GL_SAMPLER_BINDING, &last_sampler); \
GLint last_array_buffer; glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &last_array_buffer); \
GLint last_element_array_buffer; glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, &last_element_array_buffer); \
GLint last_vertex_array; glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &last_vertex_array); \
GLint last_polygon_mode[2]; glGetIntegerv(GL_POLYGON_MODE, last_polygon_mode); \
GLint last_viewport[4]; glGetIntegerv(GL_VIEWPORT, last_viewport); \
GLint last_scissor_box[4]; glGetIntegerv(GL_SCISSOR_BOX, last_scissor_box); \
GLenum last_blend_src_rgb; glGetIntegerv(GL_BLEND_SRC_RGB, (GLint*)&last_blend_src_rgb); \
GLenum last_blend_dst_rgb; glGetIntegerv(GL_BLEND_DST_RGB, (GLint*)&last_blend_dst_rgb); \
GLenum last_blend_src_alpha; glGetIntegerv(GL_BLEND_SRC_ALPHA, (GLint*)&last_blend_src_alpha); \
GLenum last_blend_dst_alpha; glGetIntegerv(GL_BLEND_DST_ALPHA, (GLint*)&last_blend_dst_alpha); \
GLenum last_blend_equation_rgb; glGetIntegerv(GL_BLEND_EQUATION_RGB, (GLint*)&last_blend_equation_rgb); \
GLenum last_blend_equation_alpha; glGetIntegerv(GL_BLEND_EQUATION_ALPHA, (GLint*)&last_blend_equation_alpha); \
GLboolean last_enable_blend = glIsEnabled(GL_BLEND); \
GLboolean last_enable_cull_face = glIsEnabled(GL_CULL_FACE); \
GLboolean last_enable_depth_test = glIsEnabled(GL_DEPTH_TEST); \
GLboolean last_enable_scissor_test = glIsEnabled(GL_SCISSOR_TEST); \
GLint last_cull_face_mode; glGetIntegerv(GL_CULL_FACE_MODE, &last_cull_face_mode); \
GLint last_front_face; glGetIntegerv(GL_FRONT_FACE, &last_front_face); \
GLint last_depth_func; glGetIntegerv(GL_DEPTH_FUNC, &last_depth_func); \
GLboolean last_depth_mask; glGetBooleanv(GL_DEPTH_WRITEMASK, &last_depth_mask); \
GLboolean last_color_writemask[4]; glGetBooleanv(GL_COLOR_WRITEMASK, last_color_writemask); \
GLboolean last_stencil_test = glIsEnabled(GL_STENCIL_TEST); \
GLint last_stencil_func; glGetIntegerv(GL_STENCIL_FUNC, &last_stencil_func); \
GLint last_stencil_ref; glGetIntegerv(GL_STENCIL_REF, &last_stencil_ref); \
GLint last_stencil_value_mask; glGetIntegerv(GL_STENCIL_VALUE_MASK, &last_stencil_value_mask); \
GLint last_stencil_fail; glGetIntegerv(GL_STENCIL_FAIL, &last_stencil_fail); \
GLint last_stencil_pass_depth_fail; glGetIntegerv(GL_STENCIL_PASS_DEPTH_FAIL, &last_stencil_pass_depth_fail); \
GLint last_stencil_pass_depth_pass; glGetIntegerv(GL_STENCIL_PASS_DEPTH_PASS, &last_stencil_pass_depth_pass); \

#define RESTORE_GL_STATE() \
glUseProgram(last_program); \
glBindTexture(GL_TEXTURE_2D, last_texture); \
glBindSampler(0, last_sampler); \
glActiveTexture(last_active_texture); \
glBindVertexArray(last_vertex_array); \
glBindBuffer(GL_ARRAY_BUFFER, last_array_buffer); \
glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, last_element_array_buffer); \
glBlendEquationSeparate(last_blend_equation_rgb, last_blend_equation_alpha); \
glBlendFuncSeparate(last_blend_src_rgb, last_blend_dst_rgb, last_blend_src_alpha, last_blend_dst_alpha); \
if (last_enable_blend) glEnable(GL_BLEND); else glDisable(GL_BLEND); \
if (last_enable_cull_face) glEnable(GL_CULL_FACE); else glDisable(GL_CULL_FACE); \
if (last_enable_depth_test) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST); \
if (last_enable_scissor_test) glEnable(GL_SCISSOR_TEST); else glDisable(GL_SCISSOR_TEST); \
glPolygonMode(GL_FRONT_AND_BACK, last_polygon_mode[0]); \
glViewport(last_viewport[0], last_viewport[1], (GLsizei)last_viewport[2], (GLsizei)last_viewport[3]); \
glScissor(last_scissor_box[0], last_scissor_box[1], (GLsizei)last_scissor_box[2], (GLsizei)last_scissor_box[3]); \
glColorMask(last_color_writemask[0], last_color_writemask[1], last_color_writemask[2], last_color_writemask[3]); \
glCullFace(last_cull_face_mode); \
glFrontFace(last_front_face); \
glDepthFunc(last_depth_func); \
glDepthMask(last_depth_mask); \
glColorMask(last_color_writemask[0], last_color_writemask[1], last_color_writemask[2], last_color_writemask[3]); \
if (last_stencil_test) glEnable(GL_STENCIL_TEST); else glDisable(GL_STENCIL_TEST); \
glStencilFunc(last_stencil_func, last_stencil_ref, last_stencil_value_mask); \
glStencilOp(last_stencil_fail, last_stencil_pass_depth_fail, last_stencil_pass_depth_pass); \


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
    scm::gl::frustum frustum_ = scm_camera_->get_frustum_by_model(scm::math::mat4f(model_matrix)); \
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


void LamurePointCloudPlugin::printNodePath(osg::ref_ptr<osg::Node> pointer) {
	osg::NodePathList npl = pointer->getParentalNodePaths();
	int path_size = npl.size();
	std::cout << pointer->className() << " at level " << path_size << std::endl;
	if (path_size > 0) {
		std::cout << "plugin->gl_grp->getParentalNodePaths()" << std::endl << "Size NodePathList: " << path_size << std::endl;
		for (int j = 0; j < npl[0].size(); j++) {
			std::cout << "[" << j << "] " << npl[0][j]->className() << ":  " << npl[0][j]->getName() << std::endl;
		}
		std::cout << "" << std::endl;
	}
	std::cout << "" << std::endl;
}


void wait_for_opengl_context() {
	// Warten auf den OpenGL-Kontext, maximal max_wait_time_ms Millisekunden
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

//#ifdef DEBUG
		std::cout << "wglGetCurrentContext(): " << wglGetCurrentContext() << std::endl;
		std::cout << "wglGetCurrentDC(): " << wglGetCurrentDC() << std::endl;
		std::cout << "WindowFromDC(wglGetCurrentDC()): " << WindowFromDC(wglGetCurrentDC()) << std::endl;
		std::cout << "FindWindow(NULL, 'OpenCOVER'): " << FindWindow(NULL, "OpenCOVER") << std::endl;
		std::cout << "FindWindow(NULL, 'COVER'): " << FindWindow(NULL, "COVER") << std::endl;
		std::cout << "GetDC(FindWindow(NULL, 'OpenCOVER')): " << GetDC(FindWindow(NULL, "OpenCOVER")) << std::endl;
		std::cout << "GetDC(FindWindow(NULL, 'COVER')): " << GetDC(FindWindow(NULL, "COVER")) << std::endl;
//#endif
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
		// Negative Dezimalstellen sind nicht erlaubt
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

	std::cout << "wglGetCurrentContext(): " << wglGetCurrentContext() << std::endl;
	std::cout << "FindWindow(NULL, 'OpenCOVER'): " << FindWindow(NULL, "OpenCOVER") << std::endl;
	std::cout << "FindWindow(NULL, 'COVER'): " << FindWindow(NULL, "COVER") << std::endl;
	std::cout << "GetDC(plugin->hwnd_opencover): " << GetDC(FindWindow(NULL, "OpenCOVER")) << std::endl;
	std::cout << "GetDC(plugin->hwnd_cover): " << GetDC(FindWindow(NULL, "COVER")) << std::endl;
	std::cout << "wglGetCurrentDC(): " << wglGetCurrentDC() << std::endl;

	for (const auto& input_file : settings_.models_) {
		lamure::model_t model_id = database->add_model(input_file, std::to_string(num_models_));
		model_info_.model_transformations_.push_back(settings_.transforms_[num_models_] * scm::math::mat4d(scm::math::make_translation(database->get_model(num_models_)->get_bvh()->get_translation())));
		++num_models_;
	}
	return 1;
}


std::mutex context_mutex;
std::condition_variable context_initialized_cv;
bool is_context_initialized = false;


void initialize_opengl_context() {
	std::lock_guard<std::mutex> lock(context_mutex);
	// Hier wird der OpenGL-Kontext initialisiert...
	is_context_initialized = true;  // Der Kontext ist jetzt bereit.
	context_initialized_cv.notify_all();  // Benachrichtige alle wartenden Threads.
}


void render_scene() {
	std::unique_lock<std::mutex> lock(context_mutex);
	context_initialized_cv.wait(lock, [] { return is_context_initialized; });

	if (context_ == nullptr) {
		std::cerr << "Fehler: OpenGL-Kontext noch nicht initialisiert!" << std::endl;
		return;
	}
}


void LamurePointCloudPlugin::updateModelRotation() {

	scm::math::mat4d rotX = scm::math::make_rotation(rotationAngles.x, scm::math::vec3d(1.0, 0.0, 0.0));
	scm::math::mat4d rotY = scm::math::make_rotation(rotationAngles.y, scm::math::vec3d(0.0, 1.0, 0.0));
	scm::math::mat4d rotZ = scm::math::make_rotation(rotationAngles.z, scm::math::vec3d(0.0, 0.0, 1.0));

	settings_.model_rot_ = rotZ * rotY * rotX;
}


bool LamurePointCloudPlugin::init2() {
	std::cout << "init()" << std::endl;
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

	plugin->LamureGroup = new osg::Group();
	plugin->LamureGroup->setName("LamureGroup");
	cover->getObjectsRoot()->addChild(plugin->LamureGroup);

	plugin->transform = new osg::MatrixTransform();
	plugin->LamureGroup->addChild(plugin->file);
	plugin->LamureGroup->addChild(plugin->transform);

	plugin->geode = new osg::Geode();
	plugin->geode->setName("LamureGeode");
	plugin->transform->addChild(plugin->geode);

	wait_for_opengl_context();

	plugin->hdc_current = wglGetCurrentDC();
	plugin->hdc_cover = GetDC(plugin->hwnd_cover);
	plugin->hdc_opencover = GetDC(plugin->hwnd_opencover);

	plugin->hwnd_current = WindowFromDC(plugin->hdc_current);
	plugin->hwnd_cover = FindWindow(NULL, "COVER");
	plugin->hwnd_opencover = FindWindow(NULL, "OpenCOVER");

	plugin->HGLRC_current = wglGetCurrentContext();

	std::cout << "wglGetCurrentDC(): " << wglGetCurrentDC() << std::endl;
	std::cout << "GetDC(plugin->hwnd_opencover): " << GetDC(FindWindow(NULL, "OpenCOVER")) << std::endl;
	std::cout << "GetDC(plugin->hwnd_cover): " << GetDC(FindWindow(NULL, "COVER")) << std::endl;

	std::cout << "WindowFromDC(wglGetCurrentDC()): " << WindowFromDC(wglGetCurrentDC()) << std::endl;
	std::cout << "FindWindow(NULL, 'COVER'): " << FindWindow(NULL, "COVER") << std::endl;
	std::cout << "FindWindow(NULL, 'OpenCOVER'): " << FindWindow(NULL, "OpenCOVER") << std::endl;

	std::cout << "wglGetCurrentContext(): " << wglGetCurrentContext() << std::endl;


	//create schism objects, backup and restore gl state -> keeps osg state in place
	// Backup GL state
	BACKUP_GL_STATE();

	device_.reset(new scm::gl::render_device());
	if (!device_) { std::cout << "error creating device" << std::endl; }
	context_ = device_->main_context();
	if (!context_) { std::cout << "error creating context" << std::endl; }

	//coVRConfig::instance()->windows[0].context->getState()->print(std::cout);
	std::ostringstream oss;
	device_->dump_memory_info(oss);
	std::cout << oss.str() << std::endl;
	//std::cout << (*device_);
	scm::gl::render_device::device_capabilities capa = device_->capabilities();

	plugin->create_framebuffers();
	plugin->init_lamure_shader();
	plugin->init_camera();
	plugin->create_aux_resources();
	//plugin->create_pcl_resources();
	plugin->init_render_states();

	/*Hier noch nicht die display-func aufrufen! Nur lamure-obj. erstellen und Initialisierung in OpenCOVER abschließen lassen.*/
	// Restore modified GL state
	RESTORE_GL_STATE();

	// Slider für Kamera X-Position
	cameraPosXSlider = new ui::Slider(menu, "camera_translation_x");
	cameraPosXSlider->setText("Camera Translation X");
	cameraPosXSlider->setBounds(-1000.0, 1000.0);
	cameraPosXSlider->setValue(scm_camera_->get_cam_pos()[0]);
	cameraPosXSlider->setShared(true);
	cameraPosXSlider->setCallback([this](double value, bool released) {
		scm_camera_->set_cam_pos(scm::math::vec3d(roundToDecimal(value,3), scm_camera_->get_cam_pos()[1], scm_camera_->get_cam_pos()[2]));
		//scm_camera_->update_frustum();
	});
	
	// Slider für Kamera Y-Position
	cameraPosYSlider = new ui::Slider(menu, "camera_translation_y");
	cameraPosYSlider->setText("Camera Translation Y");
	cameraPosYSlider->setBounds(-1000.0, 1000.0);
	cameraPosYSlider->setValue(scm_camera_->get_cam_pos()[1]);
	cameraPosYSlider->setShared(true);
	cameraPosYSlider->setCallback([this](double value, bool released) {
		scm_camera_->set_cam_pos(scm::math::vec3d(scm_camera_->get_cam_pos()[0], roundToDecimal(value,3), scm_camera_->get_cam_pos()[2]));
		//scm_camera_->update_frustum();
	});

	// Slider für Kamera Z-Position
	cameraPosZSlider = new ui::Slider(menu, "camera_translation_z");
	cameraPosZSlider->setText("Camera Translation Z");
	cameraPosZSlider->setBounds(-1000.0, 1000.0);
	cameraPosZSlider->setValue(scm_camera_->get_cam_pos()[2]);
	cameraPosZSlider->setShared(true);
	cameraPosZSlider->setCallback([this](double value, bool released) {
		scm_camera_->set_cam_pos(scm::math::vec3d(scm_camera_->get_cam_pos()[0], scm_camera_->get_cam_pos()[1], roundToDecimal(value,3)));
		//scm_camera_->update_frustum();
	});

	// Slider für Modell X-Position
	modelPosXSlider = new ui::Slider(menu, "model_translation_x");
	modelPosXSlider->setText("Model Translation X");
	modelPosXSlider->setBounds(-1000.0, 1000.0);
	modelPosXSlider->setValue(settings_.model_tl_.x);
	modelPosXSlider->setShared(true);
	modelPosXSlider->setCallback([this](double value, bool released) { settings_.model_tl_.x = value; });

	// Slider für Modell Y-Position
	modelPosYSlider = new ui::Slider(menu, "model_translation_y");
	modelPosYSlider->setText("Model Translation Y");
	modelPosYSlider->setBounds(-1000.0, 1000.0);
	modelPosYSlider->setValue(settings_.model_tl_.y);
	modelPosYSlider->setShared(true);
	modelPosYSlider->setCallback([this](double value, bool released) { settings_.model_tl_.y = value; });

	// Slider für Modell Z-Position
	modelPosZSlider = new ui::Slider(menu, "model_translation_z");
	modelPosZSlider->setText("Model Translation Z");
	modelPosZSlider->setBounds(-1000.0, 1000.0);
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

	bounding_box_show_button = new ui::Button(group, "bounding boxes");
	frustum_show_button = new ui::Button(group, "frustum");
	coord_show_button = new ui::Button(group, "coordinates");
	sync_cam_button = new ui::Button(group, "sync cam");
	notify_button = new ui::Button(group, "notify toggle");

	bounding_box_show_button->setShared(true);
	frustum_show_button->setShared(true);
	coord_show_button->setShared(true);
	sync_cam_button->setShared(true);
	notify_button->setShared(true);

	//shader_list = new ui::SelectionList(model_grp, "shader_list");
	//shader_list->setText("");
	//shader_list->setShared(true);
	//shader_list->setCallback([this](int idx) {
	//    std::cout << "shader_list, callback" << std::endl;
	//    });
	//for (int m_id = 0; m_id < num_models_; m_id++) {
	//    shader_list->append(settings_.models_[m_id]);
	//}



	//coVRConfig::instance()->windows[0].context->getState()->print(std::cout);
	//VRViewer::instance()->statsDisplay->showStats(coVRStatsDisplay::VIEWER_SCENE_STATS, VRViewer::instance());
	//VRSceneGraph::instance()->viewAll();
	//plugin->setUpStateSets();

	return 1;
}


using namespace scm::gl;
struct GLGrp : public osg::Group
{
	GLGrp()
		:
		_triangle_geode(new osg::Geode),
		_line_geode(new osg::Geode),
		_point_geode(new osg::Geode)
	{
		addChild(_triangle_geode.get());
		addChild(_line_geode.get());
		addChild(_point_geode.get());
		//_geo_switch->setAllChildrenOff();
	}

	void addTriangles(osg::Stats* viewerStats, LamurePointCloudPlugin* plugin) {
		_triangle_geode->addDrawable(new TriangleDrawable(this, viewerStats, plugin));
	}

	void removeTriangles(osg::Stats* viewerStats) {
		uint8_t num_drawables = _triangle_geode->getNumDrawables();
		_triangle_geode->removeDrawables(0, _triangle_geode->getNumDrawables());
	}

	void addLines(osg::Stats* viewerStats, LamurePointCloudPlugin* plugin) {
		_line_geode->addDrawable(new LinesDrawable(this, viewerStats, plugin));
	}

	void addPoints(osg::Stats* viewerStats, LamurePointCloudPlugin* plugin) {
		_point_geode->addDrawable(new PointsDrawable(this, viewerStats, plugin));
	}

	osg::ref_ptr<osg::Geode> _triangle_geode;
	osg::ref_ptr<osg::Geode> _line_geode;
	osg::ref_ptr<osg::Geode> _point_geode;


protected:

	struct TriangleDrawable : public osg::Drawable
	{
		TriangleDrawable(struct GLGrp* gl_grp, osg::Stats* viewerStats, LamurePointCloudPlugin* plugin)
		{
			std::cout << "protected struct TriangleDrawable()" << std::endl;

			setStateSet(plugin->_triangleStateSet.get());
			setDrawCallback(new TriangleUpdateCallback(gl_grp, viewerStats, plugin));
		}
	};
	struct TriangleUpdateCallback : public virtual osg::Drawable::DrawCallback
	{
		TriangleUpdateCallback(GLGrp* gl_grp, osg::Stats* viewerStats, LamurePointCloudPlugin* plugin)
			: _gl_grp(gl_grp),
			_viewerStats(viewerStats),
			_plugin(plugin)
		{
			std::cout << "protected struct TriangleUpdateCallback()" << std::endl;
		}

		/** do customized draw code.*/
		virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const {

			// Backup GL state
			BACKUP_GL_STATE();

			const osg::GLExtensions* api = osg::GLExtensions::Get(renderInfo.getContextID(), true);
			const osg::StateSet* stateset = drawable->getStateSet();

			GLfloat pm[16];
			glGetFloatv(GL_PROJECTION_MATRIX, pm);
			GLfloat vm[16];
			glGetFloatv(GL_MODELVIEW_MATRIX, vm);

			float position[27] = {
					0.0f, 0.0f, 0.0f,
					300.0f, 10.0f, -10.0f,
					300.0f, -10.0f, 10.0f,

					0.0f, 0.0f, 0.0f,
					-10.0f, 300.0f, 10.0f,
					10.0f, 300.0f, -10.0f,

					0.0f, 0.0f, 0.0f,
					-10.0f, 10.0f, 300.0f,
					10.0f, -10.0f, 300.0f,
			};

			GLuint vbo;
			glGenBuffers(1, &vbo);
			glBindBuffer(GL_ARRAY_BUFFER, vbo);
			glBufferData(GL_ARRAY_BUFFER, sizeof(position), &position[0], GL_STREAM_DRAW);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
			glEnableVertexAttribArray(0);


			std::string vertexShader =
				"#version 330 core\n"
				"layout(location = 0) in vec3 position;\n"
				"uniform mat4 mts;\n"
				"uniform mat4 model;\n"
				"uniform mat4 view;\n"
				"uniform mat4 projection;\n"
				"void main()\n"
				"{\n"
				"   gl_Position = projection * view * vec4(position, 1.0);\n"
				"}\n";

			std::string fragmentShader =
				"#version 330 core\n"
				"out vec4 color;\n"
				"void main()\n"
				"{\n"
				"   color = vec4(0.2, 0.1, 0.3, 0.8);\n"
				"}\n";

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

			glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE, &vm[0]);
			glUniformMatrix4fv(glGetUniformLocation(program, "projection"), 1, GL_FALSE, &pm[0]);
			glDrawArrays(GL_TRIANGLES, 0, 9);
			glDisableVertexAttribArray(0);

			RESTORE_GL_STATE();

			drawable->drawImplementation(renderInfo);
		}

		GLGrp* _gl_grp = nullptr;
		osg::Stats* _viewerStats;
		LamurePointCloudPlugin* _plugin;
	};

	struct LinesDrawable : public osg::Drawable
	{
		LinesDrawable(struct GLGrp* gl_grp, osg::Stats* viewerStats, LamurePointCloudPlugin* plugin)
		{
			LamurePointCloudPlugin* _plugin = plugin;
			std::cout << "LinesDrawable()" << std::endl;
			setUseDisplayList(false);
			setUseVertexBufferObjects(true);
			setUseVertexArrayObject(false);
			//setStateSet(plugin->_lineStateSet.get());
			setDrawCallback(new LinesUpdateCallback(gl_grp, viewerStats, _plugin));
		}
	};
	struct LinesUpdateCallback : public virtual osg::Drawable::DrawCallback
	{
		LinesUpdateCallback(GLGrp* gl_grp, osg::Stats* viewerStats, LamurePointCloudPlugin* plugin)
			: _gl_grp(gl_grp),
			_viewerStats(viewerStats),
			_plugin(plugin)
		{
			std::cout << "LinesUpdateCallback()" << std::endl;
		}

		/** do customized draw code.*/
		virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const
		{
			if ((_plugin->bounding_box_show_button->state() == 0) && (_plugin->frustum_show_button->state() == 0)) {
				return;
			}

			BACKUP_GL_STATE();

			GLdouble gl_mvm[16];
			glGetDoublev(GL_MODELVIEW_MATRIX, gl_mvm);
			GLdouble gl_pm[16];
			glGetDoublev(GL_PROJECTION_MATRIX,gl_pm);

			scm::math::mat4d gl_view_matrix_d = scm::math::mat4d(gl_mvm[0], gl_mvm[1], gl_mvm[2], gl_mvm[3], gl_mvm[4], gl_mvm[5], gl_mvm[6], gl_mvm[7], gl_mvm[8], gl_mvm[9], gl_mvm[10], gl_mvm[11], gl_mvm[12], gl_mvm[13], gl_mvm[14], gl_mvm[15]);
			scm::math::mat4d gl_projection_matrix_d = scm::math::mat4d(gl_pm[0],gl_pm[1],gl_pm[2],gl_pm[3],gl_pm[4],gl_pm[5],gl_pm[6],gl_pm[7],gl_pm[8],gl_pm[9],gl_pm[10],gl_pm[11],gl_pm[12],gl_pm[13],gl_pm[14],gl_pm[15]);
			
			scm::math::mat4d view_matrix = gl_view_matrix_d;
			scm::math::mat4d projection_matrix = gl_projection_matrix_d;

			float* vm = scm::math::mat4f(view_matrix).data_array;
			float* pm = scm::math::mat4f(projection_matrix).data_array;
			
			//initialize lamure objects
			lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
			lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
			lamure::ren::controller* controller = lamure::ren::controller::get_instance();
			lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();
			if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) { controller->reset_system(data_provenance_); }
			else { controller->reset_system(); }

			scm::math::vec3d translation = settings_.model_tl_;
			scm::math::mat4d translationMatrix = scm::math::make_translation(translation);
			scm::math::mat4d rotationMatrix = settings_.model_rot_;

			// get model_transformations 
			lamure::context_t context_id = controller->deduce_context_id(lmr_ctx);
			for (lamure::model_t model_id = 0; model_id < num_models_; ++model_id) {
				lamure::model_t m_id = controller->deduce_model_id(std::to_string(model_id)); 
				cuts->send_transform(context_id, m_id, scm::math::mat4f(model_info_.model_transformations_[m_id] * translationMatrix * rotationMatrix));
				cuts->send_threshold(context_id, m_id, settings_.lod_error_);
				cuts->send_rendered(context_id, m_id);
				database->get_model(m_id)->set_transform(scm::math::mat4f(model_info_.model_transformations_[m_id] * translationMatrix * rotationMatrix));
			}

			// send camera/view/frustum to get relevant database cuts
			lamure::view_t view_id = controller->deduce_view_id(context_id, scm_camera_->view_id());
			cuts->send_camera(context_id, view_id, *scm_camera_);
			std::vector<scm::math::vec3d> corner_values = scm_camera_->get_frustum_corners();
			double top_minus_bottom = scm::math::length((corner_values[2]) - (corner_values[0]));
			height_divided_by_top_minus_bottom_ = lamure::ren::policy::get_instance()->window_height() / top_minus_bottom;
			cuts->send_height_divided_by_top_minus_bottom(context_id, view_id, height_divided_by_top_minus_bottom_);

			// dispatch new data if lod_update is set
			if (settings_.lod_update_) {
				if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
					controller->dispatch(context_id, device_, data_provenance_);
				}
				else {
					controller->dispatch(context_id, device_);
				}
			}

			glBindBuffer(GL_ARRAY_BUFFER, box_resource_.vbo_);
			glBindVertexArray(box_resource_.vao_);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, box_resource_.ibo_);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, box_resource_.corners_idx_.size() * sizeof(unsigned short), box_resource_.corners_idx_.data(), GL_STATIC_DRAW);
			glUseProgram(box_resource_.program_);
			glUniformMatrix4fv(glGetUniformLocation(box_resource_.program_, "view_matrix"), 1, GL_FALSE, &vm[0]);
			glUniformMatrix4fv(glGetUniformLocation(box_resource_.program_, "projection_matrix"), 1, GL_FALSE, &pm[0]);

			glEnableVertexAttribArray(0);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);


			for (uint16_t model_id = 0; model_id < num_models_; ++model_id) {
				auto bvh_ = bvh_res_[model_id];

				scm::math::mat4d model_matrix = model_info_.model_transformations_[model_id] * translationMatrix * rotationMatrix;
				//scm::math::mat4d model_matrix = scm::math::mat4d::identity();
				scm::math::mat4d mvp_matrix = projection_matrix * view_matrix * model_matrix;
				scm::gl::frustum frustum_ = scm_camera_->get_frustum_by_model(scm::math::mat4f(model_matrix));

				float* mvp = scm::math::mat4f(mvp_matrix).data_array;

				glUniformMatrix4fv(glGetUniformLocation(box_resource_.program_, "mvp_matrix"), 1, GL_FALSE, &mvp[0]);

				lamure::context_t context_id = controller->deduce_context_id(lmr_ctx);
				lamure::ren::cut& cut = cuts->get_cut(context_id, lmr_ctx, model_id);
				std::vector<lamure::ren::cut::node_slot_aggregate> renderable = cut.complete_set();
				const lamure::ren::bvh* bvh = database->get_model(model_id)->get_bvh();
				std::vector<scm::gl::boxf>const& bbv = bvh->get_bounding_boxes();

				if (_plugin->bounding_box_show_button->state() == 1) {
					glUniform4f(glGetUniformLocation(box_resource_.program_, "in_color"), settings_.bvh_color_[0], settings_.bvh_color_[1], settings_.bvh_color_[2], settings_.bvh_color_[3]);
					bool draw = true;
					for (auto const& node_slot_aggregate : renderable) {
						uint32_t node_culling_result = scm_camera_->cull_against_frustum(frustum_, bbv[node_slot_aggregate.node_id_]);
						if (node_culling_result != 1) {
							std::vector<float> corners_ = bvh_.corners_[node_slot_aggregate.node_id_];
							glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 24, &corners_[0], GL_STATIC_DRAW);
							glDrawElements(GL_LINES, 24, GL_UNSIGNED_SHORT, nullptr);
						}
					}
				}
			}

			scm::math::vec3d cam_pos = scm_camera_->get_cam_pos();
			scm::math::mat4d model_matrix = scm::math::make_translation(cam_pos);
			scm::math::mat4d mvp_matrix = projection_matrix * view_matrix * model_matrix;

			float* mvp = scm::math::mat4f(mvp_matrix).data_array;

			glUniformMatrix4fv(glGetUniformLocation(box_resource_.program_, "mvp_matrix"), 1, GL_FALSE, &mvp[0]);

			if (_plugin->frustum_show_button->state() == 1) {

				glUniform4f(glGetUniformLocation(box_resource_.program_, "in_color"), settings_.frustum_color_[0], settings_.frustum_color_[1], settings_.frustum_color_[2], settings_.frustum_color_[3]);
				for (int i = 0; i < frustum_resource_.corners_.size(); i++) {
					std::vector<float> corners_ = frustum_resource_.corners_[i];
					glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 24, &corners_[0], GL_STATIC_DRAW);
					glDrawElements(GL_LINES, 24, GL_UNSIGNED_SHORT, nullptr);
				}

				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, plane_resource_.ibo_);
				glBufferData(GL_ELEMENT_ARRAY_BUFFER, plane_resource_.corners_idx_.size() * sizeof(unsigned short), plane_resource_.corners_idx_.data(), GL_DYNAMIC_DRAW);
				glDrawElements(GL_TRIANGLES, 8, GL_UNSIGNED_SHORT, nullptr);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

				scm::math::vec3f cam_pos = scm::math::vec3f(scm_camera_->get_cam_pos());
				scm::math::vec3f model_center = scm::math::vec3f(model_info_.models_center);
				scm::math::vec3f origin = scm::math::vec3f::zero();

				std::vector<float> points = {
					origin.x, origin.y, origin.z
				};


				glBindBuffer(GL_ARRAY_BUFFER, sphere_resource_.vbo_);
				glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3, &points[0], GL_STATIC_DRAW);

				glEnableVertexAttribArray(0);
				glVertexAttribPointer(0, 3, GL_FLOAT, false, 0, 0);

				glEnable(GL_PROGRAM_POINT_SIZE);
				glEnable(GL_POINT_SMOOTH);
				glPointSize(10.0f); 

				//glUniform4f(glGetUniformLocation(box_resource_.program_, "in_color"), 0.0f, 1.0f, 0.0f, 1.0f);
				//glDrawArrays(GL_POINTS, 0, 1);

				//glUniform4f(glGetUniformLocation(box_resource_.program_, "in_color"), 1.0f, 0.0f, 0.0f, 1.0f);
				//glDrawArrays(GL_POINTS, 1, 1);

				glUniform4f(glGetUniformLocation(box_resource_.program_, "in_color"), 0.0f, 0.0f, 1.0f, 1.0f);
				glDrawArrays(GL_POINTS, 1, 1);

				glBindVertexArray(0);
			}

			glDisableVertexAttribArray(0);

			glBindVertexArray(0);
			glUseProgram(0);

			RESTORE_GL_STATE();

			drawable->drawImplementation(renderInfo);
		};
		GLGrp* _gl_grp = nullptr;
		osg::Stats* _viewerStats;
		LamurePointCloudPlugin* _plugin;
	};


	struct PointsDrawable : public osg::Drawable
	{
		PointsDrawable(struct GLGrp* gl_grp, osg::Stats* viewerStats, LamurePointCloudPlugin* plugin)
		{
			std::cout << "PointsDrawable()" << std::endl;
			setUseDisplayList(false);
			setUseVertexBufferObjects(true);
			setUseVertexArrayObject(false);
			setDrawCallback(new PointsUpdateCallback(gl_grp, viewerStats, plugin));
		}
	};

	struct PointsUpdateCallback : public virtual osg::Drawable::DrawCallback
	{
		PointsUpdateCallback(GLGrp* gl_grp, osg::Stats* viewerStats, LamurePointCloudPlugin* plugin)
			: _gl_grp(gl_grp),
			_viewerStats(viewerStats),
			_plugin(plugin)
		{
			std::cout << "protected struct PointsUpdateCallback()" << std::endl;
		}

		/** do customized draw code.*/
		virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const
		{
			if (_plugin->notify_button->state()) { std::cout << "[Notify toggle] PointsUpdateCallback::drawImplementation()" << std::endl; }
			if (_plugin->rendering_) { return; }
			_plugin->rendering_ = true;

			//BACKUP_GL_STATE();

			GLdouble vm[16];
			glGetDoublev(GL_MODELVIEW_MATRIX, vm);
			GLdouble pm[16];
			glGetDoublev(GL_PROJECTION_MATRIX, pm);

			scm::math::mat4d gl_view_matrix_d = scm::math::mat4d(vm[0], vm[1], vm[2], vm[3], vm[4], vm[5], vm[6], vm[7], vm[8], vm[9], vm[10], vm[11], vm[12], vm[13], vm[14], vm[15]);
			scm::math::mat4d gl_projection_matrix_d = scm::math::mat4d(pm[0], pm[1], pm[2], pm[3], pm[4], pm[5], pm[6], pm[7], pm[8], pm[9], pm[10], pm[11], pm[12], pm[13], pm[14], pm[15]);

			//screen_quad_.reset(new scm::gl::quad_geometry(device_, scm::math::vec2f(-1.0f, -1.0f), scm::math::vec2f(1.0f, 1.0f)));

			//initialize lamure objects
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

			// send camera/view/frustum to get relevant database cuts
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
				if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
					controller->dispatch(context_id, device_, data_provenance_);
				}
				else {
					controller->dispatch(context_id, device_);
				}
			}
			context_->set_rasterizer_state(no_backface_culling_rasterizer_state_, 1.0f, 1.0f);

			//multipass
			if (settings_.splatting_) {
				//PASS 1
				auto selected_pass1_shading_program = vis_xyz_pass1_shader_;
				//auto selected_pass1_shading_program = vis_xyz_shader_;
				context_->clear_color_buffer(pass1_fbo_, 0, scm::math::vec4f(settings_.background_color_.x, settings_.background_color_.y, settings_.background_color_.z, 1.0f));
				context_->clear_depth_stencil_buffer(pass1_fbo_);
				context_->set_frame_buffer(pass1_fbo_);
				context_->apply_frame_buffer();
				context_->bind_program(selected_pass1_shading_program);
				context_->set_blend_state(color_no_blending_state_);
				context_->set_depth_stencil_state(depth_state_less_);
				_plugin->set_uniforms(selected_pass1_shading_program);
				context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(traits->width, traits->height)));
				context_->apply();
				DRAW_ALL_MODELS(selected_pass1_shading_program);
				//PASS 2
				auto selected_pass2_shading_program = vis_xyz_pass2_shader_;
				if (settings_.enable_lighting_) { auto selected_pass2_shading_program = vis_xyz_pass2_lighting_shader_; }
				context_->clear_color_buffer(pass2_fbo_, 0, scm::math::vec4f(.0f, .0f, .0f, 0.0f));
				context_->clear_color_buffer(pass2_fbo_, 1, scm::math::vec4f(.0f, .0f, .0f, 0.0f));
				context_->clear_color_buffer(pass2_fbo_, 2, scm::math::vec4f(.0f, .0f, .0f, 0.0f));
				context_->set_frame_buffer(pass2_fbo_);
				context_->set_blend_state(color_blending_state_);
				context_->set_depth_stencil_state(depth_state_without_writing_);
				context_->set_rasterizer_state(no_backface_culling_rasterizer_state_);
				context_->bind_program(selected_pass2_shading_program);
				_plugin->set_uniforms(selected_pass2_shading_program);
				context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(traits->width, traits->height)));
				context_->apply();
				DRAW_ALL_MODELS(selected_pass2_shading_program);
				_plugin->draw_brush(selected_pass2_shading_program);
				//PASS 3
				auto selected_pass3_shading_program = vis_xyz_pass3_shader_;
				if (settings_.enable_lighting_) { selected_pass3_shading_program = vis_xyz_pass3_lighting_shader_; }
				context_->clear_color_buffer(fbo_, 0, scm::math::vec4f(0.0, 0.0, 0.0, 1.0f));
				context_->clear_depth_stencil_buffer(fbo_);
				context_->set_frame_buffer(fbo_);
				context_->set_depth_stencil_state(depth_state_disable_);
				context_->bind_program(selected_pass3_shading_program);
				_plugin->set_uniforms(selected_pass3_shading_program);
				selected_pass3_shading_program->uniform("background_color", scm::math::vec3f(settings_.background_color_.x, settings_.background_color_.y, settings_.background_color_.z));
				selected_pass3_shading_program->uniform_sampler("in_color_texture", 0);
				context_->bind_texture(pass2_color_buffer_, filter_nearest_, 0);
				if (settings_.enable_lighting_) {
					context_->bind_texture(pass2_normal_buffer_, filter_nearest_, 1);
					context_->bind_texture(pass2_view_space_pos_buffer_, filter_nearest_, 2);
				}
				context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(traits->width, traits->height)));
				context_->apply();

				//screen_quad_->draw(context_);
			}
			else {
				auto selected_single_pass_shading_program = vis_xyz_shader_;
				//auto selected_single_pass_shading_program = vis_surfel_shader_;
				if (settings_.enable_lighting_) {
					selected_single_pass_shading_program = vis_xyz_lighting_shader_;
				}
				context_->clear_color_buffer(fbo_, 0, scm::math::vec4f(settings_.background_color_.x, settings_.background_color_.y, settings_.background_color_.z, 1.0f));
				context_->clear_depth_stencil_buffer(fbo_);
				context_->set_frame_buffer(fbo_);
				context_->apply_frame_buffer();
				context_->bind_program(selected_single_pass_shading_program);
				context_->set_blend_state(color_no_blending_state_);
				context_->set_depth_stencil_state(depth_state_less_);
				_plugin->set_uniforms(selected_single_pass_shading_program);
				context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(traits->width, traits->height)));
				context_->apply();

				// ab hier: draw_all_models
				DRAW_ALL_MODELS(selected_single_pass_shading_program);

				//if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
				//        context_->bind_vertex_array(controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, device_, data_provenance_));
				//}
				//else { context_->bind_vertex_array(controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, device_)); }

				//rendered_splats_ = 0;
				//rendered_nodes_ = 0;

				//for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
				//    lamure::context_t context_id = controller->deduce_context_id(lmr_ctx);
				//    lamure::ren::cut& cut = cuts->get_cut(context_id, lmr_ctx, model_id);
				//    std::vector<lamure::ren::cut::node_slot_aggregate> renderable = cut.complete_set();
				//    const lamure::ren::bvh* bvh = database->get_model(model_id)->get_bvh();
				//    size_t surfels_per_node = database->get_primitives_per_node();
				//    std::vector<scm::gl::boxf>const& bounding_box_vector = bvh->get_bounding_boxes();

				//    scm::math::mat4d model_matrix = model_info_.model_transformations_[model_id];
				//    scm::gl::frustum frustum_by_model = scm_camera_->get_frustum_by_model(scm::math::mat4f(model_matrix));
				//    scm::math::mat4d projection_matrix = scm::math::mat4d(gl_projection_matrix_d);
				//    scm::math::mat4d view_matrix = gl_view_matrix_d;
				//    scm::math::mat4d model_view_matrix = view_matrix * model_matrix;
				//    scm::math::mat4d model_view_projection_matrix = projection_matrix * model_view_matrix;

				//    shader_->uniform("mvp_matrix", scm::math::mat4f(model_view_projection_matrix));
				//    shader_->uniform("model_matrix", scm::math::mat4f(model_matrix));
				//    shader_->uniform("model_view_matrix", scm::math::mat4f(model_view_matrix));
				//    shader_->uniform("inv_mv_matrix", scm::math::mat4f(scm::math::transpose(scm::math::inverse(model_view_matrix))));

				//    const scm::math::mat4d viewport_scale = scm::math::make_scale(traits->width * 0.5, traits->height * 0.5, 0.5);
				//    const scm::math::mat4d viewport_translate = scm::math::make_translation(1.0, 1.0, 1.0);
				//    const scm::math::mat4d model_to_screen = viewport_scale * viewport_translate;

				//    shader_->uniform("model_to_screen_matrix", scm::math::mat4f(model_to_screen));
				//    shader_->uniform("model_radius_scale", settings_.scale_radius_);
				//    shader_->uniform("max_radius", settings_.max_radius_);

				//    glEnable(GL_PROGRAM_POINT_SIZE);
				//    glEnable(GL_POINT_SMOOTH);

				//    context_->apply_uniform_buffer_bindings();
				//    context_->bind_program(shader_);
				//    context_->set_blend_state(color_no_blending_state_);
				//    context_->set_depth_stencil_state(depth_state_less_);
				//    context_->apply_state_objects();
				//    context_->apply_program();
				//    bool draw = true;
				//    for (auto const& node_slot_aggregate : renderable) {
				//        uint32_t node_culling_result = scm_camera_->cull_against_frustum(frustum_by_model, bounding_box_vector[node_slot_aggregate.node_id_]);
				//        if (node_culling_result != 1) {
				//            if (draw) {
				//                    context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, (node_slot_aggregate.slot_id_) * (GLsizei)surfels_per_node, surfels_per_node);
				//                    rendered_splats_ += surfels_per_node;
				//                    ++rendered_nodes_;
				//            }
				//        }
				//    }
				//}
			}

			//PASS 4: fullscreen quad
			//context_->clear_default_depth_stencil_buffer();
			//context_->clear_default_color_buffer();
			//context_->set_default_frame_buffer();
			//context_->set_depth_stencil_state(depth_state_disable_);
			//context_->bind_program(vis_quad_shader_);
			//context_->bind_texture(fbo_color_buffer_, filter_linear_, 0);
			//vis_quad_shader_->uniform("gamma_correction", (bool)settings_.gamma_correction_);
			//context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(traits->width, traits->height)));
			//context_->apply();
			//screen_quad_->draw(context_);
			_plugin->rendering_ = false;
			drawable->drawImplementation(renderInfo);
			//RESTORE_GL_STATE();
		}
		GLGrp* _gl_grp = nullptr;
		osg::Stats* _viewerStats;
		LamurePointCloudPlugin* _plugin;
	};
};


unsigned int counter = 0;
void LamurePointCloudPlugin::preFrame() {
	if (cover->getPointerButton()->getState() == 1 && counter == 0) {
		plugin->gl_grp = new GLGrp();
		plugin->LamureGroup->addChild(plugin->gl_grp);
		plugin->gl_grp->addPoints(VRViewer::instance()->getViewerStats(), plugin);
		plugin->gl_grp->addLines(VRViewer::instance()->getViewerStats(), plugin);
		//plugin->gl_grp->addTriangles(VRViewer::instance()->getViewerStats(), plugin);
		printNodePath(plugin->gl_grp);
		counter = counter + 1;
	}
	else if (counter != 0) {
		if (plugin->notify_button->state() == 0) {
			osg_camera_->getGraphicsContext()->getState()->setCheckForGLErrors(osg::State::NEVER_CHECK_GL_ERRORS);
		}
		else {
			osg_camera_->getGraphicsContext()->getState()->setCheckForGLErrors(osg::State::ONCE_PER_FRAME);
		}
		if (plugin->sync_cam_button->state() == 1 && cover->getPointerButton()->getState()) {
			plugin->sync_cameras();

			//scm::math::mat4f projection_matrix = scm::math::mat4f(scm_camera_->get_projection_matrix());
			//scm::math::mat4f view_matrix = scm_camera_->get_view_matrix();
			//scm::math::vec3d cam_pos = scm_camera_->get_cam_pos();

			//std::cout << "projection_matrix:" << std::endl << projection_matrix << std::endl;
			//std::cout << "view_matrix:" << std::endl << view_matrix << std::endl;
			//std::cout << "cam_pos:" << std::endl << cam_pos << std::endl;
		}
	}
}


scm::math::mat4f createSwapYZMatrix() {
	scm::math::mat4f swapMatrix = scm::math::mat4f::identity();
	swapMatrix[5] = 0.0f;
	swapMatrix[6] = 1.0f;
	swapMatrix[9] = -1.0f;
	swapMatrix[10] = 0.0f;
	return swapMatrix;
}


scm::math::mat4d createSwapYZ() {
	scm::math::mat4d swapMatrix = scm::math::mat4d::identity();
	swapMatrix[5] = 0.0f;
	swapMatrix[6] = 1.0f;
	swapMatrix[9] = -1.0f;
	swapMatrix[10] = 0.0f;
	return swapMatrix;
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

	scm::math::mat4d swapMatrix = createSwapYZ();

	//scm_camera_->set_view_matrix(createSwapYZ() * matConv4D(VRViewer::instance()->getViewerMat()));

	scm::math::mat4d model_matrix = model_info_.model_transformations_[0];
	scm::gl::frustum frustum_ = scm_camera_->get_frustum_by_model(scm::math::mat4f(model_matrix));

	scm::math::mat4f vm = createSwapYZMatrix() * scm_camera_->get_view_matrix();
	cover->setXformMat(matConv4F(vm));
	//cover->pos;

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

	//std::cout << "base_matrix" << std::endl << base_matrix << std::endl << std::endl;
	//std::cout << "x_form_matrix" << std::endl << x_form_matrix << std::endl << std::endl;
	//std::cout << "viewer_matrix" << std::endl << viewer_matrix << std::endl << std::endl;

	//scm::math::mat4d model_matrix = model_info_.model_transformations_[0];
	//scm::math::mat4d model_view_matrix = view_matrix_osg * model_matrix;
	//scm::math::mat4d inv_mv_matrix = scm::math::transpose(scm::math::inverse(model_view_matrix));
	//scm::math::mat4d mvp_matrix = proj_matrix_osg * model_view_matrix;

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
	//std::cout << "mvp_matrix" << std::endl << mvp_matrix << std::endl << std::endl;
	//std::cout << "view_matrix_osg" << std::endl << view_matrix_osg << std::endl << std::endl;
	//std::cout << "proj_matrix_osg" << std::endl << proj_matrix_osg << std::endl << std::endl;
	//std::cout << "model_view_matrix" << std::endl << model_view_matrix << std::endl << std::endl;
	//std::cout << "model_matrix" << std::endl << model_matrix << std::endl << std::endl;

	//std::cout << "viewport_scale" << std::endl << viewport_scale << std::endl << std::endl;
	//std::cout << "viewport_translate" << std::endl << viewport_translate << std::endl << std::endl;
	//std::cout << "model_to_screen" << std::endl << model_to_screen << std::endl << std::endl;
}


//void LamurePointCloudPlugin::init_rtt_camera() {
	//osg::GraphicsContext* gc = coVRConfig::instance()->windows[0].context;
	//uint8_t ctx_id = gc->getState()->getContextID();
	//osg::GLExtensions* gl_api = new osg::GLExtensions(ctx_id);
	//osg::GraphicsContext* rttGc = coVRConfig::instance()->windows[0].context;
	//osg::GraphicsContext::GraphicsContexts gcs = osg::GraphicsContext::getAllRegisteredGraphicsContexts();
	//osg::GraphicsContext::GraphicsContexts gcs1 = osg::GraphicsContext::getRegisteredGraphicsContexts(0);
	//osg_cam_ = VRViewer::instance()->getCamera();
	//osg::ref_ptr<osg::Camera> rttCamera = new osg::Camera;
	//rttCamera->setGraphicsContext(rttGc);
	//rttCamera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
	//rttCamera->setRenderOrder(osg::Camera::PRE_RENDER);
	//rttCamera->setName("RenderTexture");
	//_texture = new osg::Texture2D();
	//_texture->setSourceFormat(GL_RGBA);
	//_texture->setInternalFormat(GL_RGBA32F_ARB);
	//_texture->setSourceType(GL_FLOAT);
	//attach(osg::Camera::COLOR_BUFFER0, _texture);
	//gl_api->glGenFramebuffers(1, &(context_bindable_object::_gl_object_id));
	//_selected_color_attachments.resize(in_device.capabilities()._max_frame_buffer_color_attachments);
	//_current_color_attachments.resize(in_device.capabilities()._max_frame_buffer_color_attachments);
	//_draw_buffers.resize(in_device.capabilities()._max_frame_buffer_color_attachments);
	//std::fill(_draw_buffers.begin(), _draw_buffers.end(), GL_NONE);
//}


void LamurePointCloudPlugin::init_camera() {
	osg_camera_ = VRViewer::instance()->getCamera();
	lmr_ctx = osg_camera_->getGraphicsContext()->getState()->getContextID();

	// zur Berechnung von Zentrum und min/max Vertex über alle Modelle
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

	scm_camera_ = new lamure::ren::camera(
		(lamure::view_t)lmr_ctx,
		scm::math::vec3f(model_info_.models_center),
		settings_.fov_,
		float(traits->width) / float(traits->height),
		float(settings_.near_plane_),
		float(settings_.far_plane_)
	);

	screen_quad_.reset(new scm::gl::quad_geometry(device_, scm::math::vec2f(-1.0f, -1.0f), scm::math::vec2f(1.0f, 1.0f)));

	//scm_camera_ = new lamure::ren::camera(
	//	(lamure::view_t)lmr_ctx,
	//  scm::math::make_look_at_matrix(model_info_.models_center + scm::math::vec3f(0.f, 0.1f, -0.01f), model_info_.models_center, scm::math::vec3f(0.0f, 0.0f, 1.0f)),
	//  length(model_info_.models_max - model_info_.models_min));
	//scm_camera_->set_projection_matrix(settings_.fov_, float(traits->width / traits->height), settings_.near_plane_, settings_.far_plane_);

	//scm_camera_ = new lamure::ren::camera(
	//	(lamure::view_t)lmr_ctx,
	//	scm::math::vec3f(model_info_.models_center),
	//	length(model_info_.models_max - model_info_.models_min),
	//	settings_.fov_, 
	//	float(traits->width) / float(traits->height), 
	//	float(settings_.near_plane_),
	//	float(settings_.far_plane_));

	//scm_camera_->set_projection_matrix(settings_.fov_, float(traits->width / traits->height), settings_.near_plane_, settings_.far_plane_);

	//coVRConfig::instance()->channels[0].camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);

	//osg::Vec3f eye, center_osg, up;
	//osg_camera_->getViewMatrixAsLookAt(eye, center_osg, up);
	//scm_camera_ = new lamure::ren::camera(
	//    (lamure::view_t)lmr_ctx,
	//    scm::math::make_look_at_matrix(vecConv3F(eye), vecConv3F(center_osg), vecConv3F(up)),
	//    length(root_bb_max - root_bb_min), false, false
	//);
	//double fovy, aspectRatio, zNear, zFar;
	//osg_camera_->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
	//scm_camera_->set_projection_matrix(fovy, aspectRatio, zNear, zFar);

	//lamure_camera_ = lmr_camera::instance(lmr_ctx);

	//osg::Matrixd& view_mat_osg = osg_camera_->getViewMatrix();
	//scm::math::mat4d view_mat = matConv4D(view_mat_osg);

	//osg::Matrixd& proj_mat_osg = osg_camera_->getProjectionMatrix();
	//scm::math::mat4d proj_mat = matConv4D(proj_mat_osg);

	//scm_camera_->set_hp_view_matrix(view_mat);
	//scm_camera_->set_hp_projection_matrix(proj_mat);

	//lamure_camera_ = lmr_camera::instance(lmr_ctx);
	//const scm::math::mat4d look_at_matrix = scm::math::mat4d(scm::math::make_look_at_matrix(center + scm::math::vec3f(0.f, 0.1f, -0.01f), center, scm::math::vec3f(0.f, 1.f, 0.f)));
	//lamure_camera_->set_view_matrix(look_at_matrix);
	//const scm::math::mat4d perspective_matrix = scm::math::mat4d(scm::math::make_perspective_matrix((float)settings_.fov_, (float)(traits->width/traits->height), (float)settings_.near_plane_, (float)settings_.far_plane_));
	//lamure_camera_->set_hp_projection_matrix(perspective_matrix);
	//lamure_camera_->set_near_plane_value(settings_.near_plane_);
	//lamure_camera_->set_far_plane_value(settings_.far_plane_);
	//scm::gl::frustum temp_frustum_ = lamure_camera_->calc_get_frustum();

	//osg::Polytope frustum;
	//frustum.setToUnitFrustum(false, false);
	//frustum.transformProvidingInverse(*(proj_mat_osg));
	//frustum.transformProvidingInverse(*(cv->getModelViewMatrix()));

	//osgViewer::GraphicsWindow* gw = dynamic_cast<osgViewer::GraphicsWindow*>(osg_camera_->getGraphicsContext());
	//const osg::GraphicsContext::Traits* traits = osg_camera_->getGraphicsContext()->getTraits();

	//rtt_camera_ = new osg::Camera;
	//rtt_camera_->setGraphicsContext(gw);
	//rtt_camera_->setViewport(gw->getTraits()->x, gw->getTraits()->y, gw->getTraits()->width, gw->getTraits()->height);
	//rtt_camera_->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	//rtt_camera_->setRenderOrder(osg::Camera::PRE_RENDER, 10);
	//rtt_camera_->setProjectionMatrix(osg::Matrix::ortho2D(0.0, (double)traits->width, 0.0, (double)traits->height));
	//rtt_camera_->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	//rtt_camera_->setViewMatrix(osg::Matrix::identity());
	//rtt_camera_->setClearMask(0); // only clear the depth buffer
	//rtt_camera_->setRenderer(new osgViewer::Renderer(rtt_camera_.get()));
}


void LamurePointCloudPlugin::setUpStateSets() {

	unsigned int trianglesRenderBin = 12;
	unsigned int lineRenderBin = 12;
	unsigned int pointRenderBin = 11;

	OSG_INFO << "LamurePointCloudPlugin::setUpStateSets" << std::endl;

	if (!_triangleStateSet)
	{
		_triangleStateSet = new osg::StateSet;
		_triangleStateSet->setName("triangle_stateset");

		osg::Program* program = new osg::Program;

		std::string vertexShader =
			"#version 330 core\n"
			"layout(location = 0) in vec3 position;\n"
			"uniform mat4 mts;\n"
			"uniform mat4 model;\n"
			"uniform mat4 view;\n"
			"uniform mat4 projection;\n"
			"void main()\n"
			"{\n"
			"   gl_Position = projection * view * vec4(position, 1.0);\n"
			"}\n";

		std::string fragmentShader =
			"#version 330 core\n"
			"out vec4 color;\n"
			"void main()\n"
			"{\n"
			"   color = vec4(0.2, 0.1, 0.3, 0.8);\n"
			"}\n";

		program->addShader(new osg::Shader(osg::Shader::VERTEX, vertexShader));
		program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragmentShader));

		_triangleStateSet->setAttribute(program);
		_triangleStateSet->setRenderBinDetails(trianglesRenderBin, "DepthSortedBin");
	}

	if (!_lineStateSet)
	{
		_lineStateSet = new osg::StateSet;

		osg::Program* program = new osg::Program;
		_lineStateSet->setAttribute(program);
		_lineStateSet->setRenderBinDetails(lineRenderBin, "DepthSortedBin");
		program->addShader(new osg::Shader(osg::Shader::VERTEX, vis_line_vs_source));
		program->addShader(new osg::Shader(osg::Shader::FRAGMENT, vis_line_fs_source));
	}

	if (!_pointStateSet)
	{
		_pointStateSet = new osg::StateSet;

		osg::Program* program = new osg::Program;
		_pointStateSet->setAttribute(program);
		_pointStateSet->setRenderBinDetails(pointRenderBin, "DepthSortedBin");

		program->addShader(new osg::Shader(osg::Shader::VERTEX, vis_surfel_shader_vs_source));
		program->addShader(new osg::Shader(osg::Shader::FRAGMENT, vis_surfel_shader_fs_source));

		/// Setup the point sprites
		osg::PointSprite* sprite = new osg::PointSprite();
		_pointStateSet->setTextureAttributeAndModes(0, sprite, osg::StateAttribute::ON);

#if !defined(OSG_GLES1_AVAILABLE) && !defined(OSG_GLES2_AVAILABLE)
		_pointStateSet->setMode(GL_VERTEX_PROGRAM_POINT_SIZE, osg::StateAttribute::ON);
#else
		OSG_NOTICE << "Warning: ParticleEffect::setUpGeometries(..) not fully implemented." << std::endl;
#endif
	}
}


void LamurePointCloudPlugin::postFrame() {
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


void LamurePointCloudPlugin::create_pcl_resources() {
	std::cout << "create_pcl_resources() " << std::endl;
	unsigned int program = glCreateProgram();
	unsigned int vs = CompileShader(GL_VERTEX_SHADER, vis_surfel_shader_vs_source, 0);
	unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, vis_surfel_shader_fs_source, 0);
	glAttachShader(program, vs);
	glAttachShader(program, fs);
	glLinkProgram(program);
	glValidateProgram(program);
	glDeleteProgram(vs);
	glDeleteProgram(fs);
	pcl_resource_;
}


void LamurePointCloudPlugin::create_aux_resources() {
	if (!settings_.create_aux_resources_) {
		return;
	}

	// init bounding box rendering

	GLuint vbo;
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);

	GLuint vao;
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	GLuint ibo;
	glGenBuffers(1, &ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, box_resource_.corners_idx_.size() * sizeof(unsigned short), box_resource_.corners_idx_.data(), GL_STATIC_DRAW);

	unsigned int program = glCreateProgram();
	unsigned int vs = CompileShader(GL_VERTEX_SHADER, vis_line_bb_vs_source, 0);
	unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, vis_line_bb_fs_source, 0);
	glAttachShader(program, vs);
	glAttachShader(program, fs);
	glLinkProgram(program);
	glValidateProgram(program);
	glDeleteProgram(vs);
	glDeleteProgram(fs);

	box_resource_.vbo_ = vbo;
	box_resource_.vao_ = vao;
	box_resource_.ibo_ = ibo;
	box_resource_.program_ = program;

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

	// init bounding box rendering
	GLuint ibo_p;
	glGenBuffers(1, &ibo_p);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_p);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, plane_resource_.corners_idx_.size() * sizeof(unsigned short), plane_resource_.corners_idx_.data(), GL_STATIC_DRAW);
	plane_resource_.ibo_ = ibo_p;

	//scm::gl::frustum_impl fr = scm_camera_->get_frustum();

	//create initial frustum representation
	std::vector<scm::math::vec3d> corner_values = scm_camera_->get_frustum_corners();
	vector<float> corners_;
	for (uint64_t i = 0; i < corner_values.size(); ++i) {
		auto vv = scm::math::vec3f(corner_values[i]);
		corners_.insert(corners_.end(), { vv.x, vv.y, vv.z });
	}
	frustum_resource_.corners_.push_back(corners_);
	//create bvh representation
	for (uint32_t model_id = 0; model_id < num_models_; ++model_id) {
		const auto& bvh_ = lamure::ren::model_database::get_instance()->get_model(model_id)->get_bvh();
		const auto& bounding_boxes = bvh_->get_bounding_boxes();
		std::vector<vector<float>> corners_;
		for (uint64_t node_id = 0; node_id < bounding_boxes.size(); ++node_id) {
			corners_.push_back(getBoxCorners(bounding_boxes[node_id]));
		}
		bvh_res_[model_id].corners_ = corners_;
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


bool LamurePointCloudPlugin::read_shader(std::string const& path_string, std::string& shader_string, bool keep_optional_shader_code = false) {
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
	std::cout << "init_lamure_shader()" << std::endl;
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
	std::cout << "create_framebuffers()" << std::endl;
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
	fbo_depth_buffer_ = device_->create_texture_2d(scm::math::vec2ui(traits->width, render_height_), scm::gl::FORMAT_D24, 1, 1, 1);
	fbo_->attach_color_buffer(0, fbo_color_buffer_);
	fbo_->attach_depth_stencil_buffer(fbo_depth_buffer_);

	pass1_fbo_ = device_->create_frame_buffer();
	pass1_depth_buffer_ = device_->create_texture_2d(scm::math::vec2ui(traits->width, render_height_), scm::gl::FORMAT_D24, 1, 1, 1);
	pass1_fbo_->attach_depth_stencil_buffer(pass1_depth_buffer_);

	pass2_fbo_ = device_->create_frame_buffer();
	pass2_color_buffer_ = device_->create_texture_2d(scm::math::vec2ui(traits->width, render_height_), scm::gl::FORMAT_RGBA_32F, 1, 1, 1);
	pass2_fbo_->attach_color_buffer(0, pass2_color_buffer_);
	pass2_fbo_->attach_depth_stencil_buffer(pass1_depth_buffer_);

	pass2_normal_buffer_ = device_->create_texture_2d(scm::math::vec2ui(traits->width, render_height_), scm::gl::FORMAT_RGB_32F, 1, 1, 1);
	pass2_fbo_->attach_color_buffer(1, pass2_normal_buffer_);
	pass2_view_space_pos_buffer_ = device_->create_texture_2d(scm::math::vec2ui(traits->width, render_height_), scm::gl::FORMAT_RGB_32F, 1, 1, 1);
	pass2_fbo_->attach_color_buffer(2, pass2_view_space_pos_buffer_);
}


void LamurePointCloudPlugin::init_render_states() {
	std::cout << "init_render_states()" << std::endl;
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


void LamurePointCloudPlugin::set_uniforms(scm::gl::program_ptr shader) {
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


struct DrawableNode : public osg::Drawable {
	DrawableNode()
	{
	}
	virtual void drawImplementation(osg::RenderInfo& renderInfo) {
		std::cout << "Drawables drawImplementation, every loop?" << std::endl;
	}
};


struct Event : public virtual osg::Drawable::EventCallback
{
	Event() {
		std::cout << "Event worked" << std::endl;
	}

	virtual void drawImplementation(osg::RenderInfo& renderInfo) const
	{
		std::cout << "drawImplementation Event worked" << std::endl;
		drawImplementation(renderInfo);
	}
};


struct Draw : public virtual osg::Drawable::DrawCallback
{
	Draw() {
		std::cout << "draw worked" << std::endl;
	}

	virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const
	{
		drawable->drawImplementation(renderInfo);
		std::cout << "Drawcallbacks drawImplementation" << std::endl;

		glBegin(GL_TRIANGLES);
		{
			glVertex3f(-500.0f, 0.0f, -500.0f);
			glVertex3f(500.0f, 0.0f, 500.0f);
			glVertex3f(500.0f, 0.0f, -500);
		}
		glEnd();
	}
};


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
	//assertions
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
		set_uniforms(shader);
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

			frustum_resource_ = lines_resource;
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
				scm::gl::frustum frustum_by_model = scm_camera_->get_frustum_by_model(scm::math::mat4f(model_matrix));

				auto bvh_res = bvh_res_[model_id];
				if (bvh_res.num_primitives_ > 0) {
					context_->bind_vertex_array(bvh_res.array_);
					context_->apply();
					for (auto const& node_slot_aggregate : renderable) {
						uint32_t node_culling_result = scm_camera_->cull_against_frustum(frustum_by_model, bounding_box_vector[node_slot_aggregate.node_id_]);
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

			LamurePointCloudPlugin::instance()->set_uniforms(vis_xyz_shader_);

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
		if (settings_.show_views_ || settings_.show_octrees_) {
			context_->bind_program(vis_line_shader_);

			scm::math::mat4f projection_matrix = scm::math::mat4f(scm_camera_->get_projection_matrix());
			scm::math::mat4f view_matrix = scm_camera_->get_view_matrix();
			vis_line_shader_->uniform("model_matrix", scm::math::mat4f::identity());
			vis_line_shader_->uniform("view_matrix", view_matrix);
			vis_line_shader_->uniform("projection_matrix", projection_matrix);

			for (uint32_t model_id = 0; model_id < num_models_; ++model_id) {
				if (selection_.selected_model_ != -1) {
					model_id = selection_.selected_model_;
				}
				if (settings_.show_views_) {
					uint32_t num_views = provenance_[model_id].num_views_;
					auto f_res = frustum_resource_;
					if (f_res.num_primitives_ > 0) {
						context_->bind_vertex_array(f_res.array_);
						context_->apply();
						if (selection_.selected_views_.empty()) {
							context_->draw_arrays(scm::gl::PRIMITIVE_LINE_LIST, 0, f_res.num_primitives_);
						}
						else {
							for (const auto view : selection_.selected_views_) {
								context_->draw_arrays(scm::gl::PRIMITIVE_LINE_LIST, view * 16, 16);
							}
						}
					}
				}
				if (settings_.show_octrees_) {
					auto o_res = octree_res_[model_id];
					if (o_res.num_primitives_ > 0) {
						context_->bind_vertex_array(o_res.array_);
						context_->apply();
						context_->draw_arrays(scm::gl::PRIMITIVE_LINE_LIST, 0, o_res.num_primitives_);
					}
				}
				if (selection_.selected_model_ != -1) {
					break;
				}
			}
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


/*
bool LamurePointCloudPlugin::ImplGlfwGL3_CreateFontsTexture() {
	// Build texture atlas
	ImGuiIO& io = ImGui::GetIO();
	unsigned char* pixels;
	int width, height;
	io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);   // Load as RGBA 32-bits (75% of the memory is wasted, but default font is so small) because it is more likely to be compatible with user's existing shaders. If your ImTextureId represent a higher-level concept than just a GL texture id, consider calling GetTexDataAsAlpha8() instead to save on GPU memory.

	// Upload texture to graphics system
	GLint last_texture;
	glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
	glGenTextures(1, &g_FontTexture);
	glBindTexture(GL_TEXTURE_2D, g_FontTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

	// Store our identifier
	io.Fonts->TexID = (void*)(intptr_t)g_FontTexture;

	// Restore state
	glBindTexture(GL_TEXTURE_2D, last_texture);

	return true;

}

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
//            for (int node_id = 0; node_id < 5000; node_id++) {
//
//            }
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