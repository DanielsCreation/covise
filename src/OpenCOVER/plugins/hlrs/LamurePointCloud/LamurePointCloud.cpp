//local
#include "LamurePointCloud.h"
//#include "LamureDrawable.h"

//std
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
#include <boost/regex/v4/regex.hpp>
#include <boost/regex/v4/regex_replace.hpp>

#include <config/coConfigConstants.h>
#include <config/coConfigLog.h>
#include <config/coConfigConstants.h>
#include <config/coConfig.h>
#include <cover/ui/SelectionList.h>
#include <cover/coVRStatsDisplay.h>
#include <cover/VRSceneGraph.h>
#include <config/coConfigConstants.h>
#include <C:\src\covise\src\OpenCOVER\cover\ui\FileBrowser.h>
#include <C:\src\covise\src\3rdparty\deskvox/virvo/virvo/vvtoolshed.h>
#include <config/coConfigLog.h>
#include <config/coConfig.h>
#include <config/coConfigString.h>
#include <config\coConfigEntryString.h>

#include "cover/OpenCOVER.h"
#include <cover/VRWindow.h>
#include <cover/VRViewer.h>

#define ASSERT(x) if (!(x)) __debugbreak();
#define GLCall(x) GLClearError();\
    x;\
    ASSERT(GLLogCall(#x,__FILE__, __LINE__))

#ifdef __cplusplus
extern "C" {
#endif

    __declspec(dllexport) DWORD NvOptimusEnablement = 1;
    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;

#ifdef __cplusplus
}
#endif

int32_t num_models_ = 0;
std::vector<scm::math::mat4d> model_transformations_;

scm::shared_ptr<scm::gl::render_device>     device_;
scm::shared_ptr<scm::gl::render_context>    context_;

camera* camera_ = nullptr;

double fps_ = 0.0;
uint64_t rendered_splats_ = 0;
uint64_t rendered_nodes_ = 0;

Data_Provenance data_provenance_;
float height_divided_by_top_minus_bottom_ = 0.f;
coVRShader* pointShader = coVRShaderList::instance()->get("Points");

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
scm::gl::frame_buffer_ptr pass2_fbo_;
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

scm::shared_ptr<scm::gl::quad_geometry> screen_quad_;
scm::time::accum_timer<scm::time::high_res_timer> frame_time_;

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


struct input {
    float trackball_x_ = 0.f;
    float trackball_y_ = 0.f;
    scm::math::vec2i mouse_;
    scm::math::vec2i prev_mouse_;
    bool brush_mode_ = 0;
    bool brush_clear_ = 0;
    bool gui_lock_ = false;
    camera::mouse_state mouse_state_;
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
    int32_t width_{ 800 };
    int32_t height_{ 450 };
    int32_t frame_div_{ 1 };
    int32_t vram_{ 2048 };
    int32_t ram_{ 4096 };
    int32_t upload_{ 32 };
    bool provenance_{ 0 };
    bool create_aux_resources_{ 0 };
    float near_plane_{ 0.001f };
    float far_plane_{ 1000.0f };
    float fov_{ 30.0f };
    bool splatting_{ 1 };
    bool gamma_correction_{ 1 };
    int32_t gui_{ 0 };
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


using namespace osg;
using namespace std;
using covise::coCoviseConfig;
using vrui::coInteraction;

COVERPLUGIN(LamurePointCloudPlugin)
LamurePointCloudPlugin* LamurePointCloudPlugin::plugin = nullptr;

static FileHandler handler = 
    { NULL, 
      LamurePointCloudPlugin::loadLMR, 
      LamurePointCloudPlugin::unloadLMR, 
      "lmr"
};


// Constructor
LamurePointCloudPlugin::LamurePointCloudPlugin(): ui::Owner("LamurePointCloud", cover->ui)
{
    coVRFileManager::instance()->registerFileHandler(&handler);
    plugin = this;
}


LamurePointCloudPlugin::~LamurePointCloudPlugin()
{
    fprintf(stderr, "LamurePlugin::~LamurePlugin\n");
    coVRFileManager::instance()->unregisterFileHandler(&handler);
    cover->getObjectsRoot()->removeChild(LamureGroup);
}


bool LamurePointCloudPlugin::init() 
{
    printf("init()\n");
    cover->addPlugin("Move");
    cover->addPlugin("Annotation");

    std::cerr << "hostname: " << covise::coConfigConstants::getHostname() << std::endl;

    //Create main menu button
    lamureMenu = new ui::Menu("LamureMenu", this);
    lamureMenu->setText("LamurePlugin");

    //loadGroup = new ui::Group("Load", loadMenu);
    //deleteButton = new ui::Button(fileGroup,"Delete");

    selectionGroup = new ui::Group(lamureMenu, "Selection");
    selectionButtonGroup = new ui::ButtonGroup(selectionGroup, "SelectionGroup");
    selectionButtonGroup->enableDeselect(true);
    singleSelectButton = new ui::Button(selectionGroup, "SelectPoints", selectionButtonGroup);
    singleSelectButton->setText("Select Points");
    
    translationButton = new ui::Button(selectionGroup, "MakeTranslate", selectionButtonGroup);
    translationButton->setText("Make Translate");
    
    rotPointsButton = new ui::Button(selectionGroup, "RotationbyPointsSelection", selectionButtonGroup);
    rotPointsButton->setText("Rotation by Points Selection");
   
    rotAxisButton = new ui::Button(selectionGroup, "RotationAxisbyPointer", selectionButtonGroup);
    rotAxisButton->setText("Rotation Axis by Pointer");
    
    moveButton = new ui::Button(selectionGroup, "FreeMovement", selectionButtonGroup);
    moveButton->setText("Free Movement");

    deselectButton = new ui::Button(selectionGroup, "DeselectPoints", selectionButtonGroup);
    deselectButton->setText("Deselect Points");
    
    fileButtonGroup = new ui::ButtonGroup(selectionGroup, "FileButtonGroup");
    fileButtonGroup->enableDeselect(true);

    // Create an OSG drawable
    // Set up the geometry, vertex and color arrays, etc.
    // init dummy data
    plugin->LamureGroup = new osg::Group();
    plugin->LamureGroup->setName("LamureGroup");

    // Create an OSG Geode to hold the drawable
    plugin->geo = new osg::Geode();
    plugin->geo->setName("LamureGeode");

    plugin->pointSet = new PointSet[1];
    plugin->pointSet[0].colors = new Color[1024];
    plugin->pointSet[0].points = new ::Point[1024];
    plugin->pointSet[0].size = 1024;

    for (int n = 0; n < 1024; ++n)
    {
        plugin->pointSet[0].points[n].coordinates.x() = 1000 + n;
        plugin->pointSet[0].points[n].coordinates.y() = 1000 + n;
        plugin->pointSet[0].points[n].coordinates.z() = 1000 + n;

        plugin->pointSet[0].colors[n].r = n / 1024.0;
        plugin->pointSet[0].colors[n].g = (n + 500) / 2048.0;
        plugin->pointSet[0].colors[n].b = (n + 500) / 4096.0;
    }

    plugin->drawable = new LamureGeometry(&plugin->pointSet[0]);
    plugin->geo->addDrawable(plugin->drawable.get());

    
    // Set up Transformation
    plugin->transform = new osg::MatrixTransform();
    plugin->transform->addChild(plugin->geo);

    // Add Group to root
    plugin->LamureGroup->addChild(plugin->transform);
    cover->getObjectsRoot()->addChild(plugin->LamureGroup);

    //std::cout << covise::coConfigDefaultPaths::getDefaultTransformFileName() << std::endl;
    //std::cout << covise::coConfigDefaultPaths::getDefaultLocalConfigFileName() << std::endl;
    //std::cout << covise::coConfigDefaultPaths::getDefaultGlobalConfigFileName() << std::endl;

    //coCoviseConfig::ScopeEntries entries = coCoviseConfig::getScopeEntries("COVER.Plugin.LamurePointCloud");
    //std::cout << "getScopeEntries(): ";
    //std::cout << "[";
    //for (const auto& entry : entries)
    //{
    //    std::cout << entry.first.c_str();
    //    std::cout << ":";
    //    std::cout << entry.second.c_str();
    //}
    //std::cout << "]" << std::endl;

    plugin->lamureFileNode = coVRFileManager::instance()->loadFile(getConfigEntry("COVER.Plugin.LamurePointCloud").c_str());
    plugin->transform->addChild(plugin->lamureFileNode);

    VRSceneGraph::instance()->viewAll();
    VRViewer::instance()->statsDisplay->showStats(coVRStatsDisplay::VIEWER_SCENE_STATS, VRViewer::instance());
    
    return 1;
}

string LamurePointCloudPlugin::getConfigEntry(string scope) {
    std::cout << "getConfigEntry(): ";
    coCoviseConfig::ScopeEntries entries = coCoviseConfig::getScopeEntries(scope);
    for (const auto& entry : entries)
    {
        return entry.second;
    }
    return "";
}

string LamurePointCloudPlugin::getConfigEntry(string scope, string name) {
    std::cout << "getConfigEntry(): ";
    coCoviseConfig::ScopeEntries entries = coCoviseConfig::getScopeEntries(scope);
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


const LamurePointCloudPlugin* LamurePointCloudPlugin::instance() const
{
    return plugin;
}


static void GLClearError() 
{
    while (glGetError() != GL_NO_ERROR);
}


static bool GLLogError() 
{
    while (GLenum error = glGetError())
    {
        std::cout << "[OpenGL Error] (" << error << ")" << std::endl;
    }
    return true;
}


int LamurePointCloudPlugin::loadLMR(const char* filename, osg::Group* parent, const char* covise_key)
{
    printf("loadLMR()\n");

    const osg::GraphicsContext::Traits* traits = coVRConfig::instance()->windows[0].context->getTraits();
    /*std::cout << covise::coConfigDefaultPaths::getDefaultTransformFileName() << std::endl;
    std::cout << covise::coConfigDefaultPaths::getDefaultLocalConfigFileName() << std::endl;
    std::cout << covise::coConfigDefaultPaths::getDefaultGlobalConfigFileName() << std::endl;*/

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
        data_provenance_ = Data_Provenance::parse_json(settings_.json_);
        std::cout << "size of provenance: " << data_provenance_.get_size_in_bytes() << std::endl;
    }
    
    char str[200];
    sprintf(str, "COVER.WindowConfig.Window:%d", 0);

    printf("render_width_: %03" PRId32 "\n", traits->width);
    printf("render_height_: %03" PRId32 "\n", traits->height);

    lamure::ren::policy* policy = lamure::ren::policy::get_instance();
    policy->set_max_upload_budget_in_mb(settings_.upload_);
    policy->set_render_budget_in_mb(settings_.vram_);
    policy->set_out_of_core_budget_in_mb(settings_.ram_);
    policy->set_window_width(traits->width);
    policy->set_window_height(traits->height);
    
    model_database* database = model_database::get_instance();
    for (const auto& input_file : settings_.models_) {
        model_t model_id = database->add_model(input_file, std::to_string(num_models_));
        model_transformations_.push_back(settings_.transforms_[num_models_] * scm::math::mat4d(scm::math::make_translation(database->get_model(num_models_)->get_bvh()->get_translation())));
        ++num_models_;
    }

    std::cout << "(const char*)glGetString(GL_VERSION):" << std::endl;
    //std::cout << (GLubyte*)glGetString(GL_VERSION) << std::endl;

    printf("num_models: %i\n", num_models_);
    printf("GraphicsContext::getMaxContextID(): %i\n", GraphicsContext::getMaxContextID());

    std::cout << "osg::getGLVersionNumber(): " << osg::getGLVersionNumber() << std::endl;

    std::cout << "wglGetCurrentDC(): " << wglGetCurrentDC() << std::endl;
    std::cout << "wglGetCurrentContext(): " << wglGetCurrentContext() << std::endl;
    
    //std::cout << "OpenCOVER::instance()->parentWindow: " << OpenCOVER::instance()->parentWindow << std::endl;

    HWND hwnd_ = FindWindow(NULL, "COVER");
    std::cout << "FindWindow(NULL, 'COVER'): " << FindWindow(NULL, "COVER") << std::endl;

    HWND hwnd___ = FindWindow(NULL, "OpenCOVER");
    std::cout << "FindWindow(NULL, 'OpenCOVER'): " << FindWindow(NULL, "OpenCOVER") << std::endl;

    HDC hdc_covise = GetDC(FindWindow(NULL, "COVER"));
    HDC hdc_opencover = GetDC(FindWindow(NULL, "OpenCOVER"));


    //VRViewer::ViewerBase::Contexts contexts;
    //VRViewer::instance()->getContexts(contexts);

    //VRViewer::ViewerBase::Windows windows;
    //VRViewer::instance()->getWindows(windows);
    //bool currentornot = contexts[0]->makeCurrent();

    std::cout << "Vergleich der Windows: " << std::endl;
    std::cout << "WindowFromDC(GetDC(FindWindow(NULL, COVER)): " << WindowFromDC(GetDC(FindWindow(NULL, "COVER"))) << std::endl;
    std::cout << "WindowFromDC(GetDC(FindWindow(NULL, OpenCOVER))): " << WindowFromDC(GetDC(FindWindow(NULL, "OpenCOVER"))) << std::endl;
    std::cout << "WindowFromDC(wglGetCurrentDC()): " << WindowFromDC(wglGetCurrentDC()) << std::endl;

    std::cout << "Vergleich der DCs: " << std::endl;
    std::cout << "GetDC(FindWindow(NULL, COVER)): " << GetDC(FindWindow(NULL, "COVER")) << std::endl;
    std::cout << "GetDC(FindWindow(NULL, OpenCOVER)): " << GetDC(FindWindow(NULL, "OpenCOVER")) << std::endl;
    std::cout << "wglGetCurrentDC(): " << wglGetCurrentDC() << std::endl;
    

    //Rename window by using the handle to make sure it's the right handle
    //SetWindowText(hWnd__, "NEW_PROCESS_WINDOW_TEXT.c_str()");

    // other approaches
    // HWND hwnd_ = GetDesktopWindow();
    // HWND hwnd_ = WindowFromDC(hdc__);

    //VRViewer::instance()->getDisplaySettings();

    //osgViewer::GraphicsWindowWin32::getWindowName;
    //osgViewer::GraphicsWindowWin32::getWGLContext;
    //osgViewer::GraphicsWindowWin32::getHWND;
    //osgViewer::GraphicsWindowWin32::getHDC;

    //std::cout << (*device_);
    //std::cout << "device_->device_context_version():" << std::endl;
    //std::cout << device_->device_context_version() << std::endl;

    //std::cout << "context_ = device_->create_context():" << std::endl;
    //context_ = device_->create_context();

    //std::cout << "device_->device_vendor(): ";
    //std::cout << device_->device_vendor() << std::endl;

    //std::cout << "device_->device_renderer(): ";
    //std::cout << device_->device_renderer() << std::endl;

    //std::cout << "device_->device_shader_compiler(): ";
    //std::cout << device_->device_shader_compiler() << std::endl;

    //std::cout << device_->device_context_version();
    //std::cout << device_->device_context_version() << std::endl;

    //std::cout << "(*(plugin->device_))" << std::endl;
    //std::cout << (*device_);

    //std::cout << (*device_);
    //std::cout << opencover::coVRConfig::instance()->glVersion << std::endl;

    //device_.reset(new scm::gl::render_device());
    //if (!device_) {
    //    std::cout << "error creating device" << std::endl;
    //}
    //context_ = device_->main_context();

    //if (!context_) {
    //    std::cout << "error creating context" << std::endl;
    //}

    //plugin->init_lamure_shader();
    //plugin->create_framebuffers();
    //plugin->create_aux_resources();
    //plugin->init_render_states();
    //plugin->init_camera();
    //plugin->lamure_display();


    //SwapBuffers(hdc);

    //HWND hwnd = FindWindow(NULL, "COVER");
    //HDC hdc = GetDC(hwnd);
    //HGLRC hglrc = wglCreateContext(hdc);
    //wglMakeCurrent(hdc, hglrc);


    
    
    return 1;
}


void LamurePointCloudPlugin::preFrame()
{   
    
    /*VRViewer::ViewerBase::Contexts contexts;
    VRViewer::instance()->getContexts(contexts);

    VRViewer::ViewerBase::Windows windows;
    VRViewer::instance()->getWindows(windows);
    bool a_is_already_current = contexts[0]->isCurrent();*/
    //bool a_made_current_successfully = contexts[0]->makeCurrent();

    if (cover->getPointerButton()->getState() == 1) {
    //    //plugin->lamure_display();
    //    for (int v = 0; v < context_->current_viewports().size(); v++) {
    //        std::cout << "viewport.position: " << context_->current_viewports().viewports()[v]._position[0] << "  " << context_->current_viewports().viewports()[v]._position[1] << std::endl;
    //        std::cout << "viewport.dimension: " << context_->current_viewports().viewports()[v]._dimensions[0] << "  " << context_->current_viewports().viewports()[v]._dimensions[1] << std::endl;
    //        std::cout << "viewport.depth_range: " << context_->current_viewports().viewports()[v]._depth_range[0] << "  " << context_->current_viewports().viewports()[v]._depth_range[1] << std::endl;
    //        std::cout << " " << std::endl;
    //    };

        //osg::Matrixf m = cover->getViewerMat();
        //std::cout << m(0, 0) << " " << m(0, 1) << " " << m(0, 2) << " " << m(0, 3) << std::endl;
        //std::cout << m(1, 0) << " " << m(1, 1) << " " << m(1, 2) << " " << m(1, 3) << std::endl;
        //std::cout << m(2, 0) << " " << m(2, 1) << " " << m(2, 2) << " " << m(2, 3) << std::endl;
        //std::cout << m(3, 0) << " " << m(3, 1) << " " << m(3, 2) << " " << m(3, 3) << std::endl;

        //osg::Vec3f t = (cover->getViewerMat()).getTrans();
        //std::cout << "" << std::endl;


        VRViewer::ViewerBase* viewer = VRViewer::instance();

        std::string font = coVRFileManager::instance()->getFontFile(NULL);

        // collect all the relevant cameras
        osgViewer::ViewerBase::Cameras validCameras;
        viewer->getCameras(validCameras);

        osgViewer::ViewerBase::Cameras cameras;
        for (osgViewer::ViewerBase::Cameras::iterator itr = validCameras.begin();
            itr != validCameras.end();
            ++itr)
        {
            if ((*itr)->getStats())
            {
                cameras.push_back(*itr);
            }
        }

        const osg::GraphicsContext::Traits* traits = coVRConfig::instance()->windows[0].context->getTraits();

        if (traits->doubleBuffer)
        {
            glDrawBuffer(GL_FRONT);
        }

        glClearColor(0.5, 0.5, 0.5, 1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (traits->doubleBuffer)
        {
            glDrawBuffer(GL_BACK);
            glClearColor(0.5, 0.5, 0.5, 1.0);
            glClear(GL_COLOR_BUFFER_BIT);
        }


        
    };

    //osgViewer::ViewerBase::Contexts contexts;
    //VRViewer::instance()->getContexts(contexts);

    //osgViewer::ViewerBase::Contexts::iterator citr;
    //osg::GraphicsContext* gc = (*citr);

    //gc->makecurrent();

    //VRViewer::instance()->clearWindow = false;



    //if (traits->doubleBuffer)
    //{
    //    glDrawBuffer(GL_FRONT);
    //}
    //glClearColor(0.0, 0.0, 0.0, 1.0);
    //glClear(GL_COLOR_BUFFER_BIT);

    //float positions[6] = { 
    //    -0.5f, -0.5f, 
    //    0.0f, 0.5f,
    //    0.5f, -0.5f 
    //};
    //unsigned int buffer;
    //glGenBuffers(1, &buffer);
    //glBindBuffer(GL_ARRAY_BUFFER, buffer);
    //glBufferData(GL_ARRAY_BUFFER, 6 * sizeof(float), positions, GL_STATIC_DRAW);


    //glClear(GL_COLOR_BUFFER_BIT);
    //glBegin(GL_TRIANGLES);
    //glVertex2f(-0.5f, -0.5f);
    //glVertex2f(0.0f, 0.5f);
    //glVertex2f(0.5f, -0.5f);
    //glEnd();
    

    //pointShader->apply(plugin->geo, plugin->drawable);

    //// Get the vertex array from the geometry object
    //osg::ref_ptr<osg::Vec3Array> vertices = dynamic_cast<osg::Vec3Array*>(plugin->drawable->getVertexArray());
    //osg::ref_ptr<osg::Vec3Array> colors = dynamic_cast<osg::Vec3Array*>(plugin->drawable->getColorArray());
    //vertices->dirty();
    //colors->dirty();
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


void LamurePointCloudPlugin::load_settings(std::string const& filename) {
    std::cout << filename << std::endl;
    std::ifstream lmr_file(filename.c_str());
    if (!lmr_file.is_open()) {
        std::cout << "could not open lmr file" << std::endl;
        exit(-1);
    }
    else {
        model_t model_id = 0;
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
                        settings_.max_radius_ = std::max(atof(value.c_str()), 0.0);
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


void lines_from_min_max(
    const scm::math::vec3f& min_vertex,
    const scm::math::vec3f& max_vertex,
    std::vector<scm::math::vec3f>& lines) {

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


float get_atlas_scale_factor() {
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
        std::string vis_quad_vs_source;
        std::string vis_quad_fs_source;
        std::string vis_line_vs_source;
        std::string vis_line_fs_source;

        std::string vis_triangle_vs_source;
        std::string vis_triangle_fs_source;

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

        if (!read_shader(shader_root_path + "/vis/vis_quad.glslv", vis_quad_vs_source)
            || !read_shader(shader_root_path + "/vis/vis_quad.glslf", vis_quad_fs_source)
            || !read_shader(shader_root_path + "/vis/vis_line.glslv", vis_line_vs_source)
            || !read_shader(shader_root_path + "/vis/vis_line.glslf", vis_line_fs_source)
            || !read_shader(shader_root_path + "/vis/vis_triangle.glslv", vis_triangle_vs_source)
            || !read_shader(shader_root_path + "/vis/vis_triangle.glslf", vis_triangle_fs_source)

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

        /*auto myshader = device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_quad_vs_source);
        scm::gl::shader& obj = *myshader;
        std::cout << obj << std::endl;*/

        vis_quad_shader_ = device_->create_program(
            boost::assign::list_of
            (device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_quad_vs_source))
            (device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_quad_fs_source)));

        if (!vis_quad_shader_) {
            std::cout << "error creating shader vis_quad_shader_ program" << std::endl;
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
            std::cout << "error creating shader vis_vt_shader_ program" << std::endl;
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


void LamurePointCloudPlugin::create_framebuffers() {
    const osg::GraphicsContext::Traits* traits = NULL;
    traits = coVRConfig::instance()->windows[0].context->getTraits();

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
    pass1_depth_buffer_ = device_->create_texture_2d(scm::math::vec2ui(traits->width, traits->height), scm::gl::FORMAT_D24, 1, 1, 1);
    pass1_fbo_->attach_depth_stencil_buffer(pass1_depth_buffer_);

    pass2_fbo_ = device_->create_frame_buffer();
    pass2_color_buffer_ = device_->create_texture_2d(scm::math::vec2ui(traits->width, traits->height), scm::gl::FORMAT_RGBA_32F, 1, 1, 1);
    pass2_fbo_->attach_color_buffer(0, pass2_color_buffer_);
    pass2_fbo_->attach_depth_stencil_buffer(pass1_depth_buffer_);

    pass2_normal_buffer_ = device_->create_texture_2d(scm::math::vec2ui(traits->width, traits->height), scm::gl::FORMAT_RGB_32F, 1, 1, 1);
    pass2_fbo_->attach_color_buffer(1, pass2_normal_buffer_);
    pass2_view_space_pos_buffer_ = device_->create_texture_2d(scm::math::vec2ui(traits->width, traits->height), scm::gl::FORMAT_RGB_32F, 1, 1, 1);
    pass2_fbo_->attach_color_buffer(2, pass2_view_space_pos_buffer_);
}


void LamurePointCloudPlugin::init_render_states() {

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

    vt_filter_linear_ = device_->create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);
    vt_filter_nearest_ = device_->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);
}

void LamurePointCloudPlugin::init_camera() {

    const osg::GraphicsContext::Traits* traits = NULL;
    traits = coVRConfig::instance()->windows[0].context->getTraits();


    auto root_bb = model_database::get_instance()->get_model(0)->get_bvh()->get_bounding_boxes()[0];
    auto root_bb_min = scm::math::mat4f(model_transformations_[0]) * root_bb.min_vertex();
    auto root_bb_max = scm::math::mat4f(model_transformations_[0]) * root_bb.max_vertex();
    scm::math::vec3f center = (root_bb_min + root_bb_max) / 2.f;

    camera_ = new camera(0,
        make_look_at_matrix(center + scm::math::vec3f(0.f, 0.1f, -0.01f), center, scm::math::vec3f(0.f, 1.f, 0.f)),
        length(root_bb_max - root_bb_min), false, false);
    camera_->set_dolly_sens_(settings_.travel_speed_);

    if (settings_.use_view_tf_) {
        camera_->set_view_matrix(settings_.view_tf_);
        std::cout << "view_tf:" << std::endl;
        std::cout << camera_->get_high_precision_view_matrix() << std::endl;
        camera_->set_dolly_sens_(settings_.travel_speed_);
    }

    camera_->set_projection_matrix(settings_.fov_, float(traits->width) / float(traits->height), settings_.near_plane_, settings_.far_plane_);

    screen_quad_.reset(new scm::gl::quad_geometry(device_, scm::math::vec2f(-1.0f, -1.0f), scm::math::vec2f(1.0f, 1.0f)));

    gui_.ortho_matrix_ = scm::math::make_ortho_matrix(0.0f, static_cast<float>(traits->width),
        0.0f, static_cast<float>(traits->height), -1.0f, 1.0f);
}



void LamurePointCloudPlugin::set_uniforms(scm::gl::program_ptr shader) {
    const osg::GraphicsContext::Traits* traits = NULL;
    traits = coVRConfig::instance()->windows[0].context->getTraits();

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


void LamurePointCloudPlugin::draw_all_models(const context_t context_id, const view_t view_id, scm::gl::program_ptr shader) {

    const osg::GraphicsContext::Traits* traits = NULL;
    traits = coVRConfig::instance()->windows[0].context->getTraits();

    controller* controller = controller::get_instance();
    cut_database* cuts = cut_database::get_instance();
    model_database* database = model_database::get_instance();
    lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();

    if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
        context_->bind_vertex_array(
            controller->get_context_memory(context_id, bvh::primitive_type::POINTCLOUD, device_, data_provenance_));
    }
    else {
        context_->bind_vertex_array(
            controller->get_context_memory(context_id, bvh::primitive_type::POINTCLOUD, device_));
    }
    context_->apply();

    for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
        if (selection_.selected_model_ != -1) {
            model_id = selection_.selected_model_;
        }
        bool draw = true;
        if (settings_.show_sparse_ && sparse_resources_[model_id].num_primitives_ > 0) {
            if (selection_.selected_model_ != -1) break;
            //else continue; //don't show lod when sparse is already shown
            else draw = false;
        }
        model_t m_id = controller->deduce_model_id(std::to_string(model_id));
        cut& cut = cuts->get_cut(context_id, view_id, m_id);
        std::vector<cut::node_slot_aggregate> renderable = cut.complete_set();
        const bvh* bvh = database->get_model(m_id)->get_bvh();
        if (bvh->get_primitive() != bvh::primitive_type::POINTCLOUD) {
            if (selection_.selected_model_ != -1) break;
            //else continue;
            else draw = false;
        }

        //uniforms per model
        scm::math::mat4d model_matrix = model_transformations_[model_id];
        scm::math::mat4d projection_matrix = scm::math::mat4d(camera_->get_projection_matrix());
        scm::math::mat4d view_matrix = camera_->get_high_precision_view_matrix();
        scm::math::mat4d model_view_matrix = view_matrix * model_matrix;
        scm::math::mat4d model_view_projection_matrix = projection_matrix * model_view_matrix;

        shader->uniform("mvp_matrix", scm::math::mat4f(model_view_projection_matrix));
        shader->uniform("model_matrix", scm::math::mat4f(model_matrix));
        shader->uniform("model_view_matrix", scm::math::mat4f(model_view_matrix));
        shader->uniform("inv_mv_matrix", scm::math::mat4f(scm::math::transpose(scm::math::inverse(model_view_matrix))));

        const scm::math::mat4d viewport_scale = scm::math::make_scale(traits->width * 0.5, traits->width * 0.5, 0.5);
        const scm::math::mat4d viewport_translate = scm::math::make_translation(1.0, 1.0, 1.0);
        const scm::math::mat4d model_to_screen = viewport_scale * viewport_translate * model_view_projection_matrix;
        shader->uniform("model_to_screen_matrix", scm::math::mat4f(model_to_screen));

        //scm::math::vec4d x_unit_vec = scm::math::vec4d(1.0,0.0,0.0,0.0);
        //float model_radius_scale = scm::math::length(scm::math::vec3d(model_matrix * x_unit_vec));
        //shader->uniform("model_radius_scale", model_radius_scale);
        shader->uniform("model_radius_scale", 1.f);

        size_t surfels_per_node = database->get_primitives_per_node();
        std::vector<scm::gl::boxf>const& bounding_box_vector = bvh->get_bounding_boxes();

        scm::gl::frustum frustum_by_model = camera_->get_frustum_by_model(scm::math::mat4f(model_matrix));

        for (auto const& node_slot_aggregate : renderable) {
            uint32_t node_culling_result = camera_->cull_against_frustum(
                frustum_by_model,
                bounding_box_vector[node_slot_aggregate.node_id_]);

            if (node_culling_result != 1) {
                
                /*if (settings_.use_pvs_ && pvs->is_activated() && settings_.pvs_culling_
                    && !lamure::pvs::pvs_database::get_instance()->get_viewer_visibility(model_id, node_slot_aggregate.node_id_)) {
                    continue;
                }*/
                
                if (settings_.show_accuracy_) {
                    const float accuracy = 1.0 - (bvh->get_depth_of_node(node_slot_aggregate.node_id_) * 1.0) / (bvh->get_depth() - 1);
                    shader->uniform("accuracy", accuracy);
                }
                if (settings_.show_radius_deviation_) {
                    shader->uniform("average_radius", bvh->get_avg_primitive_extent(node_slot_aggregate.node_id_));
                }

                context_->apply();

                if (draw) {
                    context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST,
                        (node_slot_aggregate.slot_id_) * (GLsizei)surfels_per_node, surfels_per_node);
                    rendered_splats_ += surfels_per_node;
                    ++rendered_nodes_;
                }
            }
        }
        if (selection_.selected_model_ != -1) {
            break;
        }
    }
}


void LamurePointCloudPlugin::draw_brush(scm::gl::program_ptr shader) {

    if (selection_.brush_end_ > 0) {
        set_uniforms(shader);

        scm::math::mat4d model_matrix = scm::math::mat4d::identity();
        scm::math::mat4d projection_matrix = scm::math::mat4d(camera_->get_projection_matrix());
        scm::math::mat4d view_matrix = camera_->get_high_precision_view_matrix();
        scm::math::mat4d model_view_matrix = view_matrix * model_matrix;
        scm::math::mat4d model_view_projection_matrix = projection_matrix * model_view_matrix;

        shader->uniform("mvp_matrix", scm::math::mat4f(model_view_projection_matrix));
        shader->uniform("model_matrix", scm::math::mat4f(model_matrix));
        shader->uniform("model_view_matrix", scm::math::mat4f(model_view_matrix));
        shader->uniform("inv_mv_matrix", scm::math::mat4f(scm::math::transpose(scm::math::inverse(model_view_matrix))));

        shader->uniform("point_size_factor", settings_.aux_point_scale_);

        shader->uniform("model_to_screen_matrix", scm::math::mat4f::identity());
        shader->uniform("model_radius_scale", 1.f);

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


void apply_vt_cut_update() {

    auto* cut_db = &vt::CutDatabase::get_instance();

    for (vt::cut_map_entry_type cut_entry : (*cut_db->get_cut_map())) {
        vt::Cut* cut = cut_db->start_reading_cut(cut_entry.first);

        if (!cut->is_drawn()) {
            cut_db->stop_reading_cut(cut_entry.first);
            continue;
        }

        std::set<uint16_t> updated_levels;

        for (auto position_slot_updated : cut->get_front()->get_mem_slots_updated()) {
            const vt::mem_slot_type* mem_slot_updated = cut_db->read_mem_slot_at(position_slot_updated.second);

            if (mem_slot_updated == nullptr || !mem_slot_updated->updated
                || !mem_slot_updated->locked || mem_slot_updated->pointer == nullptr) {
                if (mem_slot_updated == nullptr) {
                    std::cerr << "Mem slot at " << position_slot_updated.second << " is null" << std::endl;
                }
                else {
                    std::cerr << "Mem slot at " << position_slot_updated.second << std::endl;
                    std::cerr << "Mem slot #" << mem_slot_updated->position << std::endl;
                    std::cerr << "Tile id: " << mem_slot_updated->tile_id << std::endl;
                    std::cerr << "Locked: " << mem_slot_updated->locked << std::endl;
                    std::cerr << "Updated: " << mem_slot_updated->updated << std::endl;
                    std::cerr << "Pointer valid: " << (mem_slot_updated->pointer != nullptr) << std::endl;
                }

                throw std::runtime_error("updated mem slot inconsistency");
            }

            updated_levels.insert(vt::QuadTree::get_depth_of_node(mem_slot_updated->tile_id));

            // update_physical_texture_blockwise
            size_t slots_per_texture = vt::VTConfig::get_instance().get_phys_tex_tile_width() *
                vt::VTConfig::get_instance().get_phys_tex_tile_width();
            size_t layer = mem_slot_updated->position / slots_per_texture;
            size_t rel_slot_position = mem_slot_updated->position - layer * slots_per_texture;

            size_t x_tile = rel_slot_position % vt::VTConfig::get_instance().get_phys_tex_tile_width();
            size_t y_tile = rel_slot_position / vt::VTConfig::get_instance().get_phys_tex_tile_width();

            scm::math::vec3ui origin = scm::math::vec3ui(
                (uint32_t)x_tile * vt::VTConfig::get_instance().get_size_tile(),
                (uint32_t)y_tile * vt::VTConfig::get_instance().get_size_tile(), (uint32_t)layer);
            scm::math::vec3ui dimensions = scm::math::vec3ui(vt::VTConfig::get_instance().get_size_tile(),
                vt::VTConfig::get_instance().get_size_tile(), 1);

            context_->update_sub_texture(vt_.physical_texture_, scm::gl::texture_region(origin, dimensions), 0,
                get_tex_format(), mem_slot_updated->pointer);
        }


        for (auto position_slot_cleared : cut->get_front()->get_mem_slots_cleared()) {
            const vt::mem_slot_type* mem_slot_cleared = cut_db->read_mem_slot_at(position_slot_cleared.second);

            if (mem_slot_cleared == nullptr) {
                std::cerr << "Mem slot at " << position_slot_cleared.second << " is null" << std::endl;
            }

            updated_levels.insert(vt::QuadTree::get_depth_of_node(position_slot_cleared.first));
        }

        // update_index_texture
        for (uint16_t updated_level : updated_levels) {
            uint32_t size_index_texture = (uint32_t)vt::QuadTree::get_tiles_per_row(updated_level);

            scm::math::vec3ui origin = scm::math::vec3ui(0, 0, 0);
            scm::math::vec3ui dimensions = scm::math::vec3ui(size_index_texture, size_index_texture, 1);

            context_->update_sub_texture(vt_.index_texture_hierarchy_.at(updated_level),
                scm::gl::texture_region(origin, dimensions), 0, scm::gl::FORMAT_RGBA_8UI,
                cut->get_front()->get_index(updated_level));

        }

        cut_db->stop_reading_cut(cut_entry.first);
    }
    context_->sync();
}


void LamurePointCloudPlugin::draw_resources(const context_t context_id, const view_t view_id) {
    const osg::GraphicsContext::Traits* traits = NULL;
    traits = coVRConfig::instance()->windows[0].context->getTraits();

    if (sparse_resources_.size() > 0) {
        if ((settings_.show_sparse_ || settings_.show_views_) && sparse_resources_.size() > 0) {

            context_->bind_program(vis_xyz_shader_);
            context_->set_blend_state(color_no_blending_state_);
            context_->set_depth_stencil_state(depth_state_less_);

            set_uniforms(vis_xyz_shader_);

            scm::math::mat4d model_matrix = scm::math::mat4d::identity();
            scm::math::mat4d projection_matrix = scm::math::mat4d(camera_->get_projection_matrix());
            scm::math::mat4d view_matrix = camera_->get_high_precision_view_matrix();
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

            for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
                if (selection_.selected_model_ != -1) {
                    model_id = selection_.selected_model_;
                }

                auto s_res = sparse_resources_[model_id];
                if (s_res.num_primitives_ > 0) {
                    context_->bind_vertex_array(s_res.array_);
                    context_->apply();

                    uint32_t num_views = provenance_[model_id].num_views_;

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
                        context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, num_views,
                            s_res.num_primitives_ - num_views);
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

            uint64_t color_cut_id =
                (((uint64_t)vt_.texture_id_) << 32) | ((uint64_t)vt_.view_id_ << 16) | ((uint64_t)vt_.context_id_);
            //uint32_t max_depth_level_color = (*vt::CutDatabase::get_instance().get_cut_map())[color_cut_id]->get_atlas()->getDepth() - 1;

            scm::math::mat4f view_matrix = camera_->get_view_matrix();
            scm::math::mat4f projection_matrix = scm::math::mat4f(camera_->get_projection_matrix());

            vis_vt_shader_->uniform("model_view_matrix", view_matrix);
            vis_vt_shader_->uniform("projection_matrix", projection_matrix);

            vis_vt_shader_->uniform("physical_texture_dim", vt_.physical_texture_size_);
            //vis_vt_shader_->uniform("max_level", max_depth_level_color);
            vis_vt_shader_->uniform("tile_size", scm::math::vec2((uint32_t)vt::VTConfig::get_instance().get_size_tile()));
            vis_vt_shader_->uniform("tile_padding", scm::math::vec2((uint32_t)vt::VTConfig::get_instance().get_size_padding()));

            vis_vt_shader_->uniform("enable_hierarchy", vt_.enable_hierarchy_);
            vis_vt_shader_->uniform("toggle_visualization", vt_.toggle_visualization_);

            for (uint32_t i = 0; i < vt_.index_texture_hierarchy_.size(); ++i) {
                std::string texture_string = "hierarchical_idx_textures";
                vis_vt_shader_->uniform(texture_string, i, int((i)));
            }

            vis_vt_shader_->uniform("physical_texture_array", 17);

            //context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(render_width_, render_height_)));
            context_->set_depth_stencil_state(depth_state_less_);
            context_->set_rasterizer_state(no_backface_culling_rasterizer_state_);
            context_->set_blend_state(color_no_blending_state_);
            context_->sync();

            apply_vt_cut_update();

            for (uint16_t i = 0; i < vt_.index_texture_hierarchy_.size(); ++i) {
                context_->bind_texture(vt_.index_texture_hierarchy_.at(i), vt_filter_nearest_, i);
            }
            context_->bind_texture(vt_.physical_texture_, vt_filter_linear_, 17);
            context_->bind_storage_buffer(vt_.feedback_lod_storage_, 0);
            context_->bind_storage_buffer(vt_.feedback_count_storage_, 1);
            context_->apply();

            for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
                if (selection_.selected_model_ != -1) {
                    model_id = selection_.selected_model_;
                }
                auto t_res = image_plane_resources_[model_id];
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
        }

        if (settings_.show_views_ || settings_.show_octrees_) {
            context_->bind_program(vis_line_shader_);

            scm::math::mat4f projection_matrix = scm::math::mat4f(camera_->get_projection_matrix());
            scm::math::mat4f view_matrix = camera_->get_view_matrix();
            vis_line_shader_->uniform("model_matrix", scm::math::mat4f::identity());
            vis_line_shader_->uniform("view_matrix", view_matrix);
            vis_line_shader_->uniform("projection_matrix", projection_matrix);

            for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
                if (selection_.selected_model_ != -1) {
                    model_id = selection_.selected_model_;
                }

                if (settings_.show_views_) {
                    uint32_t num_views = provenance_[model_id].num_views_;
                    auto f_res = frusta_resources_[model_id];
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
                    auto o_res = octree_resources_[model_id];
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

    if (settings_.show_bvhs_) {

        controller* controller = controller::get_instance();
        cut_database* cuts = cut_database::get_instance();
        model_database* database = model_database::get_instance();
        //lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();

        context_->bind_program(vis_line_shader_);

        scm::math::mat4f projection_matrix = scm::math::mat4f(camera_->get_projection_matrix());
        scm::math::mat4f view_matrix = camera_->get_view_matrix();

        vis_line_shader_->uniform("view_matrix", view_matrix);
        vis_line_shader_->uniform("projection_matrix", projection_matrix);

        for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
            if (selection_.selected_model_ != -1) {
                model_id = selection_.selected_model_;
            }

            bool draw = true;
            model_t m_id = controller->deduce_model_id(std::to_string(model_id));
            cut& cut = cuts->get_cut(context_id, view_id, m_id);
            std::vector<cut::node_slot_aggregate> renderable = cut.complete_set();
            const bvh* bvh = database->get_model(m_id)->get_bvh();
            if (bvh->get_primitive() != bvh::primitive_type::POINTCLOUD) {
                if (selection_.selected_model_ != -1) break;
                else draw = false;
            }

            if (draw) {
                //uniforms per model
                scm::math::mat4d model_matrix = model_transformations_[model_id];
                vis_line_shader_->uniform("model_matrix", scm::math::mat4f(model_matrix));
                std::vector<scm::gl::boxf>const& bounding_box_vector = bvh->get_bounding_boxes();
                scm::gl::frustum frustum_by_model = camera_->get_frustum_by_model(scm::math::mat4f(model_matrix));
                auto bvh_res = bvh_resources_[model_id];
                if (bvh_res.num_primitives_ > 0) {
                    context_->bind_vertex_array(bvh_res.array_);
                    context_->apply();

                    for (auto const& node_slot_aggregate : renderable) {
                        uint32_t node_culling_result = camera_->cull_against_frustum(
                            frustum_by_model,
                            bounding_box_vector[node_slot_aggregate.node_id_]);

                        if (node_culling_result != 1) {
                            
                            /*if (settings_.use_pvs_ && pvs->is_activated() && settings_.pvs_culling_ && !lamure::pvs::pvs_database::get_instance()->get_viewer_visibility(model_id, node_slot_aggregate.node_id_)) {
                                continue;
                            }*/
                            
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

    if (settings_.pvs_ != "" && settings_.show_pvs_) {
        if (pvs_resource_.num_primitives_ > 0) {
            context_->bind_program(vis_line_shader_);

            scm::math::mat4f projection_matrix = scm::math::mat4f(camera_->get_projection_matrix());
            scm::math::mat4f view_matrix = camera_->get_view_matrix();
            vis_line_shader_->uniform("model_matrix", scm::math::mat4f::identity());
            vis_line_shader_->uniform("view_matrix", view_matrix);
            vis_line_shader_->uniform("projection_matrix", projection_matrix);

            context_->bind_vertex_array(pvs_resource_.array_);
            context_->apply();
            context_->draw_arrays(scm::gl::PRIMITIVE_LINE_LIST, 0, pvs_resource_.num_primitives_);
        }
    }
}

void LamurePointCloudPlugin::covise_display() {
    const osg::GraphicsContext::Traits* traits = NULL;
    traits = coVRConfig::instance()->windows[0].context->getTraits();

    std::cout << "covise_display()" << std::endl;
    if (rendering_) {
        return;
    }
    rendering_ = true;

    std::cout << "camera_->get_projection_matrix(): " << std::endl;
    std::cout << camera_->get_projection_matrix() << "\n" << std::endl;

    camera_->set_projection_matrix(settings_.fov_, float(settings_.width_) / float(settings_.height_), settings_.near_plane_, settings_.far_plane_);

    std::cout << "settings_.transforms_[0] " << std::endl;
    std::cout << settings_.transforms_[0] << "\n" << std::endl;

    std::cout << "camera_->get_cam_matrix(): " << std::endl;
    std::cout << camera_->get_cam_matrix() << "\n" << std::endl;

    std::cout << "camera_->get_projection_matrix(): " << std::endl;
    std::cout << camera_->get_projection_matrix() << "\n" << std::endl;

    std::cout << "camera_->get_view_matrix(): " << std::endl;
    std::cout << camera_->get_view_matrix() << "\n" << std::endl;

    std::cout << "camera_->trackball_matrix(): " << std::endl;
    std::cout << camera_->trackball_matrix() << "\n" << std::endl;

    model_database* database = model_database::get_instance();
    dataset* ds = database->get_model(0);
    const bvh* bvh = ds->get_bvh();

    std::cout << "bvh->get_depth(): " << bvh->get_depth() << std::endl;
    for (int i = 0; i < bvh->get_depth(); i++) {
        std::cout << i << ": " << bvh->get_length_of_depth(i) << std::endl;
    };
    std::cout << "" << std::endl;

    cut_database* cuts = cut_database::get_instance();
    controller* controller = controller::get_instance();
    //lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();

    if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
        controller->reset_system(data_provenance_);
    }
    else {
        controller->reset_system();
    }

    context_t context_id = controller->deduce_context_id(0);
    for (model_t model_id = 0; model_id < settings_.models_.size(); ++model_id) {
        model_t m_id = controller->deduce_model_id(std::to_string(model_id));
        cuts->send_transform(context_id, m_id, scm::math::mat4f(model_transformations_[m_id]));
        cuts->send_threshold(context_id, m_id, settings_.lod_error_);
        cuts->send_rendered(context_id, m_id);
        database->get_model(m_id)->set_transform(scm::math::mat4f(model_transformations_[m_id]));
    }

    view_t cam_id = controller->deduce_view_id(context_id, camera_->view_id());
    cuts->send_camera(context_id, cam_id, *camera_);
    std::vector<scm::math::vec3d> corner_values = camera_->get_frustum_corners();
    double top_minus_bottom = scm::math::length((corner_values[2]) - (corner_values[0]));
    height_divided_by_top_minus_bottom_ = lamure::ren::policy::get_instance()->window_height() / top_minus_bottom;

    std::cout << "top_minus_bottom: " << top_minus_bottom << std::endl;
    std::cout << "height_divided_by_top_minus_bottom_: " << height_divided_by_top_minus_bottom_ << std::endl;
    std::cout << "corner_values.size(): " << corner_values.size() << std::endl;

    for (int iter = 0; iter < corner_values.size(); ++iter) {
        std::cout << "corner_values.at(" << iter << "): " << corner_values.at(iter) << std::endl;
    }
    std::cout << "" << std::endl;

    printf("corner_values[1] - corner_values[0]: %f \n", scm::math::length((corner_values[1]) - (corner_values[0])));
    printf("corner_values[2] - corner_values[0]: %f \n", scm::math::length((corner_values[2]) - (corner_values[0])));
    printf("corner_values[3] - corner_values[0]: %f \n", scm::math::length((corner_values[3]) - (corner_values[0])));

    cuts->send_height_divided_by_top_minus_bottom(context_id, cam_id, height_divided_by_top_minus_bottom_);

    if (settings_.use_pvs_) {
        scm::math::mat4f cm = scm::math::inverse(scm::math::mat4f(camera_->trackball_matrix()));
        scm::math::vec3d cam_pos = scm::math::vec3d(cm[12], cm[13], cm[14]);
        //pvs->set_viewer_position(cam_pos);
    }

    if (settings_.lod_update_) {
        if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
            controller->dispatch(context_id, device_, data_provenance_);
        }
        else {
            controller->dispatch(context_id, device_);
        }
    }
    view_t view_id = controller->deduce_view_id(context_id, camera_->view_id());
    context_->set_rasterizer_state(no_backface_culling_rasterizer_state_);

    context_->clear_depth_stencil_buffer(pass1_fbo_);
    context_->set_frame_buffer(pass1_fbo_);

    context_->bind_program(vis_xyz_pass1_shader_);
    context_->set_blend_state(color_no_blending_state_);
    context_->set_depth_stencil_state(depth_state_less_);

    set_uniforms(vis_xyz_pass1_shader_);

   
    context_->apply();
    
    co_draw_all_models(context_id, view_id, vis_xyz_pass1_shader_);
    //draw_brush(vis_xyz_pass1_shader_);
    rendering_ = false;
}

void LamurePointCloudPlugin::co_draw_all_models(const context_t context_id, const view_t view_id, scm::gl::program_ptr shader) {
    const osg::GraphicsContext::Traits* traits = NULL;
    traits = coVRConfig::instance()->windows[0].context->getTraits();

    std::cout << "co_draw_all_models()" << std::endl;

    controller* controller = controller::get_instance();
    cut_database* cuts = cut_database::get_instance();
    model_database* database = model_database::get_instance();
    //lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();

    if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
        context_->bind_vertex_array(
            controller->get_context_memory(context_id, bvh::primitive_type::POINTCLOUD, device_, data_provenance_));
    }
    else {
        context_->bind_vertex_array(
            controller->get_context_memory(context_id, bvh::primitive_type::POINTCLOUD, device_));
    }
    context_->apply();

    for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
        if (selection_.selected_model_ != -1) {
            model_id = selection_.selected_model_;
        }
        bool draw = true;
        if (settings_.show_sparse_ && sparse_resources_[model_id].num_primitives_ > 0) {
            if (selection_.selected_model_ != -1) break;
            //else continue; //don't show lod when sparse is already shown
            else draw = false;
        }
        model_t m_id = controller->deduce_model_id(std::to_string(model_id));
        cut& cut = cuts->get_cut(context_id, view_id, m_id);
        std::vector<cut::node_slot_aggregate> renderable = cut.complete_set();
        const bvh* bvh = database->get_model(m_id)->get_bvh();
        if (bvh->get_primitive() != bvh::primitive_type::POINTCLOUD) {
            if (selection_.selected_model_ != -1) break;
            //else continue;
            else draw = false;
        }

        //uniforms per model
        scm::math::mat4d model_matrix = model_transformations_[model_id];
        scm::math::mat4d projection_matrix = scm::math::mat4d(camera_->get_projection_matrix());
        scm::math::mat4d view_matrix = camera_->get_high_precision_view_matrix();
        scm::math::mat4d model_view_matrix = view_matrix * model_matrix;
        scm::math::mat4d model_view_projection_matrix = projection_matrix * model_view_matrix;

        shader->uniform("mvp_matrix", scm::math::mat4f(model_view_projection_matrix));
        shader->uniform("model_matrix", scm::math::mat4f(model_matrix));
        shader->uniform("model_view_matrix", scm::math::mat4f(model_view_matrix));
        shader->uniform("inv_mv_matrix", scm::math::mat4f(scm::math::transpose(scm::math::inverse(model_view_matrix))));

        const scm::math::mat4d viewport_scale = scm::math::make_scale(traits->width * 0.5, traits->width * 0.5, 0.5);
        const scm::math::mat4d viewport_translate = scm::math::make_translation(1.0, 1.0, 1.0);
        const scm::math::mat4d model_to_screen = viewport_scale * viewport_translate * model_view_projection_matrix;
        shader->uniform("model_to_screen_matrix", scm::math::mat4f(model_to_screen));

        //scm::math::vec4d x_unit_vec = scm::math::vec4d(1.0,0.0,0.0,0.0);
        //float model_radius_scale = scm::math::length(scm::math::vec3d(model_matrix * x_unit_vec));
        //shader->uniform("model_radius_scale", model_radius_scale);
        shader->uniform("model_radius_scale", 1.f);

        size_t surfels_per_node = database->get_primitives_per_node();
        std::vector<scm::gl::boxf>const& bounding_box_vector = bvh->get_bounding_boxes();

        scm::gl::frustum frustum_by_model = camera_->get_frustum_by_model(scm::math::mat4f(model_matrix));

        std::cout << "renderable.size(): ";
        std::cout << renderable.size() << std::endl;

        for (auto const& node_slot_aggregate : renderable) {
            uint32_t node_culling_result = camera_->cull_against_frustum(
                frustum_by_model,
                bounding_box_vector[node_slot_aggregate.node_id_]);

            std::cout << "node_slot_aggregate.slot_id_: ";
            std::cout << (int)node_slot_aggregate.slot_id_ << std::endl;

            std::cout << "node_slot_aggregate.node_id_: ";
            std::cout << (int)node_slot_aggregate.node_id_ << std::endl;

            std::cout << "bounding_box_vector[node_slot_aggregate.node_id_]: ";
            //std::cout << bounding_box_vector[node_slot_aggregate.node_id_] << std::endl;
            

            if (node_culling_result != 1) {

                /*if (settings_.use_pvs_ && pvs->is_activated() && settings_.pvs_culling_
                    && !lamure::pvs::pvs_database::get_instance()->get_viewer_visibility(model_id, node_slot_aggregate.node_id_)) {
                    continue;
                }*/

                if (settings_.show_accuracy_) {
                    const float accuracy = 1.0 - (bvh->get_depth_of_node(node_slot_aggregate.node_id_) * 1.0) / (bvh->get_depth() - 1);
                    shader->uniform("accuracy", accuracy);
                }
                if (settings_.show_radius_deviation_) {
                    shader->uniform("average_radius", bvh->get_avg_primitive_extent(node_slot_aggregate.node_id_));
                }

                context_->apply();

                if (draw) {
                    context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST,
                        (node_slot_aggregate.slot_id_) * (GLsizei)surfels_per_node, surfels_per_node);
                    rendered_splats_ += surfels_per_node;
                    ++rendered_nodes_;
                }

            }
        }
        if (selection_.selected_model_ != -1) {
            break;
        }
    }
}


void LamurePointCloudPlugin::lamure_display() {
    const osg::GraphicsContext::Traits* traits = NULL;
    traits = coVRConfig::instance()->windows[0].context->getTraits();

    //std::cout << "lamure_display()" << std::endl;
    if (rendering_) {
        return;
    }
    rendering_ = true;
    camera_->set_projection_matrix(settings_.fov_, float(settings_.width_) / float(settings_.height_), settings_.near_plane_, settings_.far_plane_);
    model_database* database = model_database::get_instance();
    cut_database* cuts = cut_database::get_instance();
    controller* controller = controller::get_instance();
    //lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();
    if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
        controller->reset_system(data_provenance_);
    }
    else {
        controller->reset_system();
    }
    context_t context_id = controller->deduce_context_id(0);
    for (model_t model_id = 0; model_id < num_models_; ++model_id) {
        model_t m_id = controller->deduce_model_id(std::to_string(model_id));
        cuts->send_transform(context_id, m_id, scm::math::mat4f(model_transformations_[m_id]));
        cuts->send_threshold(context_id, m_id, settings_.lod_error_);
        cuts->send_rendered(context_id, m_id);
        database->get_model(m_id)->set_transform(scm::math::mat4f(model_transformations_[m_id]));
    }
    view_t cam_id = controller->deduce_view_id(context_id, camera_->view_id());
    cuts->send_camera(context_id, cam_id, *camera_);
    std::vector<scm::math::vec3d> corner_values = camera_->get_frustum_corners();
    double top_minus_bottom = scm::math::length((corner_values[2]) - (corner_values[0]));
    height_divided_by_top_minus_bottom_ = lamure::ren::policy::get_instance()->window_height() / top_minus_bottom;
    cuts->send_height_divided_by_top_minus_bottom(context_id, cam_id, height_divided_by_top_minus_bottom_);

    if (settings_.use_pvs_) {
        scm::math::mat4f cm = scm::math::inverse(scm::math::mat4f(camera_->trackball_matrix()));
        scm::math::vec3d cam_pos = scm::math::vec3d(cm[12], cm[13], cm[14]);
        //pvs->set_viewer_position(cam_pos);
    }

    //std::cout << (*device_);

    if (settings_.lod_update_) {
        if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
            controller->dispatch(context_id, device_, data_provenance_);
        }
        else {
            controller->dispatch(context_id, device_);
        }
    }
    view_t view_id = controller->deduce_view_id(context_id, camera_->view_id());
    context_->set_rasterizer_state(no_backface_culling_rasterizer_state_);

    if (settings_.splatting_) {
        //2 pass splatting
        //PASS 1
        context_->clear_color_buffer(pass1_fbo_, 0, scm::math::vec4f(.0f, .0f, .0f, 0.0f));
        context_->clear_depth_stencil_buffer(pass1_fbo_);
        context_->set_frame_buffer(pass1_fbo_);
        context_->bind_program(vis_xyz_pass1_shader_);
        context_->set_blend_state(color_no_blending_state_);
        context_->set_depth_stencil_state(depth_state_less_);

        set_uniforms(vis_xyz_pass1_shader_);
        //context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(render_width_, render_height_)));
        context_->set_viewport(scm::gl::viewport(scm::math::vec2f(traits->x, traits->y), scm::math::vec2f(traits->width, traits->height), scm::math::vec2f(0, 1)));
        context_->apply();

        draw_all_models(context_id, view_id, vis_xyz_pass1_shader_);
        draw_brush(vis_xyz_pass1_shader_);

        //PASS 2
        context_->clear_color_buffer(pass2_fbo_, 0, scm::math::vec4f(.0f, .0f, .0f, 0.0f));
        context_->clear_color_buffer(pass2_fbo_, 1, scm::math::vec4f(.0f, .0f, .0f, 0.0f));
        context_->clear_color_buffer(pass2_fbo_, 2, scm::math::vec4f(.0f, .0f, .0f, 0.0f));
        context_->set_frame_buffer(pass2_fbo_);
        context_->set_blend_state(color_blending_state_);
        context_->set_depth_stencil_state(depth_state_without_writing_);
        context_->set_rasterizer_state(no_backface_culling_rasterizer_state_);

        auto selected_pass2_shading_program = vis_xyz_pass2_shader_;
        if (settings_.enable_lighting_) {
            selected_pass2_shading_program = vis_xyz_pass2_lighting_shader_;
        }

        context_->bind_program(selected_pass2_shading_program);
        plugin->set_uniforms(selected_pass2_shading_program);
        //context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(render_width_, render_height_)));
        context_->apply();

        plugin->draw_all_models(context_id, view_id, selected_pass2_shading_program);
        plugin->draw_brush(selected_pass2_shading_program);

        //PASS 3
        context_->clear_color_buffer(fbo_, 0, scm::math::vec4f(0.0, 0.0, 0.0, 1.0f));
        context_->clear_depth_stencil_buffer(fbo_);
        context_->set_frame_buffer(fbo_);
        context_->set_depth_stencil_state(depth_state_disable_);

        auto selected_pass3_shading_program = vis_xyz_pass3_shader_;

        if (settings_.enable_lighting_) {
            selected_pass3_shading_program = vis_xyz_pass3_lighting_shader_;
        }

        context_->bind_program(selected_pass3_shading_program);
        plugin->set_uniforms(selected_pass3_shading_program);
        selected_pass3_shading_program->uniform("background_color", settings_.background_color_);
        selected_pass3_shading_program->uniform_sampler("in_color_texture", 0);
        context_->bind_texture(pass2_color_buffer_, filter_nearest_, 0);

        if (settings_.enable_lighting_) {
            context_->bind_texture(pass2_normal_buffer_, filter_nearest_, 1);
            context_->bind_texture(pass2_view_space_pos_buffer_, filter_nearest_, 2);
        }

        //context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(render_width_, render_height_)));
        context_->apply();
        screen_quad_->draw(context_);

        context_->clear_color_buffer(pass1_fbo_ , 0, scm::math::vec4f( .0f, .0f, .0f, 0.0f));

    }
    else {
        //single pass
        context_->clear_color_buffer(fbo_, 0, scm::math::vec4f(settings_.background_color_.x, settings_.background_color_.y, settings_.background_color_.z, 1.0f));
        context_->clear_depth_stencil_buffer(fbo_);
        context_->set_frame_buffer(fbo_);

        auto selected_single_pass_shading_program = vis_xyz_shader_;
        if (settings_.enable_lighting_) {
            selected_single_pass_shading_program = vis_xyz_lighting_shader_;
        }

        context_->bind_program(selected_single_pass_shading_program);
        context_->set_blend_state(color_no_blending_state_);
        context_->set_depth_stencil_state(depth_state_less_);

        set_uniforms(selected_single_pass_shading_program);
        if (settings_.background_image_ != "") {
          context_->bind_texture(bg_texture_, filter_linear_, 0);
          selected_single_pass_shading_program->uniform("background_image", true);
        }

        //context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(render_width_, render_height_)));
        context_->set_viewport(scm::gl::viewport(scm::math::vec2f(traits->x, traits->y), scm::math::vec2f(traits->width, traits->height), scm::math::vec2f(0, 1)));
        context_->apply();

        plugin->draw_all_models(context_id, view_id, selected_single_pass_shading_program);

        context_->bind_program(vis_xyz_shader_);
        plugin->draw_brush(vis_xyz_shader_);
        plugin->draw_resources(context_id, view_id);

    }

    //PASS 4: fullscreen quad
    context_->clear_default_depth_stencil_buffer();
    context_->clear_default_color_buffer();
    context_->set_default_frame_buffer();
    context_->set_depth_stencil_state(depth_state_disable_);
    context_->bind_program(vis_quad_shader_);
    context_->bind_texture(fbo_color_buffer_, filter_linear_, 0);
    vis_quad_shader_->uniform("gamma_correction", (bool)settings_.gamma_correction_);
    //context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(settings_.width_, settings_.height_)));
    context_->apply();
    screen_quad_->draw(context_);

    rendering_ = false;

    frame_time_.stop();
    frame_time_.start();
    //schism bug ? time::to_seconds yields milliseconds
    if (scm::time::to_seconds(frame_time_.accumulated_duration()) > 100.0) {
        fps_ = 1000.0f / scm::time::to_seconds(frame_time_.average_duration());
        frame_time_.reset();
    }
}

void LamurePointCloudPlugin::create_aux_resources() {

    if (!settings_.create_aux_resources_) {
        return;
    }

    //create bvh representation
    for (uint32_t model_id = 0; model_id < num_models_; ++model_id) {
        const auto& bounding_boxes = model_database::get_instance()->get_model(model_id)->get_bvh()->get_bounding_boxes();

        resource bvh_line_resource;
        bvh_line_resource.buffer_.reset();
        bvh_line_resource.array_.reset();

        std::vector<scm::math::vec3f> bvh_lines_to_upload;
        for (uint64_t node_id = 0; node_id < bounding_boxes.size(); ++node_id) {
            const auto& node = bounding_boxes[node_id];

            lines_from_min_max(node.min_vertex(), node.max_vertex(), bvh_lines_to_upload);

        }

        bvh_line_resource.buffer_ = device_->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
            scm::gl::USAGE_STATIC_DRAW, (sizeof(float) * 3) * bvh_lines_to_upload.size(), &bvh_lines_to_upload[0]);
        bvh_line_resource.array_ = device_->create_vertex_array(scm::gl::vertex_format
        (0, 0, scm::gl::TYPE_VEC3F, sizeof(float) * 3),
            boost::assign::list_of(bvh_line_resource.buffer_));

        bvh_line_resource.num_primitives_ = bvh_lines_to_upload.size();
        bvh_resources_[model_id] = bvh_line_resource;
    }

    //create auxiliary representations
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

            sparse_resources_[model_id] = points_resource;

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
            octree_resources_[model_id] = octree_resource;

            auto root_bb = model_database::get_instance()->get_model(model_id)->get_bvh()->get_bounding_boxes()[0];
            auto root_bb_min = scm::math::mat4f(model_transformations_[model_id]) * root_bb.min_vertex();
            auto root_bb_max = scm::math::mat4f(model_transformations_[model_id]) * root_bb.max_vertex();
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

                image_plane_resources_[model_id] = triangles_resource;
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

            frusta_resources_[model_id] = lines_resource;

        }
    }

}


int LamurePointCloudPlugin::unloadLMR(const char* filename, const char* covise_key)
{
    return 1;
}


void LamurePointCloudPlugin::readMenuConfigData(const char* menu, vector<ImageFileEntry>& menulist, ui::Group* subMenu)
{
    coCoviseConfig::ScopeEntries entries = coCoviseConfig::getScopeEntries(menu);
    for (const auto& entry : entries)
    {
        ui::Button* temp = new ui::Button(subMenu, entry.second);
        temp->setCallback([this, entry](bool state)
            {
                if (state)
                printf("createGeodes(planetTrans, entry.second)");
            });
        menulist.push_back(ImageFileEntry(entry.first.c_str(), entry.second.c_str(), (ui::Element*)temp));
    }
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



