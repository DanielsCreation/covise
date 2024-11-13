
#ifndef _Lamure_PC_PLUGIN_H
#define _Lamure_PC_PLUGIN_H

//gl
#ifndef __gl_h_
    #include <GL/glew.h>
#endif

#include <cover/coVRPluginSupport.h>
#include <cover/coVRMSController.h>
#include <cover/coVRPluginList.h>
#include <cover/coVRCommunication.h>
#include <cover/coVRConfig.h>
#include <cover/coVRTui.h>
#include <cover/coVRShader.h>
#include <cover/VRViewer.h>
#include <cover/PluginMenu.h>
#include <cover/ui/ButtonGroup.h>
#include <cover/ui/Button.h>
#include <cover/ui/Label.h>
#include <cover/ui/Menu.h>
#include <cover/ui/Slider.h>
#include <cover/ui/Action.h>
#include <cover/ui/Manager.h>
#include <cover/ui/Owner.h>
#include <cover/ui/SelectionList.h>
#include <cover/coVRStatsDisplay.h>
#include <cover/VRSceneGraph.h>
#include "cover/OpenCOVER.h"
#include <cover/VRWindow.h>
#include <cover/VRViewer.h>
#include <cover/coVRFileManager.h>

#include <osg/Version>
#include <osg/Geometry>
#include <osg/Vec3>
#include <osg/Vec3ui>
#include <osg/BufferObject>
#include <osg/Point>
#include <osg/PointSprite>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/LineStipple>
#include <osg/BufferTemplate>
#include <osg/State>

#include "LamureDrawable.h"
#include "LamureGeometry.h"
#include <lamure/types.h>
#include <scm/gl_core/shader_objects/shader_objects_fwd.h>
#include <lamure/ren/camera.h>
#include <lamure/lmr_camera.h>
#include <lamure/ren/trackball.h>


namespace opencover {
namespace ui {
    class Element;
    class Group;
    class Slider;
    class Menu;
    class Button;
}
}

using namespace opencover;

class LamurePointCloudPlugin : public coVRPlugin, public ui::Owner
{
    class ImageFileEntry
    {
    public:
        string menuName;
        string fileName;
        ui::Element* fileMenuItem;

        ImageFileEntry(const char* menu, const char* file, ui::Element* menuitem)
        {
            menuName = menu;
            fileName = file;
            fileMenuItem = menuitem;
        }
    };

public:
    LamurePointCloudPlugin();
    ~LamurePointCloudPlugin();

    static LamurePointCloudPlugin* instance();
    bool init2();
    void config();
    static int loadLMR(const char* filename, osg::Group* parent, const char* ck = "");
    static int unloadLMR(const char* filename, const char* ck = "");
    void preFrame();
    //bool update();
    void postFrame();
    //void preDraw();
    size_t query_video_memory_in_mb();

    // shared functions
    void init_lamure_shader();
    void init_rtt_camera();
    void sync_cameras();
    bool read_shader(std::string const& path_string, std::string& shader_string, bool keep_optional_shader_code);
    void create_aux_resources();
    void create_pcl_resources();
    void create_aux_representation();
    void draw_resources(const lamure::context_t context_id, const lamure::view_t view_id);
    void draw_brush(scm::gl::program_ptr shader);
    void set_uniforms(scm::gl::program_ptr shader);
    float get_atlas_scale_factor();
    void create_framebuffers();
    void init_render_states();
    void init_camera();
    void lamure_display();
    void draw_all_models(const lamure::context_t context_id, const lamure::view_t view_id, scm::math::mat4d view_matrix, scm::math::mat4d projection_matrix, scm::gl::program_ptr shader);
    void sync_cameras(lmr_camera* lamure_camera, osg::Camera* osg_camera);

    // util
    bool parse_prefix(std::string& in_string, std::string const& prefix);
    string getConfigEntry(string scope);
    string getConfigEntry(string scope, string name);
    string extractFilename(const string pathname);
    void strcpyTail(char* suffix, const char* str, char c);
    const char* stringToConstChar(string str);
    scm::gl::data_format get_tex_format();
    void apply_vt_cut_update();
    //float get_atlas_scale_factor();
    void lines_from_min_max(const scm::math::vec3f& min_vertex, const scm::math::vec3f& max_vertex, std::vector<scm::math::vec3f>& lines);
    void lines_from_min_max_buffered(const scm::math::vec3f& min_vertex, const scm::math::vec3f& max_vertex, osg::ref_ptr<osg::Vec3Array>& lines);
    
    // objects and pointers
    ui::Group* FileGroup;
    scm::math::mat4d load_matrix(const std::string& filename);
    void load_settings(const std::string &filename);
    bool rendering_ = false;

    HWND hwnd_cover;
    HWND hwnd_opencover;
    HWND hwnd_current;

    HGLRC HGLRC_cover;
    HGLRC HGLRC_opencover; 
    HGLRC HGLRC_current;
    HGLRC HGLRC_last;

    HDC hdc_cover;
    HDC hdc_opencover;
    HDC hdc_current; 
    HDC hdc_last;


private:
    static LamurePointCloudPlugin* plugin;
    void selectedMenuButton(ui::Element*);
    std::vector<ImageFileEntry> pointVec;
    void clearData();
    std::string const strip_whitespace(std::string const& in_string);
    void readMenuConfigData(const char*, std::vector<ImageFileEntry>&, ui::Group*);
    float pointSizeValue = 4;
    void createGeodes(osg::Group*, const std::string&);
    bool adaptLOD = true; // LOD enable/disable
    
    osg::Point* pointstate;
    osg::StateSet* stateset;
    osg::BoundingBox box;
    osg::MatrixTransform* planetTrans;
    osg::Vec3Array* points;
    osg::Vec3Array* colors;
    osg::ElementBufferObject* primitiveBufferArray;
    PointSet* pointSet = nullptr;
    osg::ref_ptr<osg::Group> LamureGroup;
    osg::ref_ptr<osg::Node> file;
    osg::ref_ptr<osg::Geode> geode;
    osg::ref_ptr<LamureGeometry> geometry;
    osg::ref_ptr<LamureDrawable> draw1;
    osg::ref_ptr<osg::Switch> _switch;




public:
    void printNodePath(osg::ref_ptr<osg::Node> pointer);
    std::vector<vector<float>> getSerializedBvhMinMax(const std::vector<scm::gl::boxf>);
    std::vector<float> getBoxCorners(scm::gl::boxf);
    float* VecToArr(std::vector<std::vector<float>> vec);
    int* VecToArr(std::vector<std::vector<int>> vec);
    osg::ref_ptr<struct GLGrp> gl_grp;
    osg::ref_ptr<osg::MatrixTransform> transform;
    ui::Menu* menu = nullptr;
    ui::Group* group = nullptr;

    ui::Button* bounding_box_show_button = nullptr;
    ui::Button* frustum_show_button = nullptr;
    ui::Button* coord_show_button = nullptr;
    ui::Button* sync_cam_button = nullptr;
    ui::Button* notify_button = nullptr;

    osg::ref_ptr<osg::Geometry> _triangleGeometry;
    osg::ref_ptr<osg::StateSet> _triangleStateSet;

    osg::ref_ptr<osg::Geometry> _lineGeometry;
    osg::ref_ptr<osg::StateSet> _lineStateSet;

    osg::ref_ptr<osg::Geometry> _pointGeometry;
    osg::ref_ptr<osg::StateSet> _pointStateSet;

    // Slider-Deklarationen

    ui::Slider* cameraPosXSlider = nullptr;
    ui::Slider* cameraPosYSlider = nullptr;
    ui::Slider* cameraPosZSlider = nullptr;

    ui::Slider* modelPosXSlider = nullptr;
    ui::Slider* modelPosYSlider = nullptr;
    ui::Slider* modelPosZSlider = nullptr;

    ui::Slider* rotationXSlider = nullptr;
    ui::Slider* rotationYSlider = nullptr;
    ui::Slider* rotationZSlider = nullptr;

    // UI-Elemente für Kamera X-Position
    ui::Label* cameraPosXLabel;
    ui::Button* cameraPosXPlusButton;
    ui::Button* cameraPosXMinusButton;

    // UI-Elemente für Kamera Y-Position
    ui::Label* cameraPosYLabel;
    ui::Button* cameraPosYPlusButton;
    ui::Button* cameraPosYMinusButton;

    // UI-Elemente für Kamera Z-Position
    ui::Label* cameraPosZLabel;
    ui::Button* cameraPosZPlusButton;
    ui::Button* cameraPosZMinusButton;

    scm::math::vec3d cameraPosition;

    scm::math::vec3d rotationAngles = scm::math::vec3d(0.0, 0.0, 0.0);

    void updateModelRotation();


protected:
    ui::Group* loadGroup = nullptr;
    ui::Group* model_grp = nullptr;
    ui::Group* viewGroup = nullptr;
    ui::Menu* loadMenu = nullptr;
    ui::Button* rotPointsButton = nullptr;
    ui::Button* rotAxisButton = nullptr;
    ui::Button* moveButton = nullptr;
    ui::Button* saveButton = nullptr;
    ui::Button* fileButton = nullptr;
    ui::Button* deselectButton = nullptr;
    ui::Button* createNurbsSurface = nullptr;
    //ui::Button *deleteButton = nullptr;
    ui::Button* adaptLODButton = nullptr;

    ui::Slider* maxRadiusSlider = nullptr;
    ui::Slider* scaleRadiusSlider = nullptr;

    ui::SelectionList* shader_list = nullptr;

    ui::Slider* lodFarDistanceSlider = nullptr;
    ui::Slider* lodNearDistanceSlider = nullptr;



    void setUpStateSets();
};


static unsigned int CreateShader(const std::string& vertexShader, const std::string& fragmentShader, uint8_t ctx_id)
{
    osg::GLExtensions* gl_api = new osg::GLExtensions(ctx_id);
    unsigned int program = gl_api->glCreateProgram();
    unsigned int vs = CompileShader(GL_VERTEX_SHADER, vertexShader, ctx_id);
    unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, fragmentShader, ctx_id);

    gl_api->glAttachShader(program, vs);
    gl_api->glAttachShader(program, fs);
    gl_api->glLinkProgram(program);
    gl_api->glValidateProgram(program);

    gl_api->glDeleteProgram(vs);
    gl_api->glDeleteProgram(fs);
    return 1;
}

static unsigned int CompileShader(unsigned int type, const std::string& source, uint8_t ctx_id)
{
    osg::GLExtensions* gl_api = new osg::GLExtensions(ctx_id);
    unsigned int id = gl_api->glCreateShader(type);
    const char* src = source.c_str();
    gl_api->glShaderSource(id, 1, &src, nullptr);
    gl_api->glCompileShader(id);

    int result;
    gl_api->glGetShaderiv(id, GL_COMPILE_STATUS, &result);
    if (result == false)
    {
        int length;
        gl_api->glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
        char* message = (char*)alloca(length * sizeof(char));
        gl_api->glGetShaderInfoLog(id, length, &length, message);
        std::cout << "Failed to compile " <<
            (type == GL_VERTEX_SHADER ? "vertex" : "fragment") << " shader!" << std::endl;
        std::cout << message << std::endl;
        gl_api->glDeleteProgram(id);
        return 0;
    };
    return id;
}

#endif