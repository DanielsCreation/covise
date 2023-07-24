
#ifndef _Lamure_PC_PLUGIN_H
#define _Lamure_PC_PLUGIN_H

//gl
#ifndef __gl_h_
    #include <GL/glew.h>
#endif
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw_gl3.h>

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
#include <cover/ui/Menu.h>
#include <cover/ui/Slider.h>
#include <cover/ui/Action.h>
#include <cover/ui/Menu.h>
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

#include "LamureGeometry.h"
#include <lamure/types.h>
#include <scm/gl_core/shader_objects/shader_objects_fwd.h>
#include <lamure/ren/camera.h>


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
    bool init();
    static int loadLMR(const char* filename, osg::Group* parent, const char* ck = "");
    static int unloadLMR(const char* filename, const char* ck = "");
    void preFrame();
    bool update();
    size_t query_video_memory_in_mb();

    // shared functions
    void init_lamure_shader();
    bool read_shader(std::string const& path_string, std::string& shader_string, bool keep_optional_shader_code);
    void create_aux_resources();
    void create_aux_resources_buffered();
    void draw_resources(const lamure::context_t context_id, const lamure::view_t view_id);
    void draw_brush(scm::gl::program_ptr shader);
    void set_uniforms(scm::gl::program_ptr shader);
    
    void create_framebuffers();
    void init_render_states();
    void init_camera();
    void lamure_display();
    void draw_all_models(const lamure::context_t context_id, const lamure::view_t view_id, scm::gl::program_ptr shader);

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
    void lines_from_min_max(const scm::math::vec3f& min_vertex, const scm::math::vec3f& max_vertex, std::vector<scm::math::vec3f> lines);
    void lines_from_min_max_buffered(const scm::math::vec3f& min_vertex, const scm::math::vec3f& max_vertex, osg::ref_ptr<osg::Vec3Array>& lines);

    // objects and pointers
    ui::Group* FileGroup;
    scm::math::mat4d load_matrix(const std::string& filename);
    void load_settings(const std::string &filename);
    bool rendering_ = false;
    HGLRC current_context;
    HDC hdc;
    HWND hwnd;

    // substitutions
    //osg::ref_ptr<LamureDevice> lmr_device = NULL;
    //osg::ref_ptr<LamureContext> lmr_device = NULL;


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
    osg::ref_ptr<osg::StateSet> state;
    osg::ref_ptr<osg::Node> file;
    osg::ref_ptr<osg::MatrixTransform> transform;
    osg::ref_ptr<osg::Geode> geode;
    osg::ref_ptr<LamureGeometry> geometry;

    osg::ref_ptr<osg::Switch> _switch;
    osg::ref_ptr<struct GeoGl> geo_gl;


protected:
    ui::Menu* lamureMenu = nullptr;
    ui::Menu* loadMenu = nullptr;
    ui::Group* loadGroup = nullptr;
    ui::Group* selectionGroup = nullptr;

    ui::Button* singleSelectButton = nullptr;
    ui::Button* translationButton = nullptr;
    ui::Button* rotPointsButton = nullptr;
    ui::Button* rotAxisButton = nullptr;
    ui::Button* moveButton = nullptr;
    ui::Button* saveButton = nullptr;
    ui::Button* fileButton = nullptr;
    ui::Button* deselectButton = nullptr;
    ui::Button* createNurbsSurface = nullptr;
    //ui::Button *deleteButton = nullptr;
    ui::ButtonGroup* selectionButtonGroup = nullptr;
    ui::ButtonGroup* fileButtonGroup = nullptr;
    ui::Group* viewGroup = nullptr;
    ui::Button* adaptLODButton = nullptr;
    ui::Slider* pointSizeSlider = nullptr;

    ui::Slider* lodFarDistanceSlider = nullptr;
    ui::Slider* lodNearDistanceSlider = nullptr;
};

class COVEREXPORT PCLNode : public osg::Drawable
{
private:
    bool displayVideo; // true if CoviseConfig.displayVideo is set
    bool renderTextures;
    std::string m_pcl_node;

public:
    PCLNode(std::string MarkerTrackingVariant);
    virtual ~PCLNode();
    static PCLNode* pcl_node;
    virtual void drawImplementation(osg::RenderInfo& renderInfo) const;
    /** Clone the type of an object, with Object* return type.
        Must be defined by derived classes.*/
    virtual osg::Object* cloneType() const;

    /** Clone the an object, with Object* return type.
        Must be defined by derived classes.*/
    virtual osg::Object* clone(const osg::CopyOp&) const;
};











#endif
