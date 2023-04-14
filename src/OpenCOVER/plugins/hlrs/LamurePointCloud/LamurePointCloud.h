
#ifndef _Lamure_PC_PLUGIN_H
#define _Lamure_PC_PLUGIN_H

//gl
//#include <GL/glew.h>

//boost
//#include <boost/assign/list_of.hpp>
#include <boost/regex.h>
//#include <boost/lexical_cast.hpp>
//#include <boost/filesystem.hpp>


//lamure
#include <types.h>
#include <config.h>
#include <model_database.h>
#include <cut_database.h>
#include <dataset.h>
#include <controller.h>
#include <policy.h>
#include <ray.h>

#include <lamure/prov/octree.h>
#include <lamure/vt/VTConfig.h>
#include <lamure/vt/ren/CutDatabase.h>
#include <lamure/vt/ren/CutUpdate.h>

//lamure
//#include <lamure/types.h>
//#include <lamure/ren/config.h>
//#include <lamure/ren/model_database.h>
//#include <lamure/ren/cut_database.h>
//#include <lamure/ren/dataset.h>
//#include <lamure/ren/controller.h>
//#include <lamure/ren/policy.h>
#include <lamure/pvs/pvs_database.h>
//#include <lamure/ren/ray.h>
//#include <lamure/prov/prov_aux.h>
//#include <lamure/vt/pre/AtlasFile.h>

//schism
#include <scm/time.h>
#include <scm/core.h>
#include <scm/core/math.h>
#include <scm/core/io/tools.h>
#include <scm/core/pointer_types.h>
#include <scm/core/platform/platform.h>
#include <scm/core/utilities/platform_warning_disable.h>
#include <scm/gl_core/gl_core_fwd.h>
#include <scm/gl_util/primitives/quad.h>
#include <scm/gl_util/font/font_face.h>
#include <scm/gl_util/font/text.h>
#include <scm/gl_util/font/text_renderer.h>
#include <scm/gl_util/primitives/primitives_fwd.h>
#include <scm/gl_util/data/imaging/texture_loader.h>
#include <scm/gl_util/primitives/geometry.h>
#include <scm/gl_util/primitives/box.h>


//#include <cover/coVRMSController.h>
//#include <cover/coVRTui.h>
//#include <util/coTypes.h>

#include <util/coExport.h>
#include <PluginUtil/FeedbackManager.h>
#include <PluginUtil/ModuleInteraction.h>
#include <OpenVRUI/coButtonInteraction.h>
#include <config/CoviseConfig.h>
#include <cover/coVRFileManager.h>
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

#include <osg/Version>
#include <osg/Geometry>
#include <osg/Vec3>
#include <osg/BufferObject>
#include <osg/Point>
#include <osg/PointSprite>
#include <osg/Texture2D>
#include <osgDB/ReadFile>

#include "C:\src\covise\src\OpenCOVER\plugins\hlrs\LamurePointCloud\LamureGeometry.h"
#include "C:\src\covise\src\OpenCOVER\plugins\hlrs\LamurePointCloud\LamureDrawable.h"
#include "C:\src\covise\src\OpenCOVER\plugins\hlrs\LamurePointCloud\Points.h"


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
    bool init();
    string getConfigEntry(string scope);
    const char* stringToConstChar(string str);
    string extractFilename(const string pathname);
    void strcpyTail(char* suffix, const char* str, char c);
    const LamurePointCloudPlugin *instance() const;
    static int loadLMR(const char* filename, osg::Group* parent, const char* ck = "");
    static int unloadLMR(const char* filename, const char* ck = "");
    ui::Group* FileGroup;
    void preFrame();
    //bool update();
    void set_uniforms(scm::gl::program_ptr shader);
    void lamure_display();
    void draw_all_models(const context_t context_id, const view_t view_id, scm::gl::program_ptr shader);
    void draw_brush(scm::gl::program_ptr shader);
    void draw_resources(const context_t context_id, const view_t view_id);
    void covise_display();
    void create_aux_resources();
    void co_draw_all_models(const context_t context_id, const view_t view_id, scm::gl::program_ptr shader);
    bool parse_prefix(std::string& in_string, std::string const& prefix);
    bool read_shader(std::string const& path_string, std::string& shader_string, bool keep_optional_shader_code);
    void init_lamure_shader();
    void create_framebuffers();
    void init_render_states();
    void init_camera();
    std::string const strip_whitespace(std::string const& in_string);
    scm::math::mat4d load_matrix(const std::string& filename);
    osg::ref_ptr<osg::Group> LamureGroup;
    void load_settings(const std::string &filename);

private:
    static LamurePointCloudPlugin* plugin;
    void selectedMenuButton(ui::Element*);
    std::vector<ImageFileEntry> pointVec;
    void clearData();
    void readMenuConfigData(const char*, std::vector<ImageFileEntry>&, ui::Group*);
    float pointSizeValue = 4;
    void createGeodes(osg::Group*, const std::string&);
    bool adaptLOD = true; // LOD enable/disable
    osg::Point* pointstate;
    osg::StateSet* stateset;
    osg::BoundingBox box;
    osg::Vec3Array* points;
    osg::Vec3Array* colors;
    osg::VertexBufferObject* vertexBufferArray;
    osg::ElementBufferObject* primitiveBufferArray;
    PointSet* pointSet = nullptr;
    osg::ref_ptr<osg::Geode> geo;
    osg::ref_ptr<osg::MatrixTransform> transform;
    osg::ref_ptr<osg::StateSet> state;
    osg::ref_ptr<LamureGeometry> drawable;
    

protected:
    ui::Menu* lamureMenu = nullptr;
    ui::Menu* loadMenu = nullptr;
    ui::Group* loadGroup = nullptr;
    ui::Group* selectionGroup = nullptr;
    osg::MatrixTransform* planetTrans;

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

#endif
