
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
//#include <lamure/types.h>
//#include <lamure/ren/config.h>
//#include <lamure/ren/model_database.h>
//#include <lamure/ren/cut_database.h>
//#include <lamure/ren/dataset.h>
#include <lamure/ren/controller.h>
#include <lamure/ren/policy.h>
//#include <lamure/pvs/pvs_database.h>
//#include <lamure/ren/ray.h>
//#include <lamure/prov/prov_aux.h>
#include <lamure/prov/octree.h>
//#include <lamure/vt/VTConfig.h>
//#include <lamure/vt/ren/CutDatabase.h>
#include <lamure/vt/ren/CutUpdate.h>
//#include <lamure/vt/pre/AtlasFile.h>

//schism
//#include <scm/high_res_timer.h>
//#include <scm/time.h>
//#include <scm/core.h>
//#include <scm/core/math.h>
//#include <scm/core/io/tools.h>
//#include <scm/core/pointer_types.h>
//#include <scm/core/platform/platform.h>
//#include <scm/core/utilities/platform_warning_disable.h>
//#include <scm/gl_core/gl_core_fwd.h>
//#include <scm/gl_util/primitives/quad.h>
//#include <scm/gl_util/font/font_face.h>
//#include <scm/gl_util/font/text.h>
//#include <scm/gl_util/font/text_renderer.h>
//#include <scm/gl_util/primitives/primitives_fwd.h>
//#include <scm/gl_util/data/imaging/texture_loader.h>
#include <scm/gl_util/primitives/geometry.h>
//#include <scm/gl_util/primitives/box.h>


//#include <cover/coVRMSController.h>
//#include <cover/coVRTui.h>
//#include <util/coTypes.h>
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

#include "C:\src\covise\src\OpenCOVER\plugins\general\PointCloud\PointCloudGeometry.h"
#include "C:\src\covise\src\OpenCOVER\plugins\hlrs\LamurePointCloud\Points.h"
#include <LamureGeometry.h>


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
    const LamurePointCloudPlugin *instance() const;
    static int load(const char* filename, osg::Group* parent, const char* ck = "");
    static int unload(const char* filename, const char* ck = "");
    void preFrame();
    bool update() const;
    //void set_uniforms(scm::gl::program_ptr shader);
    //void draw_all_models(const lamure::context_t context_id, const lamure::view_t view_id, scm::gl::program_ptr shader);
    //void draw_brush(scm::gl::program_ptr shader);
    //std::string const strip_whitespace(std::string const& in_string);
    ui::Group* FileGroup;
    osg::ref_ptr<osg::Group> LamureGroup;
    struct settings;
    //void load_settings(const std::string &filename, settings& settings);

private:
    static LamurePointCloudPlugin* plugin;
    void selectedMenuButton(ui::Element*);
    std::vector<ImageFileEntry> pointVec;
    void clearData();
    void readMenuConfigData(const char*, std::vector<ImageFileEntry>&, ui::Group*);
    float pointSizeValue = 4;
    void createGeodes(osg::Group*, const std::string&);
    osg::Point* pointstate;
    osg::StateSet* stateset;
    osg::BoundingBox box;
    osg::Vec3Array* points;
    osg::Vec3Array* colors;
    osg::VertexBufferObject* vertexBufferArray;
    osg::ElementBufferObject* primitiveBufferArray;
    PointSet* pointSet = nullptr;

protected:
    osg::MatrixTransform* planetTrans;
    ui::Menu* lamureMenu = nullptr;
    ui::Menu* loadMenu = nullptr;
    ui::Group* loadGroup = nullptr;
    ui::Group* selectionGroup = nullptr;
    ui::ButtonGroup* selectionButtonGroup = nullptr;
    ui::ButtonGroup* fileButtonGroup = nullptr;
    ui::Group* viewGroup = nullptr;
    ui::Button* adaptLODButton = nullptr;
};

#endif