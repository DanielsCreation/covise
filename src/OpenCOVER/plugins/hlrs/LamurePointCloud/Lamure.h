#ifndef _Lamure_H
#define _Lamure_H

// Platform-specific headers
#ifdef _WIN32
#include <windows.h>
#endif

//gl
#include <GL/glew.h>

#include <cover/coVRPluginSupport.h>
#include <osgViewer/ViewerBase>
#include <scm/core/math.h>
#include <vector>
#include <string>
#include <functional>
#include <memory>
#include <map>
#include <limits>

#include "renderer.h"
#include "LamureUI.h"
#include "util.h"
#include "measurement.h"

#include <lamure/ren/data_provenance.h>
#include <lamure/prov/prov_aux.h>

// Forward declarations
namespace opencover
{
    namespace ui
    {
        class Element;
        class Group;
        class Button; // Added for LamureUI::getMeasureButton()
    }
}
class LamurePointCloudInteractor;
class Measurement;

class Lamure : public opencover::coVRPlugin, public opencover::ui::Owner
{
public:

    struct Settings
    {
        int32_t frame_div{1};
        int32_t vram{1024};
        int32_t ram{4096};
        int32_t upload{32};
        bool provenance{1};
        bool create_aux_resources{1};
        bool gamma_correction{0};
        bool face_eye{0};
        int32_t gui{1};
        int32_t travel{2};
        float travel_speed{20.5f};
        int32_t max_brush_size{4096};
        bool lod_update{1};
        float lod_error{1.0f};
        LamureRenderer::ShaderType shader_type {LamureRenderer::ShaderType::Point};
        bool use_pvs{0};
        bool pvs_culling{0};
        float aux_point_size{1.0f};
        float aux_point_distance{0.5f};
        float aux_point_scale{1.0f};
        float aux_focal_length{1.0f};
        int32_t vis{0};
        int32_t show_normals{0};
        bool show_accuracy{0};
        bool show_radius_deviation{0};
        bool show_output_sensitivity{0};
        bool show_sparse{0};
        bool show_views{0};
        bool show_photos{0};
        bool show_octrees{0};
        bool show_bvhs{0};
        bool show_pvs{0};
        int32_t channel{0};
        bool enable_lighting{0};
        bool use_material_color{1};
        scm::math::vec3f material_diffuse{0.6f, 0.6f, 0.6f};
        scm::math::vec4f material_specular{0.4f, 0.4f, 0.4f, 1000.0f};
        scm::math::vec3f ambient_light_color{0.1f, 0.1f, 0.1f};
        scm::math::vec4f point_light_color{1.0f, 1.0f, 1.0f, 1.2f};
        bool heatmap{0};
        float heatmap_min{0.0f};
        float heatmap_max{0.05f};
        scm::math::vec3f background_color{68.0f / 255.0f, 0.0f, 84.0f / 255.0f};
        scm::math::vec3f heatmap_color_min{68.0f / 255.0f, 0.0f, 84.0f / 255.0f};
        scm::math::vec3f heatmap_color_max{251.f / 255.f, 231.f / 255.f, 35.f / 255.f};
        std::string atlas_file{};
        std::string json{};
        std::string pvs{};
        std::string background_image{};
        std::vector<std::string> models;
        std::vector<uint32_t> initial_selection;
        std::map<uint32_t, scm::math::mat4d> transforms;
        std::map<uint32_t, std::shared_ptr<lamure::prov::octree>> octrees;
        std::map<uint32_t, std::vector<lamure::prov::aux::view>> views;
        std::map<uint32_t, std::string> aux;
        float min_radius{0.0f};
        float max_radius{std::min(std::numeric_limits<float>::max(), 0.1f)};
        float scale_radius{1.5f};
        std::vector<float> bvh_color{1.0f, 1.0f, 0.0f, 1.0f};
        std::vector<float> frustum_color{0.0f, 0.0f, 0.0f, 1.0f};
        uint16_t num_models;
        bool show_pointcloud{ true };
        bool show_boundingbox{ false };
        bool show_frustum{ false };
        bool show_coord{ false };
        bool show_text{ false };
        bool show_sync{ true };
        bool show_notify{ true };
    };

    struct ModelInfo
    {
        std::vector<scm::math::mat4d> model_transformations;
        std::vector<scm::math::vec3f> root_bb_min;
        std::vector<scm::math::vec3f> root_bb_max;
        std::vector<scm::math::vec3f> root_center;
        scm::math::vec3f models_min;
        scm::math::vec3f models_max;
        scm::math::vec3d models_center;
    };

    struct RenderInfo
    {
        uint64_t rendered_splats{0};
        uint64_t rendered_nodes{0};
        uint64_t rendered_bounding_boxes{0};
        float fps{0.0f};
    };

    struct Trackball
    {
        float dist = 0.0;
        float size = 0.0;
        osg::Vec3 initial_pos;
        osg::Vec3 pos;
    };

    Lamure();
    ~Lamure();

    static Lamure* instance();
    bool init2();
    static int loadLMR(const char* filename, osg::Group* parent, const char* ck = "");
    static int unloadLMR(const char* filename, const char* ck = "");
    void preFrame();
    void startMeasurement();
    void stopMeasurement();

    LamureUI* getUI() { return m_ui.get(); }
    LamureRenderer* getRenderer() { return m_renderer.get(); }

    Settings& getSettings() { return m_settings; }
    ModelInfo& getModelInfo() { return m_model_info; }
    RenderInfo& getRenderInfo() { return m_render_info; }
    Trackball& getTrackball() { return m_trackball; }
    bool getProvValid() const { return prov_valid; }
    lamure::ren::Data_Provenance& getDataProvenance() { return m_data_provenance; }

    LamurePointCloudInteractor* interactor;
    
    void loadSettings(const std::string &filename);
    bool rendering_ = false;

private:
    static Lamure* plugin;

    std::unique_ptr<LamureRenderer> m_renderer;
    std::unique_ptr<LamureUI>       m_ui;

    Settings    m_settings;
    ModelInfo   m_model_info;
    RenderInfo  m_render_info;
    Trackball   m_trackball;

    lamure::ren::Data_Provenance        m_data_provenance;
    osgViewer::ViewerBase::FrameScheme  rendering_scheme;
    std::unique_ptr<Measurement>        _measurement;
    std::function<void(bool)>           _measureCB;
    std::vector<osg::Vec3>              _path;
    float                               _speed = 1.0f;
    bool                                measurement_running = 0;
    bool                                prov_valid;
};

#endif