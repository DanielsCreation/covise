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
    }
}
class LamurePointCloudInteractor;
class Measurement;


class Lamure : public opencover::coVRPlugin, public opencover::ui::Owner
{
public:

    struct Settings
    {
        int32_t frame_div_{1};
        int32_t vram_{1024};
        int32_t ram_{4096};
        int32_t upload_{32};
        bool provenance_{1};
        bool create_aux_resources_{1};
        bool gamma_correction_{0};
        bool surfel_shader_{1};
        bool face_eye_{0};
        int32_t gui_{1};
        int32_t travel_{2};
        float travel_speed_{20.5f};
        int32_t max_brush_size_{4096};
        bool lod_update_{1};
        float lod_error_{1.0f};
        bool use_pvs_{0};
        bool pvs_culling_{0};
        float point_size_factor_{1.0f};
        float surfel_size_factor_{1.0f};
        float aux_point_size_{1.0f};
        float aux_point_distance_{0.5f};
        float aux_point_scale_{1.0f};
        float aux_focal_length_{1.0f};
        int32_t vis_{0};
        int32_t show_normals_{0};
        bool show_accuracy_{0};
        bool show_radius_deviation_{0};
        bool show_output_sensitivity_{0};
        bool show_sparse_{0};
        bool show_views_{0};
        bool show_photos_{0};
        bool show_octrees_{0};
        bool show_bvhs_{0};
        bool show_pvs_{0};
        int32_t channel_{0};
        bool enable_lighting_{0};
        bool use_material_color_{1};
        scm::math::vec3f material_diffuse_{0.6f, 0.6f, 0.6f};
        scm::math::vec4f material_specular_{0.4f, 0.4f, 0.4f, 1000.0f};
        scm::math::vec3f ambient_light_color_{0.1f, 0.1f, 0.1f};
        scm::math::vec4f point_light_color_{1.0f, 1.0f, 1.0f, 1.2f};
        bool heatmap_{0};
        float heatmap_min_{0.0f};
        float heatmap_max_{0.05f};
        scm::math::vec3f background_color{68.0f / 255.0f, 0.0f, 84.0f / 255.0f};
        scm::math::vec3f heatmap_color_min_{68.0f / 255.0f, 0.0f, 84.0f / 255.0f};
        scm::math::vec3f heatmap_color_max_{251.f / 255.f, 231.f / 255.f, 35.f / 255.f};
        std::string atlas_file_{""};
        std::string json_{""};
        std::string pvs_{""};
        std::string background_image_{""};
        std::vector<std::string> models;
        std::vector<uint32_t> initial_selection;
        std::map<uint32_t, scm::math::mat4d> transforms_;
        std::map<uint32_t, std::shared_ptr<lamure::prov::octree>> octrees_;
        std::map<uint32_t, std::vector<lamure::prov::aux::view>> views_;
        std::map<uint32_t, std::string> aux_;
        float max_radius_{std::min(std::numeric_limits<float>::max(), 0.1f)};
        float scale_radius_{1.5f};
        std::vector<float> bvh_color_{1.0f, 1.0f, 0.0f, 1.0f};
        std::vector<float> frustum_color_{0.0f, 0.0f, 0.0f, 1.0f};
        uint16_t num_models;
        bool pointcloud_state{ true };
        bool boundingbox_state{ false };
        bool frustum_state{ false };
        bool coord_state{ false };
        bool text_state{ false };
        bool sync_state{ true };
        bool notify_state{ true };
    };

    struct ModelInfo
    {
        std::vector<scm::math::mat4d> model_transformations_;
        std::vector<scm::math::vec3f> root_bb_min;
        std::vector<scm::math::vec3f> root_bb_max;
        std::vector<scm::math::vec3f> root_center;
        scm::math::vec3f models_min;
        scm::math::vec3f models_max;
        scm::math::vec3d models_center;
    };

    struct RenderInfo
    {
        uint64_t rendered_splats_{0};
        uint64_t rendered_nodes_{0};
        uint64_t rendered_bounding_boxes_{0};
        float fps_{0.0f};
    };

    struct Trackball
    {
        float dist_ = 0.0;
        float size_ = 0.0;
        osg::Vec3 initial_pos_;
        osg::Vec3 pos_;
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