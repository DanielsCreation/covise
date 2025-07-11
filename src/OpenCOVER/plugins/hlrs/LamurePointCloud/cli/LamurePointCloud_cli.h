#ifndef LAMURE_POINT_CLOUD_CLI_H
#define LAMURE_POINT_CLOUD_CLI_H

#include "PointCloudRenderer_cli.h"
#include "PointCloudUIManager_cli.h"
#include "measurement.h"

#include <cover/coVRPlugin.h>
#include <cover/ui/Owner.h>
#include <lamure/types.h>
#include <lamure/ren/camera.h>
#include <scm/core/math.h>
#include <scm/gl_core/render_device.h>
#include <osg/Group>
#include <osg/Camera>
#include <vector>
#include <string>
#include <map>
#include <functional>
#include <cover/coVRConfig.h>
#include <cover/coVRFileManager.h>
#include <cover/VRSceneGraph.h>
#include <cover/VRViewer.h>
#include <cover/ui/Button.h>
#include <cover/ui/Label.h>
#include <cover/ui/Menu.h>
#include <cover/ui/Slider.h>
#include <cover/ui/SelectionList.h>
#include <limits>
#include <memory>
#include <lamure/config.h>
#include <lamure/prov/octree.h>
#include <lamure/prov/prov_aux.h>

class PointCloudRenderer_cti;
class PointCloudUIManager_cti;

struct AppSettings
{
    int32_t frame_div{1};
    int32_t vram{1024};
    int32_t ram{4096};
    int32_t upload{32};
    bool provenance{true};
    bool create_aux_resources{true};
    bool gamma_correction{false};
    bool surfel_shader{true};
    bool face_eye{false};
    int32_t gui{1};
    int32_t travel{2};
    float travel_speed{20.5f};
    int32_t max_brush_size{4096};
    bool lod_update{true};
    float lod_error{1.0f};
    bool use_pvs{false};
    bool pvs_culling{false};
    float point_size_factor{1.0f};
    float surfel_size_factor{1.0f};
    float aux_point_size{1.0f};
    float aux_point_distance{0.5f};
    float aux_point_scale{1.0f};
    float aux_focal_length{1.0f};
    int32_t vis{0};
    int32_t show_normals{0};
    bool show_accuracy{false};
    bool show_radius_deviation{false};
    bool show_output_sensitivity{false};
    bool show_sparse{false};
    bool show_views{false};
    bool show_photos{false};
    bool show_octrees{false};
    bool show_bvhs{false};
    bool show_pvs{false};
    int32_t channel{0};
    bool enable_lighting{false};
    bool use_material_color{true};
    scm::math::vec3f material_diffuse{0.6f,0.6f,0.6f};
    scm::math::vec4f material_specular{0.4f,0.4f,0.4f,1000.0f};
    scm::math::vec3f ambient_light_color{0.1f,0.1f,0.1f};
    scm::math::vec4f point_light_color{1.0f,1.0f,1.0f,1.2f};
    bool heatmap{false};
    float heatmap_min{0.0f};
    float heatmap_max{0.05f};
    scm::math::vec3f background_color{LAMURE_DEFAULT_COLOR_R,LAMURE_DEFAULT_COLOR_G,LAMURE_DEFAULT_COLOR_B};
    scm::math::vec3f heatmap_color_min{68.0f/255.0f,0.0f,84.0f/255.0f};
    scm::math::vec3f heatmap_color_max{251.0f/255.0f,231.0f/255.0f,35.0f/255.0f};
    std::string atlas_file;
    std::string json;
    std::string pvs;
    std::string background_image;
    std::vector<std::string> models;
    std::vector<uint32_t> selection;
    std::map<uint32_t, scm::math::mat4d> transforms;
    std::map<uint32_t, std::shared_ptr<lamure::prov::octree>> octrees;
    std::map<uint32_t, std::vector<lamure::prov::aux::view>> views;
    std::map<uint32_t, std::string> aux;
    float max_radius{std::min(std::numeric_limits<float>::max(),0.1f)};
    float scale_radius{1.5f};
    std::vector<float> bvh_color{1.0f,1.0f,0.0f,1.0f};
    std::vector<float> frustum_color{0.0f,0.0f,0.0f,1.0f};
};
using namespace opencover;

class LamurePointCloud_cli : public opencover::coVRPlugin,
    public opencover::ui::Owner
{
public:
    LamurePointCloud_cli();
    ~LamurePointCloud_cli();

    static LamurePointCloud_cli* getInstance();

    bool init() override;
    void preFrame() override;

    int loadLamureModel(const std::string& filename);


    // Getters for child classes
    lamure::ren::camera* getLamureCamera()       { return lamure_camera_; }
    osg::Camera*        getOsgCamera()           { return osg_camera_; }
    const AppSettings&  getSettings()     const  { return settings_; }
    const std::vector<bool>& getModelVisibility() const { return model_visible_; }
    uint32_t            getNumModels()     const  { return num_models_; }
    const std::map<uint32_t, scm::math::mat4d>& getModelTransforms() const { return settings_.transforms; }

    // UI callbacks
    void setModelVisibility(int modelIndex, bool visible);
    void togglePointCloudRendering(bool enabled);
    void toggleBoundingBoxRendering(bool enabled);
    void toggleFrustumRendering(bool enabled);
    void toggleCoordsRendering(bool enabled);
    void toggleTextRendering(bool enabled);
    void setSyncCamera(bool enabled)           { sync_camera_ = enabled; }
    void dumpSettings();
    void setSurfelShader(bool enabled)         { settings_.surfel_shader = enabled; }
    void setProvenance(bool enabled)           { settings_.provenance = enabled; }
    void setMaxRadius(float radius)            { settings_.max_radius = radius; }
    void setScaleRadius(float radius)          { settings_.scale_radius = radius; }
    void setLodError(float error)              { settings_.lod_error = error; }
    void setUploadBudget(size_t budget);
    void startMeasurement();
    void stopMeasurement();

private:
    void loadSettings(const std::string& filename);
    void initCamera();
    void printNodePath(osg::ref_ptr<osg::Node> pointer);
    bool init2();
    void debugPrintSettings()    const;
    void updateModelRotation();
    std::string stripWhitespace(const std::string& inString);
    scm::gl::data_format getTexFormat();
    std::string getConfigEntry(const std::string& scope);
    std::string getConfigEntry(const std::string& scope, const std::string& name);
    scm::math::mat4d loadMatrix(const std::string& filename);
    void strcpyTail(char* suffix, const char* str, char c);
    size_t queryVideoMemoryInMb();
    void initLamureShader();
    void syncCameras();
    bool readShader(const std::string& pathString, std::string& shaderString, bool keepOptionalShader);
    void createAuxResources();
    void drawResources(const lamure::context_t contextId, const lamure::view_t viewId);
    void drawBrush(scm::gl::program_ptr shader);
    void setLamureUniforms(scm::gl::program_ptr shader);
    void addStatesetUniforms(osg::ref_ptr<osg::StateSet> stateset);
    void setStatesetUniforms(osg::ref_ptr<osg::StateSet> stateset);
    void setGlUniforms(GLuint program);
    void createFramebuffers();
    void initRenderStates();
    void initTextRendering();
    void initPclResources();
    void initBoxResources();
    void initCoordResources();
    void initFrustumResources();
    void createAuxRepresentation();
    GLuint compileAndLinkShaders(const std::string& vsSource, const std::string& fsSource);
    GLuint compileAndLinkShaders(const std::string& vsSource, const std::string& gsSource, const std::string& fsSource);
    unsigned int createShader(const std::string& vertexShader, const std::string& fragmentShader, uint8_t ctxId);
    unsigned int compileShader(unsigned int type, const std::string& source, uint8_t ctxId);
    void initUniforms();
    void setPointUniforms();
    void setSurfelUniforms();
    void updateFrustumTransform(osg::ref_ptr<osg::MatrixTransform> matTrans, const osg::Vec3& translation);

    static LamurePointCloud_cli*            s_instance_;

    PointCloudRenderer_cti*                 renderer_         = nullptr;
    PointCloudUIManager_cli*                ui_manager_       = nullptr;
    Measurement*                            measurement_       = nullptr;

    osg::Node*             config_;

    osg::ref_ptr<osg::Group>            lamure_group_;
    osg::ref_ptr<osg::Camera>           osg_camera_;

    lamure::ren::camera*                lamure_camera_    = nullptr;

    AppSettings                         settings_;
    uint32_t                            num_models_       = 0;
    std::vector<bool>                   model_visible_;
    bool                                sync_camera_      = true;
    osgViewer::ViewerBase::FrameScheme  rendering_scheme_;
};

#endif // LAMURE_POINT_CLOUD_CLI_H
