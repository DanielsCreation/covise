//local
#include "LamurePointCloud.h"

using namespace osg;
using namespace std;
using namespace opencover;

LamurePointCloudPlugin* LamurePointCloudPlugin::plugin = NULL;

COVERPLUGIN(LamurePointCloudPlugin)

static const int NUM_HANDLERS = 1;

// Constructor
LamurePointCloudPlugin::LamurePointCloudPlugin()
{
    fprintf(stderr, "LamurePlugin::LamurePlugin\n");
    plugin = this;

    for (int index = 0; index < NUM_HANDLERS; index++)
        coVRFileManager::instance()->registerFileHandler(&handlers[index]);
}

static const FileHandler handlers[] = {
    { NULL,
      LamurePointCloudPlugin::loadVIS,
      LamurePointCloudPlugin::unloadVIS,
      "vis" }
};


LamurePointCloudPlugin::~LamurePointCloudPlugin()
{
    fprintf(stderr, "LamurePlugin::~LamurePlugin\n");
    for (int index = 0; index < NUM_HANDLERS; index++)
        coVRFileManager::instance()->unregisterFileHandler(&handlers[index]);
}

bool LamurePointCloudPlugin::init()
{
    if (plugin != NULL)
    {
        return false;
    }

    

    return 1;
}


std::string const strip_whitespace(std::string const& in_string) {
    return boost::regex_replace(in_string, boost::regex("^ +| +$|( ) +"), "$1");
}


scm::math::mat4d load_matrix(const std::string& filename) {
    std::ifstream file(filename);
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


struct settings {
    int32_t width_{ 1920 };
    int32_t height_{ 1080 };
    int32_t frame_div_{ 1 };
    int32_t vram_{ 2048 };
    int32_t ram_{ 4096 };
    int32_t upload_{ 32 };
    bool provenance_{ 0 };
    bool create_aux_resources_{ 1 };
    float near_plane_{ 0.001f };
    float far_plane_{ 1000.0f };
    float fov_{ 30.0f };
    bool splatting_{ 1 };
    bool gamma_correction_{ 1 };
    int32_t gui_{ 1 };
    int32_t travel_{ 2 };
    float travel_speed_{ 20.5f };
    int32_t max_brush_size_{ 4096 };
    bool lod_update_{ 1 };
    bool use_pvs_{ 1 };
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


void load_settings(std::string const& filename, settings& settings) {

    std::ifstream vis_file(filename.c_str());

    if (!vis_file.is_open()) {
        std::cout << "could not open vis file" << std::endl;
        exit(-1);
    }
    else {
        lamure::model_t model_id = 0;

        std::string line;
        while (std::getline(vis_file, line)) {
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

                    settings.models_.push_back(model);
                    settings.transforms_[model_id] = scm::math::mat4d::identity();
                    settings.aux_[model_id] = "";
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
                            settings.transforms_[address] = load_matrix(value);
                            std::cout << "found transform for model id " << address << std::endl;
                        }
                        else if (key == "aux") {
                            settings.aux_[address] = value;
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
                        settings.width_ = std::max(atoi(value.c_str()), 64);
                    }
                    else if (key == "height") {
                        settings.height_ = std::max(atoi(value.c_str()), 64);
                    }
                    else if (key == "frame_div") {
                        settings.frame_div_ = std::max(atoi(value.c_str()), 1);
                    }
                    else if (key == "vram") {
                        settings.vram_ = std::max(atoi(value.c_str()), 8);
                    }
                    else if (key == "ram") {
                        settings.ram_ = std::max(atoi(value.c_str()), 8);
                    }
                    else if (key == "upload") {
                        settings.upload_ = std::max(atoi(value.c_str()), 8);
                    }
                    else if (key == "near") {
                        settings.near_plane_ = std::max(atof(value.c_str()), 0.0);
                    }
                    else if (key == "far") {
                        settings.far_plane_ = std::max(atof(value.c_str()), 0.1);
                    }
                    else if (key == "fov") {
                        settings.fov_ = std::max(atof(value.c_str()), 9.0);
                    }
                    else if (key == "splatting") {
                        settings.splatting_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "gamma_correction") {
                        settings.gamma_correction_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "gui") {
                        settings.gui_ = std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "speed") {
                        settings.travel_speed_ = std::min(std::max(atof(value.c_str()), 0.0), 400.0);
                    }
                    else if (key == "pvs_culling") {
                        settings.pvs_culling_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "use_pvs") {
                        settings.use_pvs_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "lod_point_scale") {
                        settings.lod_point_scale_ = std::min(std::max(atof(value.c_str()), 0.0), 10.0);
                    }
                    else if (key == "aux_point_size") {
                        settings.aux_point_size_ = std::min(std::max(atof(value.c_str()), 0.00001), 1.0);
                    }
                    else if (key == "aux_point_distance") {
                        settings.aux_point_distance_ = std::min(std::max(atof(value.c_str()), 0.00001), 1.0);
                    }
                    else if (key == "aux_focal_length") {
                        settings.aux_focal_length_ = std::min(std::max(atof(value.c_str()), 0.001), 10.0);
                    }
                    else if (key == "max_brush_size") {
                        settings.max_brush_size_ = std::min(std::max(atoi(value.c_str()), 64), 1024 * 1024);
                    }
                    else if (key == "lod_error") {
                        settings.lod_error_ = std::min(std::max(atof(value.c_str()), 0.0), 10.0);
                    }
                    else if (key == "provenance") {
                        settings.provenance_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "create_aux_resources") {
                        settings.create_aux_resources_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_normals") {
                        settings.show_normals_ = std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_accuracy") {
                        settings.show_accuracy_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_radius_deviation") {
                        settings.show_radius_deviation_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_output_sensitivity") {
                        settings.show_output_sensitivity_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_sparse") {
                        settings.show_sparse_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_views") {
                        settings.show_views_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_photos") {
                        settings.show_photos_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_octrees") {
                        settings.show_octrees_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_bvhs") {
                        settings.show_bvhs_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_pvs") {
                        settings.show_pvs_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "channel") {
                        settings.channel_ = std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "enable_lighting") {
                        settings.enable_lighting_ = (bool)std::min(std::max(atoi(value.c_str()), 0), 1);
                    }
                    else if (key == "use_material_color") {
                        settings.use_material_color_ = (bool)std::min(std::max(atoi(value.c_str()), 0), 1);
                    }
                    else if (key == "material_diffuse_r") {
                        settings.material_diffuse_.x = std::max(atof(value.c_str()), 0.0);
                    }
                    else if (key == "material_diffuse_g") {
                        settings.material_diffuse_.y = std::max(atof(value.c_str()), 0.0);
                    }
                    else if (key == "material_diffuse_b") {
                        settings.material_diffuse_.z = std::max(atof(value.c_str()), 0.0);
                    }
                    else if (key == "material_specular_r") {
                        settings.material_specular_.x = std::max(atof(value.c_str()), 0.0);
                    }
                    else if (key == "material_specular_g") {
                        settings.material_specular_.y = std::max(atof(value.c_str()), 0.0);
                    }
                    else if (key == "material_specular_b") {
                        settings.material_specular_.z = std::max(atof(value.c_str()), 0.0);
                    }
                    else if (key == "material_specular_exponent") {
                        settings.material_specular_.w = std::min(std::max(atof(value.c_str()), 0.0), 10000.0);
                    }
                    else if (key == "ambient_light_color_r") {
                        settings.ambient_light_color_.r = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
                    }
                    else if (key == "ambient_light_color_g") {
                        settings.ambient_light_color_.g = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
                    }
                    else if (key == "ambient_light_color_b") {
                        settings.ambient_light_color_.b = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
                    }
                    else if (key == "point_light_color_r") {
                        settings.point_light_color_.r = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
                    }
                    else if (key == "point_light_color_g") {
                        settings.point_light_color_.g = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
                    }
                    else if (key == "point_light_color_b") {
                        settings.point_light_color_.b = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
                    }
                    else if (key == "point_light_intensity") {
                        settings.point_light_color_.w = std::min(std::max(atof(value.c_str()), 0.0), 10000.0);
                    }
                    else if (key == "background_color_r") {
                        settings.background_color_.x = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
                    }
                    else if (key == "background_color_g") {
                        settings.background_color_.y = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
                    }
                    else if (key == "background_color_b") {
                        settings.background_color_.z = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
                    }
                    else if (key == "heatmap") {
                        settings.heatmap_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "heatmap_min") {
                        settings.heatmap_min_ = std::max(atof(value.c_str()), 0.0);
                    }
                    else if (key == "heatmap_max") {
                        settings.heatmap_max_ = std::max(atof(value.c_str()), 0.0);
                    }
                    else if (key == "heatmap_min_r") {
                        settings.heatmap_color_min_.x = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
                    }
                    else if (key == "heatmap_min_g") {
                        settings.heatmap_color_min_.y = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
                    }
                    else if (key == "heatmap_min_b") {
                        settings.heatmap_color_min_.z = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
                    }
                    else if (key == "heatmap_max_r") {
                        settings.heatmap_color_max_.x = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
                    }
                    else if (key == "heatmap_max_g") {
                        settings.heatmap_color_max_.y = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
                    }
                    else if (key == "heatmap_max_b") {
                        settings.heatmap_color_max_.z = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
                    }
                    else if (key == "atlas_file") {
                        settings.atlas_file_ = value;
                    }
                    else if (key == "json") {
                        settings.json_ = value;
                    }
                    else if (key == "pvs") {
                        settings.pvs_ = value;
                    }
                    else if (key == "selection") {
                        settings.selection_ = value;
                    }
                    else if (key == "background_image") {
                        settings.background_image_ = value;
                    }
                    else if (key == "use_view_tf") {
                        settings.use_view_tf_ = std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "view_tf") {
                        settings.view_tf_ = load_matrix(value);
                    }
                    else if (key == "max_radius") {
                        settings.max_radius_ = std::max(atof(value.c_str()), 0.0);
                    }
                    else {
                        std::cout << "unrecognized key: " << key << std::endl;
                        exit(-1);
                    }

                    std::cout << key << " : " << value << std::endl;
                }

            }
        }
        vis_file.close();
    }
}


int LamurePointCloudPlugin::loadVIS(const char* vis_filename, osg::Group* loadParent, const char* covise_key)
{
    if (vis_filename)
    {
        settings settings_;
        plugin->load_settings(vis_filename, settings_);

    }
    return 0;
}


int LamurePointCloudPlugin::unloadVIS(const char* filename, const char* covise_key)
{
    return 0;
}


