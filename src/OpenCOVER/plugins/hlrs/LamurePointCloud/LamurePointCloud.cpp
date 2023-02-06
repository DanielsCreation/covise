//local
#include "LamurePointCloud.h"

//std
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <vector>
#include <algorithm>
#include <list>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <boost/regex/v4/regex.hpp>
#include <boost/regex/v4/regex_replace.hpp>

//lamure
#include <lamure/ren/config.h>
#include <lamure/ren/model_database.h>
#include <lamure/ren/cut_database.h>
#include <lamure/ren/dataset.h>
#include <lamure/ren/policy.h>


using namespace osg;
using namespace std;
using covise::coCoviseConfig;
using vrui::coInteraction;

bool rendering_ = false;
int32_t render_width_ = 1280;
int32_t render_height_ = 720;
lamure::ren::Data_Provenance data_provenance_;
std::vector<scm::math::mat4d> model_transformations_;


static FileHandler handler = { NULL, LamurePointCloudPlugin::load, LamurePointCloudPlugin::unload, "lam" };
LamurePointCloudPlugin* LamurePointCloudPlugin::plugin = nullptr;

COVERPLUGIN(LamurePointCloudPlugin)

// Constructor
LamurePointCloudPlugin::LamurePointCloudPlugin() 
: ui::Owner("LamurePointCloud",cover->ui)
{
    printf("LamurePointCloudPlugin::LamurePointCloudPlugin() \n");
}


const LamurePointCloudPlugin* LamurePointCloudPlugin::instance() const
{
    return plugin;
}


bool LamurePointCloudPlugin::init()
{
    printf("init()\n");
    if (plugin != NULL)
        return false;
    plugin = this;
    coVRFileManager::instance()->registerFileHandler(&handler);

    lamureMenu = new ui::Menu("LamureMenu",this);
    lamureMenu->setText("Lamure");
    loadMenu = new ui::Menu(lamureMenu, "Load");
    return 1;
}

int LamurePointCloudPlugin::load(const char* vis_file, osg::Group* loadParent, const char* covise_key)
{
    printf("load()\n");
    //osg::Group* g = new osg::Group;
    //loadParent->addChild(g);
    //g->setName(vis_file);
 
    assert(plugin);
    plugin->load_settings(vis_file, plugin->settings_);

    plugin->settings_.vis_ = plugin->settings_.show_normals_ ? 1
        : plugin->settings_.show_accuracy_ ? 2
        : plugin->settings_.show_output_sensitivity_ ? 3
        : plugin->settings_.channel_ > 0 ? 3 + plugin->settings_.channel_
        : 0;

    if (plugin->settings_.provenance_ && plugin->settings_.json_ != "") {
        std::cout << "json: " << plugin->settings_.json_ << std::endl;
        data_provenance_ = lamure::ren::Data_Provenance::parse_json(plugin->settings_.json_);
        std::cout << "size of provenance: " << data_provenance_.get_size_in_bytes() << std::endl;
    }

    lamure::ren::policy* policy = lamure::ren::policy::get_instance();
    policy->set_max_upload_budget_in_mb(plugin->settings_.upload_);
    policy->set_render_budget_in_mb(plugin->settings_.vram_);
    policy->set_out_of_core_budget_in_mb(plugin->settings_.ram_);
    plugin->render_width_ = plugin->settings_.width_ / plugin->settings_.frame_div_;
    plugin->render_height_ = plugin->settings_.height_ / plugin->settings_.frame_div_;
    policy->set_window_width(plugin->settings_.width_);
    policy->set_window_height(plugin->settings_.height_);

    int num_models_ = 0;
    lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
    for (const auto& input_file : plugin->settings_.models_) {
        lamure::model_t model_id = database->add_model(input_file, std::to_string(num_models_));
        plugin->model_transformations_.push_back(plugin->settings_.transforms_[num_models_] * scm::math::mat4d(scm::math::make_translation(database->get_model(num_models_)->get_bvh()->get_translation())));
        ++num_models_;
    }
    return 1;
}

void LamurePointCloudPlugin::createGeodes(Group* parent, const std::string& filename)
{
    printf("createGeodes()\n");
}


int LamurePointCloudPlugin::unload(const char* filename, const char* covise_key)
{
    return 1;
}


void LamurePointCloudPlugin::readMenuConfigData(const char* menu, vector<ImageFileEntry>& menulist, ui::Group* subMenu)
{
    coCoviseConfig::ScopeEntries entries = coCoviseConfig::getScopeEntries(menu);
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


void LamurePointCloudPlugin::load_settings(std::string const& vis_file_name, settings& settings) {

    std::ifstream vis_file(vis_file_name.c_str());

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

LamurePointCloudPlugin::~LamurePointCloudPlugin()
{
    printf("~LamurePointCloudPlugin()\n");
}


