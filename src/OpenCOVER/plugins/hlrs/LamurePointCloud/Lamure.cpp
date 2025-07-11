#define GLFW_EXPOSE_NATIVE_WIN32
//local
#include "Lamure.h"
#include "gl_state.h"
#include "osg_util.h"
#include "LamurePointCloudInteractor.h"

#include <lamure/imgui.h>
#include <lamure/imgui_internal.h>
#include <lamure/imgui_impl_glfw_gl3.h>

// std
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
#include <winbase.h>
#include <mutex>
#include <filesystem>
#include <memory>

//boost
#include <boost/assign/list_of.hpp>
#include <boost/regex.hpp>
#include <boost/thread.hpp>

//schism
#include <scm/time.h>
//#include <scm/core.h>
#include <scm/core/math.h>
#include <scm/core/io/tools.h>
#include <scm/core/pointer_types.h>
//#include <scm/core/platform/platform.h>
//#include <scm/core/utilities/platform_warning_disable.h>
#include <scm/gl_util/primitives/primitives_fwd.h>
#include <scm/gl_util/primitives.h>
#include <scm/gl_core/buffer_objects/scoped_buffer_map.h>

//lamure
#include <lamure/pvs/pvs_database.h>
#include <lamure/prov/prov_aux.h>
#include <lamure/vt/pre/AtlasFile.h>
#include <lamure/prov/octree.h>
#include <lamure/vt/VTConfig.h>
#include <lamure/vt/ren/CutDatabase.h>
#include <lamure/vt/ren/CutUpdate.h>
#include <lamure/utils.h>
#include "lamure/ren/controller.h"
#include <lamure/config.h>
#include <lamure/ren/cut.h>

#include <config/coConfigConstants.h>
#include <config/coConfigLog.h>
#include <config/coConfig.h>
#include <config/coConfigString.h>
#include <config/coConfigEntryString.h>

#include <cover/ui/SelectionList.h>
#include <cover/coVRStatsDisplay.h>
#include <cover/VRSceneGraph.h>
#include "cover/OpenCOVER.h"
#include <cover/VRWindow.h>
#include <cover/VRViewer.h>
#include <cover/coHud.h>
#include <cover/coVRTui.h>
#include <cover/ui/Menu.h>
#include "cover/coVRCollaboration.h"
#include "cover/coIntersection.h"

#include <osgViewer/GraphicsWindow>
#include <osgViewer/Renderer>
#include <osgGA/EventQueue>
#include <osg/PolygonMode>
#include <osg/StateSet>

#include <util/coExport.h>
#include <PluginUtil/FeedbackManager.h>
#include <PluginUtil/ModuleInteraction.h>
#include <OpenVRUI/coButtonInteraction.h>
#include <config/CoviseConfig.h>

#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>
#include <cover/coVRFileManager.h>
#include <cover/coVRPluginSupport.h>


#ifdef __cplusplus
extern "C" {
#endif
	__declspec(dllexport) DWORD NvOptimusEnablement = 1;
	__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
#ifdef __cplusplus
}
#endif

COVERPLUGIN(Lamure)
Lamure* Lamure::plugin = nullptr;

static opencover::FileHandler handler = {NULL, Lamure::loadLMR, Lamure::unloadLMR, "lmr"};

Lamure::Lamure() : opencover::ui::Owner("Lamure", opencover::cover->ui)
{
	opencover::coVRFileManager::instance()->registerFileHandler(&handler);
	plugin = this;
    m_ui = std::make_unique<LamureUI>(this, "LamureUI");
    m_renderer = std::make_unique<LamureRenderer>(this);
}

Lamure* Lamure::instance()
{
	return plugin;
}

Lamure::~Lamure()
{
	fprintf(stderr, "LamurePlugin::~LamurePlugin\n");
	opencover::coVRFileManager::instance()->unregisterFileHandler(&handler);
	//opencover::cover->getObjectsRoot()->removeChild(LamureGroup);
}

int Lamure::unloadLMR(const char* filename, const char* covise_key)
{
	return 1;
}

void Lamure::loadSettings(const std::string& filename) {
    using namespace std::filesystem;
    std::cout << "load_settings()" << std::endl;

    auto parseIndices = [](const std::string& s, size_t max_index) {
        std::vector<uint32_t> out;
        if (s.empty()) {
            for (uint32_t i = 0; i < max_index; ++i) out.push_back(i);
            return out;
        }
        std::istringstream ss(s);
        std::string part;
        while (std::getline(ss, part, ',')) {
            size_t dash = part.find('-');
            if (dash != std::string::npos) {
                int32_t a = std::stoi(part.substr(0, dash));
                int32_t b = std::stoi(part.substr(dash + 1));
                for (int32_t i = a; i <= b; ++i)
                    if (i >= 0 && static_cast<size_t>(i) < max_index) out.push_back(i);
            } else {
                int32_t val = std::stoi(part);
                if (val >= 0 && static_cast<size_t>(val) < max_index) out.push_back(val);
            }
        }
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
        return out;
    };

    auto strip_ws = [](std::string s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) { return !std::isspace(ch); }));
        s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), s.end());
        return s;
    };

    // Prepare settings map
    using SettingHandler = std::function<void(const std::string&)>;
    std::unordered_map<std::string, SettingHandler> setting_handlers;
    auto& s = plugin->m_settings;

    setting_handlers["surfel_shader"]       = [&](const auto& v) { s.surfel_shader_ = (std::atoi(v.c_str()) != 0); };
    setting_handlers["frame_div"]           = [&](const auto& v) { s.frame_div_ = std::max(std::atoi(v.c_str()), 1); };
    setting_handlers["vram"]                = [&](const auto& v) { s.vram_ = std::max(std::atoi(v.c_str()), 8); };
    setting_handlers["ram"]                 = [&](const auto& v) { s.ram_ = std::max(std::atoi(v.c_str()), 8); };
    setting_handlers["upload"]              = [&](const auto& v) { s.upload_ = std::max(std::atoi(v.c_str()), 8); };
    setting_handlers["face_eye"]            = [&](const auto& v) { s.face_eye_ = (std::atoi(v.c_str()) != 0); };
    setting_handlers["gamma_correction"]    = [&](const auto& v) { s.gamma_correction_ = (std::atoi(v.c_str()) != 0); };
    setting_handlers["pvs_culling"]         = [&](const auto& v) { s.pvs_culling_ = (std::atoi(v.c_str()) != 0); };
    setting_handlers["use_pvs"]             = [&](const auto& v) { s.use_pvs_ = (std::atoi(v.c_str()) != 0); };
    setting_handlers["point_size_factor"]   = [&](const auto& v) { s.point_size_factor_ = std::stof(v.c_str()); };
    setting_handlers["surfel_size_factor"]  = [&](const auto& v) { s.surfel_size_factor_ = std::stof(v.c_str()); };
    setting_handlers["aux_point_size"]      = [&](const auto& v) { s.aux_point_size_ = std::clamp(std::stof(v.c_str()), 0.00001f, 1.0f); };
    setting_handlers["aux_point_distance"]  = [&](const auto& v) { s.aux_point_distance_ = std::clamp(std::stof(v.c_str()), 0.00001f, 1.0f); };
    setting_handlers["aux_focal_length"]    = [&](const auto& v) { s.aux_focal_length_ = std::clamp(std::stof(v.c_str()), 0.001f, 10.0f); };
    setting_handlers["max_brush_size"]      = [&](const auto& v) { s.max_brush_size_ = std::clamp(std::atoi(v.c_str()), 64, 1024 * 1024); };
    setting_handlers["lod_error"]           = [&](const auto& v) { s.lod_error_ = std::clamp(std::stof(v.c_str()), 1.0f, 10.0f); };
    setting_handlers["create_aux_resources"]= [&](const auto& v) { s.create_aux_resources_ = (std::atoi(v.c_str()) != 0); };
    setting_handlers["show_normals"]        = [&](const auto& v) { s.show_normals_ = (std::atoi(v.c_str()) != 0); };
    setting_handlers["show_accuracy"]       = [&](const auto& v) { s.show_accuracy_ = (std::atoi(v.c_str()) != 0); };
    setting_handlers["show_radius_deviation"] = [&](const auto& v) { s.show_radius_deviation_ = (std::atoi(v.c_str()) != 0); };
    setting_handlers["show_output_sensitivity"] = [&](const auto& v) { s.show_output_sensitivity_ = (std::atoi(v.c_str()) != 0); };
    setting_handlers["show_sparse"]         = [&](const auto& v) { s.show_sparse_ = (std::atoi(v.c_str()) != 0); };
    setting_handlers["show_views"]          = [&](const auto& v) { s.show_views_ = (std::atoi(v.c_str()) != 0); };
    setting_handlers["show_photos"]         = [&](const auto& v) { s.show_photos_ = (std::atoi(v.c_str()) != 0); };
    setting_handlers["show_octrees"]        = [&](const auto& v) { s.show_octrees_ = (std::atoi(v.c_str()) != 0); };
    setting_handlers["show_bvhs"]           = [&](const auto& v) { s.show_bvhs_ = (std::atoi(v.c_str()) != 0); };
    setting_handlers["show_pvs"]            = [&](const auto& v) { s.show_pvs_ = (std::atoi(v.c_str()) != 0); };
    setting_handlers["channel"]             = [&](const auto& v) { s.channel_ = std::max(std::atoi(v.c_str()), 0); };
    setting_handlers["enable_lighting"]     = [&](const auto& v) { s.enable_lighting_ = (std::clamp(std::atoi(v.c_str()), 0, 1) != 0); };
    setting_handlers["use_material_color"]  = [&](const auto& v) { s.use_material_color_ = (std::clamp(std::atoi(v.c_str()), 0, 1) != 0); };
    setting_handlers["material_diffuse_r"]  = [&](const auto& v) { s.material_diffuse_.x = std::max(std::stof(v.c_str()), 0.0f); };
    setting_handlers["material_diffuse_g"]  = [&](const auto& v) { s.material_diffuse_.y = std::max(std::stof(v.c_str()), 0.0f); };
    setting_handlers["material_diffuse_b"]  = [&](const auto& v) { s.material_diffuse_.z = std::max(std::stof(v.c_str()), 0.0f); };
    setting_handlers["material_specular_r"] = [&](const auto& v) { s.material_specular_.x = std::max(std::stof(v.c_str()), 0.0f); };
    setting_handlers["material_specular_g"] = [&](const auto& v) { s.material_specular_.y = std::max(std::stof(v.c_str()), 0.0f); };
    setting_handlers["material_specular_b"] = [&](const auto& v) { s.material_specular_.z = std::max(std::stof(v.c_str()), 0.0f); };
    setting_handlers["material_specular_exponent"] = [&](const auto& v) { s.material_specular_.w = std::clamp(std::stof(v.c_str()), 0.0f, 10000.0f); };
    setting_handlers["ambient_light_color_r"]= [&](const auto& v) { s.ambient_light_color_.r = std::clamp(std::stof(v.c_str()), 0.0f, 1.0f); };
    setting_handlers["ambient_light_color_g"]= [&](const auto& v) { s.ambient_light_color_.g = std::clamp(std::stof(v.c_str()), 0.0f, 1.0f); };
    setting_handlers["ambient_light_color_b"]= [&](const auto& v) { s.ambient_light_color_.b = std::clamp(std::stof(v.c_str()), 0.0f, 1.0f); };
    setting_handlers["point_light_color_r"] = [&](const auto& v) { s.point_light_color_.r = std::clamp(std::stof(v.c_str()), 0.0f, 1.0f); };
    setting_handlers["point_light_color_g"] = [&](const auto& v) { s.point_light_color_.g = std::clamp(std::stof(v.c_str()), 0.0f, 1.0f); };
    setting_handlers["point_light_color_b"] = [&](const auto& v) { s.point_light_color_.b = std::clamp(std::stof(v.c_str()), 0.0f, 1.0f); };
    setting_handlers["point_light_intensity"]= [&](const auto& v) { s.point_light_color_.w = std::clamp(std::stof(v.c_str()), 0.0f, 10000.0f); };
    auto parse_color = [&](const std::string& v) { return std::min(std::max(std::atoi(v.c_str()), 0), 255) / 255.0f; };
    setting_handlers["background_color_r"]  = [&](const auto& v) { s.background_color.x = parse_color(v); };
    setting_handlers["background_color_g"]  = [&](const auto& v) { s.background_color.y = parse_color(v); };
    setting_handlers["background_color_b"]  = [&](const auto& v) { s.background_color.z = parse_color(v); };
    setting_handlers["heatmap"]             = [&](const auto& v) { s.heatmap_ = (std::atoi(v.c_str()) != 0); };
    setting_handlers["heatmap_min"]         = [&](const auto& v) { s.heatmap_min_ = std::max(std::stof(v.c_str()), 0.0f); };
    setting_handlers["heatmap_max"]         = [&](const auto& v) { s.heatmap_max_ = std::max(std::stof(v.c_str()), 0.0f); };
    setting_handlers["heatmap_min_r"]       = [&](const auto& v) { s.heatmap_color_min_.x = parse_color(v); };
    setting_handlers["heatmap_min_g"]       = [&](const auto& v) { s.heatmap_color_min_.y = parse_color(v); };
    setting_handlers["heatmap_min_b"]       = [&](const auto& v) { s.heatmap_color_min_.z = parse_color(v); };
    setting_handlers["heatmap_max_r"]       = [&](const auto& v) { s.heatmap_color_max_.x = parse_color(v); };
    setting_handlers["heatmap_max_g"]       = [&](const auto& v) { s.heatmap_color_max_.y = parse_color(v); };
    setting_handlers["heatmap_max_b"]       = [&](const auto& v) { s.heatmap_color_max_.z = parse_color(v); };
    setting_handlers["pvs"]                 = [&](const auto& v) { s.pvs_ = v; };
    setting_handlers["background_image"]    = [&](const auto& v) { s.background_image_ = v; };
    setting_handlers["max_radius"]          = [&](const auto& v) { s.max_radius_ = std::max(std::stof(v.c_str()), 0.1f); };
    setting_handlers["scale_radius"]        = [&](const auto& v) { s.scale_radius_ = std::max(std::stof(v.c_str()), 0.1f); };
    setting_handlers["pointcloud_state"]    = [&](const auto& v) { s.pointcloud_state = (std::atoi(v.c_str()) != 0); };
    setting_handlers["boundingbox_state"]   = [&](const auto& v) { s.boundingbox_state = (std::atoi(v.c_str()) != 0); };
    setting_handlers["frustum_state"]       = [&](const auto& v) { s.frustum_state = (std::atoi(v.c_str()) != 0); };
    setting_handlers["coord_state"]         = [&](const auto& v) { s.coord_state = (std::atoi(v.c_str()) != 0); };
    setting_handlers["text_state"]          = [&](const auto& v) { s.text_state = (std::atoi(v.c_str()) != 0); };
    setting_handlers["sync_state"]          = [&](const auto& v) { s.sync_state = (std::atoi(v.c_str()) != 0); };
    setting_handlers["notify_state"]        = [&](const auto& v) { s.notify_state = (std::atoi(v.c_str()) != 0); };

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "could not open lmr file: " << filename << std::endl;
        std::exit(EXIT_FAILURE);
    }

    s.models.clear();
    s.transforms_.clear();
    s.json_.clear();

    std::set<std::string> unique_models;
    std::string data_dir;
    std::vector<std::string> setting_lines;

    // First pass: discover models and directories, separate setting lines
    for (std::string line; std::getline(file, line);) {
        std::string l = strip_ws(line);
        if (l.empty() || l[0] == '#') continue;

        size_t colon = l.find(':');
        if (colon <= 1 || colon == std::string::npos) {
            if (exists(l)) {
                if (is_directory(l)) {
                    data_dir = l;
                } else {
                    unique_models.insert(absolute(l).string());
                }
            }
        } else {
            setting_lines.push_back(l);
        }
    }
    file.close();

    if (!data_dir.empty()) {
        for (const auto& e : recursive_directory_iterator(data_dir)) {
            if (e.is_regular_file() && e.path().extension() == ".bvh") {
                unique_models.insert(absolute(e.path()).string());
            }
        }
    }

    for (const auto& path : unique_models) {
        s.models.push_back(path);
    }

    // Check for provenance files and find first json
    prov_valid = true;
    std::string first_json;
    if (!s.models.empty()) {
        for (const auto& model_path : s.models) {
            std::filesystem::path p(model_path);
            std::filesystem::path prov_file = p;
            prov_file.replace_extension(".prov");
            std::filesystem::path json_file = p;
            json_file.replace_extension(".json");
            if (!exists(prov_file) || !exists(json_file)) {
                prov_valid = false;
                break;
            }
            if (first_json.empty()) {
                first_json = json_file.string();
            }
        }
    }
    
    setting_handlers["provenance"] = [&](const auto& v) { s.provenance_ = (std::atoi(v.c_str()) != 0) && prov_valid; };
    setting_handlers["json"] = [&](const auto& v) { s.json_ = !v.empty() ? v : (!first_json.empty() ? first_json : s.json_); };
    setting_handlers["initial_selection"] = [&](const auto& v) { s.initial_selection = parseIndices(v, s.models.size()); };

    // Set default transforms
    for (lamure::model_t model_id = 0; model_id < s.models.size(); ++model_id) {
        s.transforms_[model_id] = scm::math::mat4d::identity();
    }

    // Second pass: parse settings
    for (const auto& l : setting_lines) {
        auto colon = l.find(':');
        auto key = strip_ws(l.substr(0, colon));
        auto value = strip_ws(l.substr(colon + 1));

        if (!key.empty() && key[0] == '@') {
            auto ws = l.find_first_of(' ');
            uint32_t addr = std::atoi(strip_ws(l.substr(1, ws - 1)).c_str());
            key = strip_ws(l.substr(ws + 1, colon - (ws + 1)));
            if (key == "tf") {
                s.transforms_[addr] = LamureUtil::loadMatrix(value);
            } else {
                std::cerr << "unrecognized @-key: " << key << std::endl;
                std::exit(EXIT_FAILURE);
            }
        } else {
            auto it = setting_handlers.find(key);
            if (it != setting_handlers.end()) {
                it->second(value);
            } else {
                std::cerr << "unrecognized key: " << key << std::endl;
                std::exit(EXIT_FAILURE);
            }
        }
    }

    for (auto const& m : s.models) { std::cout << "Loaded model: " << m << std::endl; }
    for (auto const& m : s.initial_selection) { std::cout << "Selected models: " << m << std::endl; }
}


int Lamure::loadLMR(const char* filename, osg::Group* parent, const char* covise_key) {
	std::printf("loadLMR()\n");
	assert(plugin);
	std::string lmr_file = std::string(filename);
	plugin->loadSettings(lmr_file);

	if (plugin->m_settings.provenance_ && plugin->m_settings.json_ != "") {
		std::cout << "json: " << plugin->m_settings.json_ << std::endl;
		plugin->m_data_provenance = lamure::ren::Data_Provenance::parse_json(plugin->m_settings.json_);
		std::cout << "size of provenance: " << plugin->m_data_provenance.get_size_in_bytes() << std::endl;
	}

	const osg::GraphicsContext::Traits *traits = opencover::coVRConfig::instance()->windows[0].context->getTraits();
	uint32_t render_width_ = traits->width / plugin->m_settings.frame_div_;
	uint32_t render_height_ = traits->height / plugin->m_settings.frame_div_;

	lamure::ren::policy* policy = lamure::ren::policy::get_instance();
	policy->set_max_upload_budget_in_mb(plugin->m_settings.upload_);
	policy->set_render_budget_in_mb(plugin->m_settings.vram_);
	policy->set_out_of_core_budget_in_mb(plugin->m_settings.ram_);
	policy->set_window_width(render_width_);
	policy->set_window_height(render_height_);

	lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
	lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
	lamure::ren::controller* controller = lamure::ren::controller::get_instance();

	uint16_t num_models = 0;
	for (const auto &input_file : plugin->m_settings.models)
	{
		lamure::model_t model_id = database->add_model(input_file, std::to_string(num_models));
		plugin->m_model_info.model_transformations_.push_back(plugin->m_settings.transforms_[num_models] * scm::math::mat4d(scm::math::make_translation(database->get_model(num_models)->get_bvh()->get_translation())));
		++num_models;
	}
	plugin->m_settings.num_models = num_models;
	
	return 1;
}

bool Lamure::init2() {
	std::cout << "init2()" << std::endl;
	std::cout << "Config: " << LamureUtil::getConfigEntry("COVER.Plugin.LamurePointCloud").c_str() << std::endl;
	osg::ref_ptr<osg::Node> file = opencover::coVRFileManager::instance()->loadFile(LamureUtil::getConfigEntry("COVER.Plugin.LamurePointCloud").c_str());
	std::cerr << "hostname: " << covise::coConfigConstants::getHostname() << std::endl;

	osg_util::waitForOpenGLContext();

	m_ui->setupUi();
	m_renderer->init();
	opencover::cover->getObjectsRoot()->addChild(m_renderer->getGroup());

	//interactor = new LamurePointCloudInteractor();
	//osg::ref_ptr<opencover::IntersectionHandler> handler = interactor;
	//opencover::coIntersection::instance()->addHandler(handler);
	opencover::coVRNavigationManager::instance()->setNavMode("Point");

	return 1;
}


void Lamure::preFrame() {
	float deltaTime = std::clamp(float(opencover::cover->frameDuration()), 1.0f / 60.0f, 1.0f / 15.0f);
	float moveAmount = 1000.0f * deltaTime;
	osg::Matrix oldMat = opencover::VRSceneGraph::instance()->getTransform()->getMatrix();

	if (GetAsyncKeyState(VK_NUMPAD4) & 0x8000)
		oldMat.postMult(osg::Matrix::translate(moveAmount, 0.0, 0.0));
	if (GetAsyncKeyState(VK_NUMPAD6) & 0x8000)
		oldMat.postMult(osg::Matrix::translate(-moveAmount, 0.0, 0.0));
	if (GetAsyncKeyState(VK_NUMPAD8) & 0x8000)
		oldMat.postMult(osg::Matrix::translate(0.0, -moveAmount, 0.0));
	if (GetAsyncKeyState(VK_NUMPAD5) & 0x8000)
		oldMat.postMult(osg::Matrix::translate(0.0, moveAmount, 0.0));

	opencover::VRSceneGraph::instance()->getTransform()->setMatrix(oldMat);
}


void Lamure::startMeasurement() {
	std::cout << "startMeasurement(): " << m_ui->getMeasureButton()->state() << std::endl;
	std::vector<Measurement::Segment> _segments = {
		{ {0,-500,0},	{0,0,360},		200.0f, 30.0f },
		{ {0,0,0},		{45,0,360},		200.0f,	30.0f },
		{ {0,-400,0},	{0,0,0},		200.0f,	30.0f },
	};
	rendering_scheme = opencover::VRViewer::instance()->getRunFrameScheme();
	opencover::VRViewer::instance()->setRunFrameScheme(osgViewer::Viewer::CONTINUOUS);
	_measurement = std::make_unique<Measurement>(opencover::VRViewer::instance(), _segments, "C:/Users/Daniel/Documents/Studium/Forschungsarbeit/Measurement/measurement1.txt", m_ui->getMeasureButton(), _measureCB);
}


void Lamure::stopMeasurement() {
	std::cout << "stopMeasurement(): " << m_ui->getMeasureButton()->state() << std::endl;
	opencover::VRViewer::instance()->setRunFrameScheme(rendering_scheme);
	_measurement->writeLogAndStop();
	m_ui->getMeasureButton()->setState(false);
	_measurement.reset();
}