#include "LamurePointCloud_cli.h"


#include "osg_util.h"

#include <lamure/ren/model_database.h>
#include <lamure/ren/cut_database.h>
#include <lamure/ren/controller.h>
#include <lamure/ren/policy.h>
#include <lamure/pvs/pvs_database.h>
#include <lamure/prov/prov_aux.h>
#include <lamure/vt/pre/AtlasFile.h>
#include <lamure/prov/octree.h>
#include <lamure/vt/VTConfig.h>
#include <lamure/vt/ren/CutDatabase.h>
#include <lamure/vt/ren/CutUpdate.h>
#include <lamure/utils.h>
#include "lamure/ren/data_provenance.h"
#include <lamure/config.h>
#include <lamure/ren/cut.h>

#include <config/coConfigConstants.h>
#include <config/coConfigLog.h>
#include <config/coConfig.h>
#include <config/coConfigString.h>
#include <config/coConfigEntryString.h>

#include <cover/coVRPluginSupport.h>
#include <cover/VRViewer.h>
#include <cover/coVRFileManager.h>
#include <cover/VRSceneGraph.h>
#include <cover/OpenCOVER.h>
#include <cover/VRWindow.h>
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
#include <osgText/Text>

#include <util/coExport.h>
#include <PluginUtil/FeedbackManager.h>
#include <PluginUtil/ModuleInteraction.h>
#include <OpenVRUI/coButtonInteraction.h>
#include <config/CoviseConfig.h>

#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include <filesystem>
#include <set>

#include <boost/assign/list_of.hpp>
#include <boost/regex.hpp>
#include <boost/thread.hpp>

#include <scm/time.h>
#include <scm/core/math.h>
#include <scm/core/io/tools.h>
#include <scm/core/pointer_types.h>
#include <scm/gl_util/primitives/primitives_fwd.h>
#include <scm/gl_util/primitives.h>
#include <scm/gl_core/buffer_objects/scoped_buffer_map.h>

#define GLFW_EXPOSE_NATIVE_WIN32

COVERPLUGIN(LamurePointCloud_cli)

LamurePointCloud_cli* LamurePointCloud_cli::s_instance_ = nullptr;

static opencover::FileHandler file_handler_ = {
    nullptr,
    [](const char* filename, osg::Group* parent, const char*) {
        return LamurePointCloud_cli::getInstance()->loadLamureModel(filename);
    },
    [](const char*, const char*) { return 1; },
    "lmr"
};

LamurePointCloud_cli::LamurePointCloud_cli() : ui::Owner("LamurePointCloud_cli", cover->ui)
{
    coVRFileManager::instance()->registerFileHandler(&file_handler_);
    s_instance_ = this;
}

// Destructor: wir löschen nur geownerte Objekte und die Wurzel-UI
LamurePointCloud_cli::~LamurePointCloud_cli()
{
    coVRFileManager::instance()->unregisterFileHandler(&file_handler_);

    delete renderer_;
    delete ui_manager_;
}

// Singleton-Zugriff
LamurePointCloud_cli* LamurePointCloud_cli::getInstance()
{
    return s_instance_;
}

// Initialization
bool LamurePointCloud_cli::init2()
{
    std::cout << "init()" << std::endl;

    config_ = opencover::coVRFileManager::instance()->loadFile(getConfigEntry("COVER.Plugin.LamurePointCloud").c_str());
    std::cerr << "hostname: " << covise::coConfigConstants::getHostname() << std::endl;

    // Scene-Graph
    lamure_group_ = new osg::Group();
    lamure_group_->setName("LamureGroup");
    cover->getObjectsRoot()->addChild(lamure_group_);

    // UI
    ui_manager_ = new PointCloudUIManager_cli(s_instance_);
    ui_manager_->createUi();


    // Initial visibility
    model_visible_.assign(num_models_, false);

    // Callbacks, Kamera, OpenGL-Setup etc.
    initCamera();

    return true;
}

void LamurePointCloud_cli::debugPrintSettings() const
{
    std::cout << "[DEBUG] Current Settings:" << std::endl;
    std::cout << "  frame_div: " << settings_.frame_div << std::endl;
    std::cout << "  vram: " << settings_.vram << std::endl;
    std::cout << "  ram: " << settings_.ram << std::endl;
    std::cout << "  upload: " << settings_.upload << std::endl;
    std::cout << "  provenance: " << settings_.provenance << std::endl;
    std::cout << "  surfel_shader: " << settings_.surfel_shader << std::endl;
    std::cout << "  lod_error: " << settings_.lod_error << std::endl;
    std::cout << "  point_size_factor: " << settings_.point_size_factor << std::endl;
    std::cout << "  max_radius: " << settings_.max_radius << std::endl;
    std::cout << "  scale_radius: " << settings_.scale_radius << std::endl;
    std::cout << "  models:" << std::endl;
    for (const auto& model : settings_.models) {
        std::cout << "    - " << model << std::endl;
    }
    std::cout << "  selection:" << std::endl;
    for (const auto& sel : settings_.selection) {
        std::cout << "    - " << sel << std::endl;
    }
    std::cout << "  transforms:" << std::endl;
    for (const auto& pair : settings_.transforms) {
        std::cout << "    - " << pair.first << ": " << pair.second[0][0] << " ..." << std::endl;
    }
}

void LamurePointCloud_cli::updateFrustumTransform(osg::ref_ptr<osg::MatrixTransform> matrixTransform, const osg::Vec3& translation) {
    osg::Matrix transMatrix = osg::Matrix::translate(translation);
    matrixTransform->setMatrix(transMatrix);
}


void LamurePointCloud_cli::initCamera()
{
    osg_camera_ = opencover::VRViewer::instance()->getCamera();
    renderer_->lmr_ctx_ = osg_camera_->getGraphicsContext()->getState()->getContextID();
    lamure_camera_ = new lamure::ren::camera(0, "lamure_camera");
}

void LamurePointCloud_cli::printNodePath(osg::ref_ptr<osg::Node> pointer)
{
    osg::NodePathList npl = pointer->getParentalNodePaths();
    int path_size = npl.size();
    std::cout << pointer->className() << " at level " << path_size << std::endl;
    if (path_size > 0) {
        for (int j = 0; j < npl[0].size(); j++) {
            std::cout << "[" << j << "] " << npl[0][j]->className() << ":  " << npl[0][j]->getName() << std::endl;
        }
        std::cout << "" << std::endl;
    }
    std::cout << "" << std::endl;
}


void LamurePointCloud_cli::preFrame()
{
    float deltaTime = std::clamp(float(opencover::cover->frameDuration()), 1.0f / 60.0f, 1.0f / 15.0f);
    float moveAmount = 1000.0f * deltaTime;

    osg::Matrix oldMat = opencover::VRSceneGraph::instance()->getTransform()->getMatrix();

    if (GetAsyncKeyState(VK_NUMPAD4) & 0x8000)
    {
        oldMat.postMult(osg::Matrix::translate(moveAmount, 0.0, 0.0));
    }
    if (GetAsyncKeyState(VK_NUMPAD6) & 0x8000)
    {
        oldMat.postMult(osg::Matrix::translate(-moveAmount, 0.0, 0.0));
    }
    if (GetAsyncKeyState(VK_NUMPAD8) & 0x8000)
    {
        oldMat.postMult(osg::Matrix::translate(0.0, -moveAmount, 0.0));
    }
    if (GetAsyncKeyState(VK_NUMPAD5) & 0x8000)
    {
        oldMat.postMult(osg::Matrix::translate(0.0, moveAmount, 0.0));
    }

    opencover::VRSceneGraph::instance()->getTransform()->setMatrix(oldMat);
}

int LamurePointCloud_cli::loadLamureModel(const std::string& filename)
{
    loadSettings(filename);

    lamure::ren::policy* policy = lamure::ren::policy::get_instance();
    policy->set_max_upload_budget_in_mb(settings_.upload);
    policy->set_render_budget_in_mb(settings_.vram);
    policy->set_out_of_core_budget_in_mb(settings_.ram);

    lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
    for (const auto& model_file : settings_.models) {
        database->add_model(model_file, std::to_string(num_models_++));
    }

    ui_manager_->setModelButtons(num_models_, settings_.models, settings_.selection);

    return 1;
}

scm::math::mat4d LamurePointCloud_cli::loadMatrix(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "could not open matrix file: " << filename << std::endl;
        std::exit(EXIT_FAILURE);
    }
    scm::math::mat4d mat;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            file >> mat[j * 4 + i];
        }
    }
    file.close();
    return mat;
}

void LamurePointCloud_cli::loadSettings(const std::string& filename) {
	using namespace std::filesystem;
	std::cout << "load_settings()" << std::endl;
	auto parseIndices = [](const std::string &s, size_t max_index) {
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
		auto not_ws = [](char c) { return !std::isspace(c); };
		s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_ws));
		s.erase(std::find_if(s.rbegin(), s.rend(), not_ws).base(), s.end());
		return s;
	};

	std::ifstream file(filename);
	if (!file.is_open()) {
		std::cerr << "could not open lmr file: " << filename << std::endl;
		std::exit(EXIT_FAILURE);
	}
	std::vector<std::string> lines;
	for (std::string line; std::getline(file, line); ) lines.push_back(line);
	file.close();

	settings_.models.clear();
	settings_.transforms.clear();
	settings_.json.clear();

	std::set<std::string> unique_models;
	std::string data_dir;
	for (const auto& raw : lines) {
		std::string l = strip_ws(raw);
		if (l.empty() || l[0] == '#') continue;
		size_t colon = l.find(':');
		if (colon <= 1) {
			if (exists(l)) {
				if (is_directory(l)) {
					data_dir = l;
				} else {
					unique_models.insert(absolute(l).string());
				}
			}
		}
	}

	if (!data_dir.empty()) {
		for (const auto& e : recursive_directory_iterator(data_dir)) {
			if (e.is_regular_file() && e.path().extension() == ".bvh") {
				unique_models.insert(absolute(e.path()).string());
			}
		}
	}

	for (const auto& path : unique_models) { settings_.models.push_back(path); }

	prov_valid = true;
	std::string first_json;
	for (const auto& model_path : settings_.models) {
		std::filesystem::path p(model_path);
		std::filesystem::path prov_file = p;
		prov_file.replace_extension(".prov");
		std::filesystem::path json_file = p;
		json_file.replace_extension(".json");
		if (!exists(prov_file) || !exists(json_file)) {
			prov_valid = false;
			break;
		} else {
			if (first_json.empty()) {
				first_json = json_file.string();
			}
		}
	}

	lamure::model_t model_id = 0;
	for (const auto& model_path : settings_.models) {
		if (settings_.transforms.find(model_id) == settings_.transforms.end()) {
			settings_.transforms[model_id] = scm::math::mat4d::identity();
		}
		++model_id;
	}

	for (const auto& raw : lines) {
		auto l = strip_ws(raw);
		if (l.empty() || l[0] == '#') continue;
		auto colon = l.find(':');
		if (colon <= 1 || (colon == std::string::npos)) continue;
		auto key = strip_ws(l.substr(0, colon));
		auto value = strip_ws(l.substr(colon + 1));

		if (!key.empty() && key[0] == '@') {
			auto ws = l.find_first_of(' ');
			uint32_t addr = std::atoi(strip_ws(l.substr(1, ws - 1)).c_str());
			key = strip_ws(l.substr(ws + 1, colon - (ws + 1)));
			if (key == "tf") settings_.transforms[addr] = loadMatrix(value);
			else {
				std::cerr << "unrecognized @-key: " << key << std::endl;
				std::exit(EXIT_FAILURE);
			}
		}
		else if (key == "surfel_shader")        settings_.surfel_shader = std::atoi(value.c_str());
		else if (key == "frame_div")            settings_.frame_div = std::max(std::atoi(value.c_str()), 1);
		else if (key == "vram")                 settings_.vram = std::max(std::atoi(value.c_str()), 8);
		else if (key == "ram")                  settings_.ram = std::max(std::atoi(value.c_str()), 8);
		else if (key == "upload")               settings_.upload = std::max(std::atoi(value.c_str()), 8);
		else if (key == "face_eye")             settings_.face_eye = std::max(std::atoi(value.c_str()), 0) != 0;
		else if (key == "gamma_correction")     settings_.gamma_correction = std::max(std::atoi(value.c_str()), 0) != 0;
		else if (key == "pvs_culling")          settings_.pvs_culling = std::max(std::atoi(value.c_str()), 0) != 0;
		else if (key == "use_pvs")              settings_.use_pvs = std::max(std::atoi(value.c_str()), 0) != 0;
		else if (key == "point_size_factor")    settings_.point_size_factor =  std::atoi(value.c_str());
		else if (key == "surfel_size_factor")   settings_.surfel_size_factor =  std::atoi(value.c_str());
		else if (key == "aux_point_size")       settings_.aux_point_size = std::clamp(std::atof(value.c_str()), 0.00001, 1.0);
		else if (key == "aux_point_distance")   settings_.aux_point_distance = std::clamp(std::atof(value.c_str()), 0.00001, 1.0);
		else if (key == "aux_focal_length")     settings_.aux_focal_length = std::clamp(std::atof(value.c_str()), 0.001, 10.0);
		else if (key == "max_brush_size")       settings_.max_brush_size = std::clamp(std::atoi(value.c_str()), 64, 1024 * 1024);
		else if (key == "lod_error")            settings_.lod_error = std::clamp(std::atof(value.c_str()), 1.0, 10.0);
		else if (key == "provenance")			settings_.provenance = (std::max(std::atoi(value.c_str()), 0) != 0) && prov_valid;
		else if (key == "createAuxResources") settings_.createAuxResources = std::max(std::atoi(value.c_str()), 0) != 0;
		else if (key == "show_normals")         settings_.show_normals = std::max(std::atoi(value.c_str()), 0) != 0;
		else if (key == "show_accuracy")        settings_.show_accuracy = std::max(std::atoi(value.c_str()), 0) != 0;
		else if (key == "show_radius_deviation")settings_.show_radius_deviation = std::max(std::atoi(value.c_str()), 0) != 0;
		else if (key == "show_output_sensitivity") settings_.show_output_sensitivity = std::max(std::atoi(value.c_str()), 0) != 0;
		else if (key == "show_sparse")          settings_.show_sparse = std::max(std::atoi(value.c_str()), 0) != 0;
		else if (key == "show_views")           settings_.show_views = std::max(std::atoi(value.c_str()), 0) != 0;
		else if (key == "show_photos")          settings_.show_photos = std::max(std::atoi(value.c_str()), 0) != 0;
		else if (key == "show_octrees")         settings_.show_octrees = std::max(std::atoi(value.c_str()), 0) != 0;
		else if (key == "show_bvhs")            settings_.show_bvhs = std::max(std::atoi(value.c_str()), 0) != 0;
		else if (key == "show_pvs")             settings_.show_pvs = std::max(std::atoi(value.c_str()), 0) != 0;
		else if (key == "channel")              settings_.channel = std::max(std::atoi(value.c_str()), 0);
		else if (key == "enable_lighting")      settings_.enable_lighting = std::clamp(std::atoi(value.c_str()), 0, 1) != 0;
		else if (key == "use_material_color")   settings_.use_material_color = std::clamp(std::atoi(value.c_str()), 0, 1) != 0;
		else if (key == "material_diffuse_r")   settings_.material_diffuse.x = std::max<float>(std::atof(value.c_str()), 0.0f);
		else if (key == "material_diffuse_g")   settings_.material_diffuse.y = std::max<float>(std::atof(value.c_str()), 0.0f);
		else if (key == "material_diffuse_b")   settings_.material_diffuse.z = std::max<float>(std::atof(value.c_str()), 0.0f);
		else if (key == "material_specular_r")  settings_.material_specular.x = std::max<float>(std::atof(value.c_str()), 0.0f);
		else if (key == "material_specular_g")  settings_.material_specular.y = std::max<float>(std::atof(value.c_str()), 0.0f);
		else if (key == "material_specular_b")  settings_.material_specular.z = std::max<float>(std::atof(value.c_str()), 0.0f);
		else if (key == "material_specular_exponent") settings_.material_specular.w = std::clamp(std::atof(value.c_str()), 0.0, 10000.0);
		else if (key == "ambient_light_color_r")settings_.ambient_light_color.r = std::clamp<float>(std::atof(value.c_str()), 0.0f, 1.0f);
		else if (key == "ambient_light_color_g")settings_.ambient_light_color.g = std::clamp<float>(std::atof(value.c_str()), 0.0f, 1.0f);
		else if (key == "ambient_light_color_b")settings_.ambient_light_color.b = std::clamp<float>(std::atof(value.c_str()), 0.0f, 1.0f);
		else if (key == "point_light_color_r")  settings_.point_light_color.r = std::clamp<float>(std::atof(value.c_str()), 0.0f, 1.0f);
		else if (key == "point_light_color_g")  settings_.point_light_color.g = std::clamp<float>(std::atof(value.c_str()), 0.0f, 1.0f);
		else if (key == "point_light_color_b")  settings_.point_light_color.b = std::clamp<float>(std::atof(value.c_str()), 0.0f, 1.0f);
		else if (key == "point_light_intensity")settings_.point_light_color.w = std::clamp<float>(std::atof(value.c_str()), 0.0f, 10000.0);
		else if (key == "background_color_r")   settings_.background_color.x = std::min(std::max(std::atoi(value.c_str()), 0), 255) / 255.0f;
		else if (key == "background_color_g")   settings_.background_color.y = std::min(std::max(std::atoi(value.c_str()), 0), 255) / 255.0f;
		else if (key == "background_color_b")   settings_.background_color.z = std::min(std::max(std::atoi(value.c_str()), 0), 255) / 255.0f;
		else if (key == "heatmap")              settings_.heatmap = std::max(std::atoi(value.c_str()), 0) != 0;
		else if (key == "heatmap_min")          settings_.heatmap_min = std::max<float>(std::atof(value.c_str()), 0.0f);
		else if (key == "heatmap_max")          settings_.heatmap_max = std::max<float>(std::atof(value.c_str()), 0.0f);
		else if (key == "heatmap_min_r")        settings_.heatmap_color_min.x = std::min(std::max(std::atoi(value.c_str()), 0), 255) / 255.0f;
		else if (key == "heatmap_min_g")        settings_.heatmap_color_min.y = std::min(std::max(std::atoi(value.c_str()), 0), 255) / 255.0f;
		else if (key == "heatmap_min_b")        settings_.heatmap_color_min.z = std::min(std::max(std::atoi(value.c_str()), 0), 255) / 255.0f;
		else if (key == "heatmap_max_r")        settings_.heatmap_color_max.x = std::min(std::max(std::atoi(value.c_str()), 0), 255) / 255.0f;
		else if (key == "heatmap_max_g")        settings_.heatmap_color_max.y = std::min(std::max(std::atoi(value.c_str()), 0), 255) / 255.0f;
		else if (key == "heatmap_max_b")        settings_.heatmap_color_max.z = std::min(std::max(std::atoi(value.c_str()), 0), 255) / 255.0f;
		else if (key == "json")					settings_.json = !value.empty() ? value : (!first_json.empty() ? first_json : settings_.json);
		else if (key == "selection")            settings_.selection = parseIndices(value, settings_.models.size());
		else if (key == "pvs")                  settings_.pvs = value;
		else if (key == "background_image")     settings_.background_image = value;
		else if (key == "max_radius")           settings_.max_radius = std::max(std::atof(value.c_str()), 0.1);
		else if (key == "scale_radius")         settings_.scale_radius = std::max(std::atof(value.c_str()), 0.1);
		else { std::cerr << "unrecognized key: " << key << std::endl; std::exit(EXIT_FAILURE); }
	}
	for (auto const& m : settings_.models) { std::cout << "Loaded model: " << m << std::endl; }
	for (auto const& m : settings_.selection) { std::cout << "Selected models: " << m << std::endl; }
}

void LamurePointCloud_cli::togglePointcloudRendering(bool enabled)
{
    pointcloud_geode_->setNodeMask(enabled ? ~0 : 0);
}

void LamurePointCloud_cli::toggleBoundingBoxRendering(bool enabled)
{
    boundingbox_geode_->setNodeMask(enabled ? ~0 : 0);
}

void LamurePointCloud_cli::toggleFrustumRendering(bool enabled)
{
    frustum_geode_->setNodeMask(enabled ? ~0 : 0);
}

void LamurePointCloud_cli::toggleCoordsRendering(bool enabled)
{
    coord_geode_->setNodeMask(enabled ? ~0 : 0);
}

void LamurePointCloud_cli::toggleTextRendering(bool enabled)
{
    text_geode_->setNodeMask(enabled ? ~0 : 0);
}

void LamurePointCloud_cli::dumpSettings()
{
    std::cout << "Dumping settings..." << std::endl;
    // Add actual dump logic here
}

void LamurePointCloud_cli::setUploadBudget(size_t budget)
{
    settings_.upload = budget;
    lamure::ren::policy::get_instance()->set_max_upload_budget_in_mb(budget);
}

void LamurePointCloud_cli::startMeasurement()
{
    std::cout << "Starting measurement..." << std::endl;
    // Add actual measurement start logic here
}

void LamurePointCloud_cli::stopMeasurement()
{
    std::cout << "Stopping measurement..." << std::endl;
    // Add actual measurement stop logic here
}

void LamurePointCloud_cli::setModelVisibility(int model_index, bool visible)
{
    if (model_index < model_visible_.size())
    {
        model_visible_[model_index] = visible;
    }
}

std::vector<float> LamurePointCloud_cli::getBoxCorners(scm::gl::boxf bbv) {
    std::vector<float> corners_ = {
        bbv.corner(0).data_array[0], bbv.corner(0).data_array[1], bbv.corner(0).data_array[2],
        bbv.corner(1).data_array[0], bbv.corner(1).data_array[1], bbv.corner(1).data_array[2],
        bbv.corner(2).data_array[0], bbv.corner(2).data_array[1], bbv.corner(2).data_array[2],
        bbv.corner(3).data_array[0], bbv.corner(3).data_array[1], bbv.corner(3).data_array[2],
        bbv.corner(4).data_array[0], bbv.corner(4).data_array[1], bbv.corner(4).data_array[2],
        bbv.corner(5).data_array[0], bbv.corner(5).data_array[1], bbv.corner(5).data_array[2],
        bbv.corner(6).data_array[0], bbv.corner(6).data_array[1], bbv.corner(6).data_array[2],
        bbv.corner(7).data_array[0], bbv.corner(7).data_array[1], bbv.corner(7).data_array[2],
    };
    return corners_;
}

std::vector<std::vector<float>> LamurePointCloud_cli::getSerializedBvhMinMax(const std::vector<scm::gl::boxf> bounding_boxes) {
    std::vector<std::vector<float>> vecOfVec;
    for (uint64_t node_id = 0; node_id < bounding_boxes.size(); ++node_id) {
        scm::math::vec3f min_vertex = bounding_boxes[node_id].min_vertex();
        scm::math::vec3f max_vertex = bounding_boxes[node_id].max_vertex();
        std::vector<float> elements{
            min_vertex.x, min_vertex.y, min_vertex.z,
            max_vertex.x, max_vertex.y, max_vertex.z };
        vecOfVec.push_back(elements);
    }
    return vecOfVec;
}

void LamurePointCloud_cli::startMeasurement() {
    std::cout << "startMeasurement(): " << ui_manager_->getMeasureButton()->state() << std::endl;
    std::vector<Measurement::Segment> _segments = {
        { {0,-500,0},    {0,0,360},        200.0f, 30.0f },
        { {0,0,0},        {45,0,360},        200.0f,    30.0f },
        { {0,-400,0},    {0,0,0},        200.0f,    30.0f },
    };
    auto rendering_scheme = opencover::VRViewer::instance()->getRunFrameScheme();
    opencover::VRViewer::instance()->setRunFrameScheme(osgViewer::Viewer::CONTINUOUS);
    measurement_ = new Measurement(opencover::VRViewer::instance(), _segments, "C:/Users/Daniel/Documents/Studium/Forschungsarbeit/Measurement/measurement1.txt", measure_button_, _measureCB);
}

void LamurePointCloud_cli::stopMeasurement() {
    std::cout << "stopMeasurement(): " << ui_manager_->getMeasureButton()->state() << std::endl;
    opencover::VRViewer::instance()->setRunFrameScheme(rendering_scheme_);
    measurement_->writeLogAndStop();
    ui_manager_->getMeasureButton()->setState(false);
    delete _measurement;
    _measurement = nullptr;
}

void LamurePointCloud_cli::initSchismObjects() {
    if (!device_) {
        device_.reset(new scm::gl::render_device());
        if (!device_) { std::cout << "error creating device" << std::endl; }
        if (notify_button_->state()) {
            std::cout << "[Notify] initSchismObjects()" << std::endl;
            std::ostringstream oss;
            device_->dump_memory_info(oss);
            std::cout << oss.str() << std::endl;
            scm::gl::render_device::device_capabilities capa = device_->capabilities();
        }
    }
    if (!context_) {
        context_ = device_->main_context();
        if (!context_) { std::cout << "error creating context" << std::endl; }
    }
}

void LamurePointCloud_cli::updateModelRotation() {
    // Implementation from original LamurePointCloud.cpp
}

std::string const LamurePointCloud_cli::stripWhitespace(std::string const& in_string)
{
    static const boost::regex ws_re("^ +| +$|( ) +");
    return boost::regex_replace(in_string, ws_re, "$1");
}

std::string getConfigEntry(const std::string& scope) {
    std::cout << "getConfigEntry(scope): ";
    covise::coCoviseConfig::ScopeEntries entries = covise::coCoviseConfig::getScopeEntries(scope);
    for (const auto& entry : entries)
    { return entry.second; }
    return "";
}


std::string getConfigEntry(const std::string& scope, const std::string& name) {
    std::cout << "getConfigEntry(scope, name): ";
    covise::coCoviseConfig::ScopeEntries entries = covise::coCoviseConfig::getScopeEntries(scope);
    for (const auto& entry : entries) {
        if (name == entry.first)
        { return entry.second; }
    }
    return "";
}

void LamurePointCloud_cli::strcpyTail(char* suffix, const char* str, char c)
{
    const char* p = strrchr(str, c);
    if (p && *(p+1))
        std::strcpy(suffix, p+1);
    else
        suffix[0] = '\0';
}

size_t LamurePointCloud_cli::queryVideoMemoryInMb()
{
    GLint memKb = 0;
    glGetIntegerv(GL_GPU_MEMORY_INFO_CURRENT_AVAILABLE_VIDMEM_NVX, &memKb);
    return static_cast<size_t>(memKb) / 1024;
}

void LamurePointCloud_cli::initLamureShader()
{
    if (notify_button_->state())
    {
        std::cout << "[Notify] initLamureShader()" << std::endl;
    }

    std::string shader_path_ = LAMURE_SHADERS_DIR;

    readShader(shader_path_ + "/vis_point.vs", vis_point_vs_source, false);
    readShader(shader_path_ + "/vis_point.fs", vis_point_fs_source, false);
    point_shader_.program = compileAndLinkShaders(vis_point_vs_source, vis_point_fs_source);

    readShader(shader_path_ + "/vis_surfel.vs", vis_surfel_vs_source, false);
    readShader(shader_path_ + "/vis_surfel.gs", vis_surfel_gs_source, false);
    readShader(shader_path_ + "/vis_surfel.fs", vis_surfel_fs_source, false);
    surfel_shader_.program = compileAndLinkShaders(vis_surfel_vs_source, vis_surfel_gs_source, vis_surfel_fs_source);

    readShader(shader_path_ + "/vis_line.vs", vis_line_vs_source, false);
    readShader(shader_path_ + "/vis_line.fs", vis_line_fs_source, false);
    line_shader_.program_ = compileAndLinkShaders(vis_line_vs_source, vis_line_fs_source);
}

void LamurePointCloud_cli::syncCameras()
{
    if (lamure_camera_ && osg_camera_)
    {
        syncCameras(lamure_camera_, osg_camera_.get());
    }
}

bool LamurePointCloud_cli::readShader(std::string const& path_string, std::string& shader_string, bool keep_optional_shader_code)
{
    std::ifstream stream(path_string);

    if (!stream.is_open())
    {
        std::cout << "Unable to open shader file: " << path_string << std::endl;
        return false;
    }

    std::string line;
    std::stringstream ss;

    while (getline(stream, line))
    {
        ss << line << "\n";
    }

    shader_string = ss.str();
    stream.close();

    return true;
}

void LamurePointCloud_cli::createFramebuffers()
{
    if (plugin->notify_button_->state()) { std::cout << "[Notify] createFramebuffers()" << std::endl; }

    fbo_.reset(new scm::gl::frame_buffer());
    pass1_fbo_.reset(new scm::gl::frame_buffer());
    pass2_fbo_.reset(new scm::gl::frame_buffer());
    pass3_fbo_.reset(new scm::gl::frame_buffer());

    fbo_color_buffer_.reset(new scm::gl::texture_2d(device_, render_width_, render_height_, scm::gl::FORMAT_RGBA_8, 1, 1, 1));
    fbo_depth_buffer_.reset(new scm::gl::texture_2d(device_, render_width_, render_height_, scm::gl::FORMAT_D24, 1, 1, 1));
    pass1_depth_buffer_.reset(new scm::gl::texture_2d(device_, render_width_, render_height_, scm::gl::FORMAT_D24, 1, 1, 1));
    pass2_color_buffer_.reset(new scm::gl::texture_2d(device_, render_width_, render_height_, scm::gl::FORMAT_RGBA_8, 1, 1, 1));
    pass2_normal_buffer_.reset(new scm::gl::texture_2d(device_, render_width_, render_height_, scm::gl::FORMAT_RGBA_32F, 1, 1, 1));
    pass2_view_space_pos_buffer_.reset(new scm::gl::texture_2d(device_, render_width_, render_height_, scm::gl::FORMAT_RGBA_32F, 1, 1, 1));
    pass2_depth_buffer_.reset(new scm::gl::texture_2d(device_, render_width_, render_height_, scm::gl::FORMAT_D24, 1, 1, 1));

    context_->bind_frame_buffer(fbo_);
    fbo_->attach_color_buffer(0, fbo_color_buffer_);
    fbo_->attach_depth_stencil_buffer(fbo_depth_buffer_);
    fbo_->draw_buffer(0);
    fbo_->check();

    context_->bind_frame_buffer(pass1_fbo_);
    pass1_fbo_->attach_depth_stencil_buffer(pass1_depth_buffer_);
    pass1_fbo_->check();

    context_->bind_frame_buffer(pass2_fbo_);
    pass2_fbo_->attach_color_buffer(0, pass2_color_buffer_);
    pass2_fbo_->attach_color_buffer(1, pass2_normal_buffer_);
    pass2_fbo_->attach_color_buffer(2, pass2_view_space_pos_buffer_);
    pass2_fbo_->attach_depth_stencil_buffer(pass2_depth_buffer_);
    pass2_fbo_->check();

    context_->bind_frame_buffer(pass3_fbo_);
    pass3_fbo_->attach_color_buffer(0, fbo_color_buffer_);
    pass3_fbo_->attach_depth_stencil_buffer(pass2_depth_buffer_);
    pass3_fbo_->check();

    context_->bind_frame_buffer(0);
}

void LamurePointCloud_cli::initRenderStates()
{
    if (plugin->notify_button_->state()) { std::cout << "[Notify] initRenderStates()" << std::endl; }

    scm::gl::depth_stencil_state::descriptor depth_state_less_desc;
    depth_state_less_desc.depth_test(true);
    depth_state_less_desc.depth_write(true);
    depth_state_less_desc.depth_function(scm::gl::COMPARISON_LESS);
    depth_state_less_ = device_->create_depth_stencil_state(depth_state_less_desc);

    scm::gl::depth_stencil_state::descriptor depth_state_disable_desc;
    depth_state_disable_desc.depth_test(false);
    depth_state_disable_desc.depth_write(false);
    depth_state_disable_ = device_->create_depth_stencil_state(depth_state_disable_desc);

    scm::gl::depth_stencil_state::descriptor depth_state_without_writing_desc;
    depth_state_without_writing_desc.depth_test(true);
    depth_state_without_writing_desc.depth_write(false);
    depth_state_without_writing_desc.depth_function(scm::gl::COMPARISON_LESS);
    depth_state_without_writing_ = device_->create_depth_stencil_state(depth_state_without_writing_desc);

    scm::gl::rasterizer_state::descriptor no_backface_culling_rasterizer_state_desc;
    no_backface_culling_rasterizer_state_desc.cull_mode(scm::gl::CULL_NONE);
    no_backface_culling_rasterizer_state_ = device_->create_rasterizer_state(no_backface_culling_rasterizer_state_desc);

    scm::gl::blend_state::descriptor color_blending_state_desc;
    color_blending_state_desc.blend_enable(true);
    color_blending_state_desc.blend_op(scm::gl::BLEND_OP_ADD);
    color_blending_state_desc.source_blend_rgb(scm::gl::FACTOR_SRC_ALPHA);
    color_blending_state_desc.dest_blend_rgb(scm::gl::FACTOR_ONE_MINUS_SRC_ALPHA);
    color_blending_state_ = device_->create_blend_state(color_blending_state_desc);

    scm::gl::blend_state::descriptor color_no_blending_state_desc;
    color_no_blending_state_desc.blend_enable(false);
    color_no_blending_state_ = device_->create_blend_state(color_no_blending_state_desc);

    scm::gl::sampler_state::descriptor filter_linear_desc;
    filter_linear_desc.min_filter(scm::gl::FILTER_MIN_LINEAR);
    filter_linear_desc.mag_filter(scm::gl::FILTER_MAG_LINEAR);
    filter_linear_ = device_->create_sampler_state(filter_linear_desc);

    scm::gl::sampler_state::descriptor filter_nearest_desc;
    filter_nearest_desc.min_filter(scm::gl::FILTER_MIN_NEAREST);
    filter_nearest_desc.mag_filter(scm::gl::FILTER_MAG_NEAREST);
    filter_nearest_ = device_->create_sampler_state(filter_nearest_desc);

    scm::gl::sampler_state::descriptor vt_filter_linear_desc;
    vt_filter_linear_desc.min_filter(scm::gl::FILTER_MIN_LINEAR_MIPMAP_LINEAR);
    vt_filter_linear_desc.mag_filter(scm::gl::FILTER_MAG_LINEAR);
    vt_filter_linear_ = device_->create_sampler_state(vt_filter_linear_desc);

    scm::gl::sampler_state::descriptor vt_filter_nearest_desc;
    vt_filter_nearest_desc.min_filter(scm::gl::FILTER_MIN_NEAREST_MIPMAP_NEAREST);
    vt_filter_nearest_desc.mag_filter(scm::gl::FILTER_MAG_NEAREST);
    vt_filter_nearest_ = device_->create_sampler_state(vt_filter_nearest_desc);
}

void LamurePointCloud_cli::initTextRendering()
{
    if (plugin->notify_button_->state()) { std::cout << "[Notify] initTextRendering()" << std::endl; }

    std::string font_path = "C:/Windows/Fonts/arial.ttf";
    text_renderer_.reset(new scm::gl::text_renderer(device_, font_path));
    if (!text_renderer_) {
        std::cout << "error creating text_renderer" << std::endl;
    }
    renderable_text_.reset(new scm::gl::text(device_));
    if (!renderable_text_) {
        std::cout << "error creating renderable_text" << std::endl;
    }
}

void LamurePointCloud_cli::initPclResources()
{
    if (plugin->notify_button_->state()) { std::cout << "[Notify] initPclResources()" << std::endl; }

    glGenVertexArrays(1, &pcl_resource_.vao_);
    glBindVertexArray(pcl_resource_.vao_);
}

void LamurePointCloud_cli::initBoxResources()
{
    if (plugin->notify_button_->state()) { std::cout << "[Notify] initBoxResources()" << std::endl; }

    lamure::ren::model_database* database = lamure::ren::model_database::get_instance();

    for (uint16_t model_id = 0; model_id < num_models_; ++model_id)
    {
        const lamure::ren::bvh* bvh = database->get_model(model_id)->get_bvh();
        std::vector<scm::gl::boxf> const& bbv = bvh->get_bounding_boxes();

        for (uint64_t node_id = 0; node_id < bbv.size(); ++node_id)
        {
            box_resource_.vertices_.push_back(getBoxCorners(bbv[node_id]));
        }
    }

    glGenVertexArrays(1, &box_resource_.vao_);
    glBindVertexArray(box_resource_.vao_);

    glGenBuffers(1, &box_resource_.vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, box_resource_.vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 24, &box_resource_.vertices_[0][0], GL_STATIC_DRAW);

    glGenBuffers(1, &box_resource_.ibo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, box_resource_.ibo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short) * box_resource_.idx_.size(), box_resource_.idx_.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
}

void LamurePointCloud_cli::initCoordResources()
{
    if (plugin->notify_button_->state()) { std::cout << "[Notify] initCoordResources()" << std::endl; }

    glGenVertexArrays(1, &coord_resource_.vao_);
    glBindVertexArray(coord_resource_.vao_);

    glGenBuffers(1, &coord_resource_.vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, coord_resource_.vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * coord_resource_.vertices_.size(), coord_resource_.vertices_.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &coord_resource_.ibo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, coord_resource_.ibo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short) * coord_resource_.idx_.size(), coord_resource_.idx_.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
}

void LamurePointCloud_cli::initFrustumResources()
{
    if (plugin->notify_button_->state()) { std::cout << "[Notify] initFrustumResources()" << std::endl; }

    glGenVertexArrays(1, &frustum_resource_.vao_);
    glBindVertexArray(frustum_resource_.vao_);

    glGenBuffers(1, &frustum_resource_.vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, frustum_resource_.vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * frustum_resource_.vertices_.size(), frustum_resource_.vertices_.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &frustum_resource_.ibo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, frustum_resource_.ibo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short) * frustum_resource_.idx_.size(), frustum_resource_.idx_.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
}

void LamurePointCloud_cli::createAuxRepresentation()
{
    // Implementation from original LamurePointCloud.cpp
}

GLuint LamurePointCloud_cli::compileAndLinkShaders(std::string vs_source, std::string fs_source)
{
    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    const char* vs_source_c = vs_source.c_str();
    glShaderSource(vs, 1, &vs_source_c, nullptr);
    glCompileShader(vs);

    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    const char* fs_source_c = fs_source.c_str();
    glShaderSource(fs, 1, &fs_source_c, nullptr);
    glCompileShader(fs);

    GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);

    glDeleteShader(vs);
    glDeleteShader(fs);

    return program;
}

GLuint LamurePointCloud_cli::compileAndLinkShaders(std::string vs_source, std::string gs_source, std::string fs_source)
{
    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    const char* vs_source_c = vs_source.c_str();
    glShaderSource(vs, 1, &vs_source_c, nullptr);
    glCompileShader(vs);

    GLuint gs = glCreateShader(GL_GEOMETRY_SHADER);
    const char* gs_source_c = gs_source.c_str();
    glShaderSource(gs, 1, &gs_source_c, nullptr);
    glCompileShader(gs);

    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    const char* fs_source_c = fs_source.c_str();
    glShaderSource(fs, 1, &fs_source_c, nullptr);
    glCompileShader(fs);

    GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, gs);
    glAttachShader(program, fs);
    glLinkProgram(program);

    glDeleteShader(vs);
    glDeleteShader(gs);
    glDeleteShader(fs);

    return program;
}

unsigned int LamurePointCloud_cli::createShader(const std::string& vertexShader, const std::string& fragmentShader, uint8_t ctx_id)
{
    if (plugin->notify_button_->state()) { std::cout << "[Notify] createShader()" << std::endl; }

    unsigned int program = glCreateProgram();
    unsigned int vs = compileShader(GL_VERTEX_SHADER, vertexShader, ctx_id);
    unsigned int fs = compileShader(GL_FRAGMENT_SHADER, fragmentShader, ctx_id);

    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glValidateProgram(program);

    glDeleteShader(vs);
    glDeleteShader(fs);

    return program;
}

unsigned int LamurePointCloud_cli::compileShader(unsigned int type, const std::string& source, uint8_t ctx_id)
{
    if (plugin->notify_button_->state()) { std::cout << "[Notify] compileShader()" << std::endl; }

    unsigned int id = glCreateShader(type);
    const char* src = source.c_str();
    glShaderSource(id, 1, &src, nullptr);
    glCompileShader(id);

    int result;
    glGetShaderiv(id, GL_COMPILE_STATUS, &result);
    if (result == GL_FALSE) {
        int length;
        glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
        char* message = (char*)alloca(length * sizeof(char));
        glGetShaderInfoLog(id, length, &length, message);
        std::cout << "Failed to compile " << (type == GL_VERTEX_SHADER ? "vertex" : "fragment") << " shader!" << std::endl;
        std::cout << message << std::endl;
        glDeleteShader(id);
        return 0;
    }

    return id;
}

void LamurePointCloud_cli::initUniforms()
{
    if (plugin->notify_button_->state()) { std::cout << "[Notify] initUniforms()" << std::endl; }

    point_shader_.mvp_matrix_loc = glGetUniformLocation(point_shader_.program, "mvp_matrix");
    point_shader_.max_radius_loc = glGetUniformLocation(point_shader_.program, "max_radius");
    point_shader_.scale_radius_loc = glGetUniformLocation(point_shader_.program, "scale_radius");
    point_shader_.point_size_factor_loc = glGetUniformLocation(point_shader_.program, "point_size_factor");
    point_shader_.proj_scale_loc = glGetUniformLocation(point_shader_.program, "proj_scale");

    surfel_shader_.mvp_matrix_loc = glGetUniformLocation(surfel_shader_.program, "mvp_matrix");
    surfel_shader_.model_view_matrix_loc = glGetUniformLocation(surfel_shader_.program, "model_view_matrix");
    surfel_shader_.max_radius_loc = glGetUniformLocation(surfel_shader_.program, "max_radius");
    surfel_shader_.scale_radius_loc = glGetUniformLocation(surfel_shader_.program, "scale_radius");
    surfel_shader_.surfel_size_factor_loc = glGetUniformLocation(surfel_shader_.program, "surfel_size_factor");
    surfel_shader_.proj_scale_loc = glGetUniformLocation(surfel_shader_.program, "proj_scale");
    surfel_shader_.viewport_loc = glGetUniformLocation(surfel_shader_.program, "viewport");

    line_shader_.mvp_matrix_location = glGetUniformLocation(line_shader_.program_, "mvp_matrix");
    line_shader_.in_color_location = glGetUniformLocation(line_shader_.program_, "in_color");
}

void LamurePointCloud_cli::setPointUniforms()
{
    glUniform1f(point_shader_.max_radius_loc, settings_.max_radius_);
    glUniform1f(point_shader_.scale_radius_loc, settings_.scale_radius_);
    glUniform1f(point_shader_.point_size_factor_loc, settings_.point_size_factor_ * cover->getScale());
    glUniform1f(point_shader_.proj_scale_loc, viewport_.y * 0.5f * projection_matrix_.data_array[5]);
}

void LamurePointCloud_cli::setSurfelUniforms()
{
    glUniform1f(surfel_shader_.max_radius_loc, settings_.max_radius_);
    glUniform1f(surfel_shader_.scale_radius_loc, settings_.scale_radius_);
    glUniform1f(surfel_shader_.surfel_size_factor_loc, settings_.surfel_size_factor_);
    glUniform1f(surfel_shader_.proj_scale_loc, viewport_.y * 0.5f * projection_matrix_.data_array[5]);
    glUniform2f(surfel_shader_.viewport_loc, viewport_.x, viewport_.y);
}

void LamurePointCloud_cli::updateFrustumTransform(osg::ref_ptr<osg::MatrixTransform> matrixTransform, const osg::Vec3& translation) {
    osg::Matrix transMatrix = osg::Matrix::translate(translation);
    matrixTransform->setMatrix(transMatrix);
}
