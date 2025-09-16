#define GLFW_EXPOSE_NATIVE_WIN32
//local
#include "Lamure.h" 
#include "gl_state.h"
#include "osg_util.h"
//#include "LamurePointCloudInteractor.h"

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
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include <winbase.h>
#include <mutex>
#include <filesystem>
#include <memory>
#include <algorithm>
#include <cmath>

//boost
#include <boost/regex.hpp>
#include <regex>


//schism
#include <scm/core/math.h>

//lamure
#include <lamure/pvs/pvs_database.h>
#include <lamure/prov/prov_aux.h>
#include <lamure/prov/octree.h>
#include "lamure/ren/controller.h"
#include <lamure/ren/cut.h>
#include <lamure/ren/policy.h>

#include <config/coConfigConstants.h>
#include <config/CoviseConfig.h>

#include <cover/VRSceneGraph.h>
#include "cover/OpenCOVER.h"
#include <cover/VRViewer.h>
#include <cover/coVRFileManager.h>
#include <cover/coVRPluginSupport.h>
#include <cover/coVRConfig.h>

#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>


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
//static opencover::FileHandler handler = {NULL, Lamure::loadLMR, Lamure::unloadLMR, "lmr"};

Lamure::Lamure() : opencover::ui::Owner("Lamure", opencover::cover->ui)
{
	//opencover::coVRFileManager::instance()->registerFileHandler(&handler);
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
	//opencover::coVRFileManager::instance()->unregisterFileHandler(&handler);
	//opencover::cover->getObjectsRoot()->removeChild(LamureGroup);
}

int Lamure::unloadLMR(const char* filename, const char* covise_key)
{
	return 1;
}

namespace {
    inline std::string getStr(const char* path, const std::string& def){
        return covise::coCoviseConfig::getEntry(std::string("value"), std::string(path), def, nullptr);
    }

    template<typename T> T getNum(const char* attr, const char* path, T def);

    template<> inline int getNum<int>(const char* attr, const char* path, int def){
        return covise::coCoviseConfig::getInt(std::string(attr), std::string(path), def);
    }
    template<> inline float getNum<float>(const char* attr, const char* path, float def){
        return covise::coCoviseConfig::getFloat(std::string(attr), std::string(path), def);
    }
    template<> inline double getNum<double>(const char* attr, const char* path, double def){
        return static_cast<double>(covise::coCoviseConfig::getFloat(std::string(attr), std::string(path), static_cast<float>(def)));
    }

    inline bool getOn(const char* path, bool def){
        return covise::coCoviseConfig::isOn(std::string("value"), std::string(path), def);
    }
} // namespace

void Lamure::loadSettingsFromCovise(){
    auto& s = m_settings;
    const char* root = "COVER.Plugin.LamurePointCloud";

    // ---- Budgets / LODs ----
    s.frame_div = getNum<int>("value", (std::string(root) + ".frame_div").c_str(), s.frame_div);
    s.vram      = getNum<int>("value", (std::string(root) + ".vram").c_str(),      s.vram);
    s.ram       = getNum<int>("value", (std::string(root) + ".ram").c_str(),       s.ram);
    s.upload    = getNum<int>("value", (std::string(root) + ".upload").c_str(),    s.upload);
    s.lod_error = getNum<float>("value", (std::string(root) + ".lod_error").c_str(), s.lod_error);

    // ---- Tuning / Flags, die bisher nur im LMR-Pfad gesetzt wurden ----
    s.face_eye            = getOn((std::string(root) + ".face_eye").c_str(),            s.face_eye);
    s.pvs_culling         = getOn((std::string(root) + ".pvs_culling").c_str(),         s.pvs_culling);
    s.use_pvs             = getOn((std::string(root) + ".use_pvs").c_str(),             s.use_pvs);
    s.create_aux_resources= getOn((std::string(root) + ".create_aux_resources").c_str(),s.create_aux_resources);
    s.max_brush_size      = getNum<int>("value", (std::string(root) + ".max_brush_size").c_str(), s.max_brush_size);
    s.channel             = getNum<int>("value", (std::string(root) + ".channel").c_str(), s.channel);

    // Aux-Parameter (inkl. scale, das im LMR-Parser bisher nicht vorkam)
    s.aux_point_size      = getNum<float>("value", (std::string(root) + ".aux_point_size").c_str(),     s.aux_point_size);
    s.aux_point_distance  = getNum<float>("value", (std::string(root) + ".aux_point_distance").c_str(), s.aux_point_distance);
    s.aux_point_scale     = getNum<float>("value", (std::string(root) + ".aux_point_scale").c_str(),    s.aux_point_scale);
    s.aux_focal_length    = getNum<float>("value", (std::string(root) + ".aux_focal_length").c_str(),   s.aux_focal_length);

    // ---- Visual toggles ----
    s.show_pointcloud         = getOn((std::string(root) + ".show_pointcloud").c_str(),        s.show_pointcloud);
    s.show_boundingbox        = getOn((std::string(root) + ".show_boundingbox").c_str(),       s.show_boundingbox);
    s.show_frustum            = getOn((std::string(root) + ".show_frustum").c_str(),           s.show_frustum);
    s.show_coord              = getOn((std::string(root) + ".show_coord").c_str(),             s.show_coord);
    s.show_text               = getOn((std::string(root) + ".show_text").c_str(),              s.show_text);
    s.show_sync               = getOn((std::string(root) + ".show_sync").c_str(),              s.show_sync);
    s.show_notify             = getOn((std::string(root) + ".show_notify").c_str(),            s.show_notify);

    s.show_normals            = getOn((std::string(root) + ".show_normals").c_str(),           s.show_normals);
    s.show_accuracy           = getOn((std::string(root) + ".show_accuracy").c_str(),          s.show_accuracy);
    s.show_radius_deviation   = getOn((std::string(root) + ".show_radius_deviation").c_str(),  s.show_radius_deviation);
    s.show_output_sensitivity = getOn((std::string(root) + ".show_output_sensitivity").c_str(),s.show_output_sensitivity);
    s.show_sparse             = getOn((std::string(root) + ".show_sparse").c_str(),            s.show_sparse);
    s.show_views              = getOn((std::string(root) + ".show_views").c_str(),             s.show_views);
    s.show_photos             = getOn((std::string(root) + ".show_photos").c_str(),            s.show_photos);
    s.show_octrees            = getOn((std::string(root) + ".show_octrees").c_str(),           s.show_octrees);
    s.show_bvhs               = getOn((std::string(root) + ".show_bvhs").c_str(),              s.show_bvhs);
    s.show_pvs                = getOn((std::string(root) + ".show_pvs").c_str(),               s.show_pvs);

    // ---- Lighting / ToneMapping ----
    s.point_light_intensity = getNum<float>("value", (std::string(root) + ".point_light_intensity").c_str(), s.point_light_intensity);
    s.ambient_intensity     = getNum<float>("value", (std::string(root) + ".ambient_intensity").c_str(),     s.ambient_intensity);
    s.specular_intensity    = getNum<float>("value", (std::string(root) + ".specular_intensity").c_str(),    s.specular_intensity);
    s.shininess             = getNum<float>("value", (std::string(root) + ".shininess").c_str(),             s.shininess);
    s.gamma                 = getNum<float>("value", (std::string(root) + ".gamma").c_str(),                 s.gamma);
    s.use_tone_mapping      = getOn((std::string(root) + ".use_tone_mapping").c_str(),                       s.use_tone_mapping);

    s.point_light_pos.x = getNum<float>("value", (std::string(root) + ".point_light_pos_x").c_str(), s.point_light_pos.x);
    s.point_light_pos.y = getNum<float>("value", (std::string(root) + ".point_light_pos_y").c_str(), s.point_light_pos.y);
    s.point_light_pos.z = getNum<float>("value", (std::string(root) + ".point_light_pos_z").c_str(), s.point_light_pos.z);

    // ---- Heatmap ----
    s.heatmap     = getOn((std::string(root) + ".heatmap").c_str(),     s.heatmap);
    s.heatmap_min = getNum<float>("value", (std::string(root) + ".heatmap_min").c_str(), s.heatmap_min);
    s.heatmap_max = getNum<float>("value", (std::string(root) + ".heatmap_max").c_str(), s.heatmap_max);

    auto clamp255 = [](int v){ return std::max(0, std::min(255, v)); };
    int hmin_r = getNum<int>("value", (std::string(root) + ".heatmap_min_r").c_str(), int(std::round(s.heatmap_color_min.x * 255.f)));
    int hmin_g = getNum<int>("value", (std::string(root) + ".heatmap_min_g").c_str(), int(std::round(s.heatmap_color_min.y * 255.f)));
    int hmin_b = getNum<int>("value", (std::string(root) + ".heatmap_min_b").c_str(), int(std::round(s.heatmap_color_min.z * 255.f)));
    int hmax_r = getNum<int>("value", (std::string(root) + ".heatmap_max_r").c_str(), int(std::round(s.heatmap_color_max.x * 255.f)));
    int hmax_g = getNum<int>("value", (std::string(root) + ".heatmap_max_g").c_str(), int(std::round(s.heatmap_color_max.y * 255.f)));
    int hmax_b = getNum<int>("value", (std::string(root) + ".heatmap_max_b").c_str(), int(std::round(s.heatmap_color_max.z * 255.f)));
    s.heatmap_color_min = scm::math::vec3f(clamp255(hmin_r)/255.f, clamp255(hmin_g)/255.f, clamp255(hmin_b)/255.f);
    s.heatmap_color_max = scm::math::vec3f(clamp255(hmax_r)/255.f, clamp255(hmax_g)/255.f, clamp255(hmax_b)/255.f);

    // ---- Shader / Radii ----
    s.shader       = getStr((std::string(root) + ".shader").c_str(), s.shader);
    s.min_radius   = getNum<float>("value", (std::string(root) + ".min_radius").c_str(),   s.min_radius);
    s.max_radius   = getNum<float>("value", (std::string(root) + ".max_radius").c_str(),   s.max_radius);
    s.min_screen_size   = getNum<float>("value", (std::string(root) + ".min_screen_size").c_str(),   s.min_screen_size);
    s.max_screen_size   = getNum<float>("value", (std::string(root) + ".max_screen_size").c_str(),   s.max_screen_size);
    s.scale_radius = getNum<float>("value", (std::string(root) + ".scale_radius").c_str(), s.scale_radius);
    s.max_radius_cut   = getNum<float>("value", (std::string(root) + ".max_radius_cut").c_str(),   s.max_radius_cut);
    s.scale_radius_gamma = getNum<float>("value", (std::string(root) + ".radius_scale_gamma").c_str(), s.scale_radius_gamma);
    s.scale_surfel = getNum<float>("value", (std::string(root) + ".scale_surfel").c_str(), s.scale_surfel);

    // ---- Multi-Pass Blending ----
    s.depth_range = getNum<float>("value", (std::string(root) + ".depth_range").c_str(), s.depth_range);
    s.flank_lift            = getNum<float>("value", (std::string(root) + ".flank_lift").c_str(),            s.flank_lift);

    // ---- Dateien / Pfade (Strings) ----
    s.pvs             = getStr((std::string(root) + ".pvs").c_str(),             s.pvs);
    s.background_image= getStr((std::string(root) + ".background_image").c_str(),s.background_image);

    // ---- Modelle: models (Semikolon), optional data_dir (rekursiv .bvh) ----
    s.models.clear();
    const std::string models_list = getStr((std::string(root) + ".models").c_str(), "");
    const std::string data_dir    = getStr((std::string(root) + ".data_dir").c_str(), "");
    for (const auto& m : LamureUtil::splitSemicolons(models_list))
        s.models.push_back(std::filesystem::absolute(m).string());
    if (!data_dir.empty()){
        for (auto& e : std::filesystem::recursive_directory_iterator(data_dir)){
            if (e.is_regular_file() && e.path().extension() == ".bvh")
                s.models.push_back(std::filesystem::absolute(e.path()).string());
        }
    }

    const std::string sel = getStr((std::string(root) + ".initial_selection").c_str(), "");
    auto parseIndices = [&](const std::string& str, size_t N){
        std::vector<uint32_t> out;
        if (str.empty()) return out;
        std::istringstream ss(str);
        std::string part;
        auto trim=[&](std::string t){ auto b=t.find_first_not_of(" \t"); auto e=t.find_last_not_of(" \t");
        return (b==std::string::npos)?std::string():t.substr(b,e-b+1); };
        while(std::getline(ss,part,',')){
            part=trim(part);
            auto dash=part.find('-');
            if(dash!=std::string::npos){
                int a=std::stoi(part.substr(0,dash));
                int b=std::stoi(part.substr(dash+1));
                if(a>b) std::swap(a,b);
                for(int i=a;i<=b;++i) if(i>=0 && (size_t)i<N) out.push_back((uint32_t)i);
            } else {
                int v=std::stoi(part);
                if(v>=0 && (size_t)v<N) out.push_back((uint32_t)v);
            }
        }
        std::sort(out.begin(),out.end());
        out.erase(std::unique(out.begin(),out.end()),out.end());
        return out;
        };
    s.initial_selection = parseIndices(sel, s.models.size());

    // ---- Initial matrices (one-liners) ----
    {
        const std::string navKey  = std::string(root) + ".initial_navigation";
        const std::string viewKey = std::string(root) + ".initial_view";

        const std::string navStr  = getStr(navKey.c_str(),  "");
        const std::string viewStr = getStr(viewKey.c_str(), "");

        std::cout << "[Lamure Debug] navStr from config: \"" << navStr << "\"\n";
        std::cout << "[Lamure Debug] viewStr from config: \"" << viewStr << "\"\n";

        auto tryParse = [](const std::string& label, const std::string& s, osg::Matrixd& out)->bool{
            if (!LamureUtil::readIndexedMatrix(s, out)) {
                if (!s.empty()) // Only log error if value was present but malformed
                    std::cerr << "[Lamure] " << label << " parse failed: " << s << "\n";
                return false;
            }
            return true;
            };

        osg::Matrixd M;

        if (tryParse("initial_navigation", navStr, M)) {
            m_settings.initial_navigation = M;
            m_settings.use_initial_navigation = true;
            std::cout << "[Lamure] initial_navigation OK\n";
        }
        if (tryParse("initial_view", viewStr, M)) {
            m_settings.initial_view = M;
            m_settings.use_initial_view = true;
            std::cout << "[Lamure] initial_view OK\n";
        }

        // >>> Fallbacks aus aktueller Pose, falls in der Config nichts Gültiges stand
        if (!m_settings.use_initial_navigation) {
            if (auto* sg = opencover::VRSceneGraph::instance()) {
                m_settings.initial_navigation = sg->getTransform()->getMatrix();
                m_settings.use_initial_navigation = true;
                std::cout << "[Lamure] initial_navigation defaulted from current transform\n";
            }
        }
        if (!m_settings.use_initial_view) {
            if (auto* viewer = opencover::VRViewer::instance()) {
                if (auto* cam = viewer->getCamera()) {
                    m_settings.initial_view = cam->getViewMatrix();
                    m_settings.use_initial_view = true;
                    std::cout << "[Lamure] initial_view defaulted from current camera view\n";
                }
            }
        }
        // <<< Ende Fallback
    }

    s.measurement_dir  = getStr((std::string(root) + ".measurement_dir").c_str(),  s.measurement_dir);
    s.measurement_name = getStr((std::string(root) + ".measurement_name").c_str(), s.measurement_name);

    // --- Measurement-Segmente (geometrischer Pfad) ---
    const std::string segs = getStr((std::string(root) + ".measurement_segments").c_str(), "");
    s.measurement_segments = parseMeasurementSegments(segs);
    if (s.measurement_segments.empty()) {
        s.measurement_segments = {
            {{0,-500,0},{0,0,360},200.f,30.f},
            {{0,0,0},{45,0,360},200.f,30.f},
            {{0,-400,0},{0,0,0},200.f,30.f}
        };
        std::cout << "[Lamure] measurement_segments: using built-in fallback (3 segments)\n";
    }

    // ---- Provenance & Selektion ----
    s.json = getStr((std::string(root) + ".json").c_str(), "");
    if (!s.json.empty() && !std::filesystem::exists(s.json)) {
        std::cerr << "[Lamure] config json points to non-existing file: " << s.json << " -> ignore\n";
        s.json.clear();
    }

    // ---- Provenance-Validierung / JSON-Fallback ----
    bool prov_valid = true; std::string first_json;
    if(!s.models.empty()){
        for(const auto& model_path: s.models){
            std::filesystem::path p(model_path), prov_file=p; prov_file.replace_extension(".prov");
            std::filesystem::path json_file=p; json_file.replace_extension(".json");
            if(!std::filesystem::exists(prov_file) || !std::filesystem::exists(json_file)){ prov_valid=false; break; }
            if(first_json.empty()) first_json=json_file.string();
        }
    }
    s.provenance = prov_valid;
    if (s.json.empty() && !first_json.empty()) s.json = first_json;

    // ---- Transforms default ----
    s.transforms.clear();
    for (lamure::model_t mid=0; mid < s.models.size(); ++mid)
        s.transforms[mid] = scm::math::mat4d::identity();

    // ---- Hintergrundfarbe aus COVER ----
    s.background_color = scm::math::vec3(
        covise::coCoviseConfig::getFloat("r", "COVER.Background", 0.0f),
        covise::coCoviseConfig::getFloat("g", "COVER.Background", 0.0f),
        covise::coCoviseConfig::getFloat("b", "COVER.Background", 0.0f)
    );

    // Debug: Shader-String einmal loggen
    std::cout << "[Lamure] shader from config: \"" << s.shader << "\"\n";
}


void Lamure::dumpSettings(const char* tag){
    auto& s = m_settings;
    auto b2 = [](bool v){ return v ? "on" : "off"; };
    auto v3 = [](const scm::math::vec3f& v){ std::ostringstream o; o<<v.x<<","<<v.y<<","<<v.z; return o.str(); };
    auto ov3 = [](const osg::Vec3& v){ std::ostringstream o; o<<v.x()<<","<<v.y()<<","<<v.z(); return o.str(); };
    auto mat = [](const osg::Matrixd& m){ std::ostringstream o; o.setf(std::ios::fixed); o.precision(3);
    o<<"["<<m(0,0)<<","<<m(0,1)<<","<<m(0,2)<<","<<m(0,3)<<"; "<<m(1,0)<<","<<m(1,1)<<","<<m(1,2)<<","<<m(1,3)<<"; "<<m(2,0)<<","<<m(2,1)<<","<<m(2,2)<<","<<m(2,3)<<"; "<<m(3,0)<<","<<m(3,1)<<","<<m(3,2)<<","<<m(3,3)<<"]"; return o.str(); };
    double coverScale = opencover::cover ? (double)opencover::cover->getScale() : (double)covise::coCoviseConfig::getFloat("value","COVER.DefaultScaleFactor",1.0f);
    std::cout<<"--- Lamure::Settings "<<(tag?tag:"")<<" ---\n";
    std::cout<<"models.size="<<s.models.size()<<"; num_models="<<s.num_models<<"\n";
    for(size_t i=0;i<s.models.size();++i) std::cout<<"  ["<<i<<"] "<<s.models[i]<<"\n";
    if(!s.initial_selection.empty()){ std::cout<<"initial_selection="; for(size_t i=0;i<s.initial_selection.size();++i){ if(i) std::cout<<","; std::cout<<s.initial_selection[i]; } std::cout<<"\n"; }
    std::cout<<"provenance="<<b2(s.provenance)<<"; json="<<(s.json.empty()?"<empty>":s.json)<<"\n";
    std::cout<<"shader='"<<s.shader<<"' ("<<(int)s.shader_type<<")\n";
    std::cout<<"budgets: frame_div="<<s.frame_div<<", vramMB="<<s.vram<<", ramMB="<<s.ram<<", uploadMB="<<s.upload<<", lod_error="<<s.lod_error<<"\n";
    std::cout<<"visuals: show_pointcloud="<<b2(s.show_pointcloud)<<", show_bbox="<<b2(s.show_boundingbox)<<", show_frustum="<<b2(s.show_frustum)<<", show_coord="<<b2(s.show_coord)<<", show_text="<<b2(s.show_text)<<", show_sync="<<b2(s.show_sync)<<", show_notify="<<b2(s.show_notify)<<"\n";
    std::cout<<"radii: min="<<s.min_radius<<", max="<<s.max_radius<<", scale="<<s.scale_radius<<"\n";
    std::cout<<"lighting: ambient="<<s.ambient_intensity<<", point_I="<<s.point_light_intensity<<", specular="<<s.specular_intensity<<", shininess="<<s.shininess<<", gamma="<<s.gamma<<", tone_map="<<b2(s.use_tone_mapping)<<"\n";
    std::cout<<"light_pos=("<<v3(s.point_light_pos)<<")\n";
    std::cout<<"heatmap: on="<<b2(s.heatmap)<<", min="<<s.heatmap_min<<", max="<<s.heatmap_max<<", cmin=("<<v3(s.heatmap_color_min)<<"), cmax=("<<v3(s.heatmap_color_max)<<")\n";
    std::cout<<"background_color=("<<v3(s.background_color)<<")\n";
    std::cout<<"  cover_scale="<<coverScale<<"\n";
    std::cout << "  initial_navigation=" << LamureUtil::matConv4F(s.initial_navigation) << "\n";
    std::cout << "  initial_view=" << LamureUtil::matConv4F(s.initial_view) << "\n";
    std::cout<<"  initial_view=" << LamureUtil::matConv4F(s.initial_view) << "\n";
    std::cout<<"transforms: count="<<s.transforms.size()<<"\n";
    size_t shown=0; for(const auto& kv : s.transforms){ if(shown++>=3){ std::cout<<"  ...\n"; break; } std::cout<<"  ["<<kv.first<<"] scm::mat4d present\n"; }
    std::cout<<"aux: octrees="<<s.octrees.size()<<", views="<<s.views.size()<<", aux="<<s.aux.size()<<"\n";
    std::cout << "measurement_dir='" << s.measurement_dir << "' measurement_name='" << s.measurement_name << "'\n";
    if(!s.measurement_segments.empty()){
        std::cout<<"measurement_segments.count="<<s.measurement_segments.size()<<"\n";
        const size_t N=std::min<size_t>(s.measurement_segments.size(),3);
        for(size_t i=0;i<N;++i){
            const auto& q=s.measurement_segments[i];
            std::cout<<"  ["<<i<<"] tra("<<q.tra.x()<<","<<q.tra.y()<<","<<q.tra.z()
                <<") rot("<<q.rot.x()<<","<<q.rot.y()<<","<<q.rot.z()
                <<") vT="<<q.transSpeed<<" vR="<<q.rotSpeed<<"\n";
        }
        if(s.measurement_segments.size()>3) std::cout<<"  ...\n";
    }
    std::cout<<"--- end settings ---\n";
}


bool Lamure::init2() {
	std::cout << "init2()" << std::endl;

    plugin->loadSettingsFromCovise();
    dumpSettings();

    if (plugin->m_settings.provenance && plugin->m_settings.json != "") {
        std::cout << "json: " << plugin->m_settings.json << std::endl;
        if (plugin->m_settings.provenance && !plugin->m_settings.json.empty()) {
            std::cout << "Provenance data is valid. Loading from: " << plugin->m_settings.json << std::endl;
            plugin->m_data_provenance = lamure::ren::Data_Provenance::parse_json(plugin->m_settings.json);
            std::cout << "size of provenance: " << plugin->m_data_provenance.get_size_in_bytes() << std::endl;
        }
        else { std::cout << "Provenance data not found or incomplete. Disabling provenance-based shaders." << std::endl; }
    }

    const osg::GraphicsContext::Traits *traits = opencover::coVRConfig::instance()->windows[0].context->getTraits();
    uint32_t render_width = traits->width / plugin->m_settings.frame_div;
    uint32_t render_height = traits->height / plugin->m_settings.frame_div;

    lamure::ren::policy* policy = lamure::ren::policy::get_instance();
    policy->set_max_upload_budget_in_mb(plugin->m_settings.upload);
    policy->set_render_budget_in_mb(plugin->m_settings.vram);
    policy->set_out_of_core_budget_in_mb(plugin->m_settings.ram);
    policy->set_window_width(render_width);
    policy->set_window_height(render_height);

    lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
    lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
    lamure::ren::controller* controller = lamure::ren::controller::get_instance();

    uint16_t num_models = 0;
    for (const auto &input_file : plugin->m_settings.models)
    {
        lamure::model_t model_id = database->add_model(input_file, std::to_string(num_models));
        plugin->m_model_info.model_transformations.push_back(plugin->m_settings.transforms[num_models] * scm::math::mat4d(scm::math::make_translation(database->get_model(num_models)->get_bvh()->get_translation())));
        ++num_models;
    }
    plugin->m_settings.num_models = num_models;
	std::cerr << "hostname: " << covise::coConfigConstants::getHostname() << std::endl;
    //osg_util::waitForOpenGLContext();
	m_ui->setupUi();
    m_renderer->init();
    opencover::cover->getObjectsRoot()->addChild(m_renderer->getGroup());

	//interactor = new LamurePointCloudInteractor();
	//osg::ref_ptr<opencover::IntersectionHandler> handler = interactor;
	//opencover::coIntersection::instance()->addHandler(handler);
    opencover::coVRNavigationManager::instance()->setNavMode("Point");
    if (m_settings.use_initial_view || m_settings.use_initial_navigation)
        plugin->applyInitialTransforms();
    return 1;
}

void Lamure::key(int type, int keySym, int /*mod*/) {
    const int idx = clampKeyIndex(keySym);
    if (type == osgGA::GUIEventAdapter::KEYDOWN) m_keyDown_.set(idx, true);
    else if (type == osgGA::GUIEventAdapter::KEYUP) m_keyDown_.set(idx, false);
    // kein return – Signatur ist void
}

void Lamure::preFrame() {
    if (_measurement && !_measurement->isRunning()) {
        stopMeasurement();
    }
    float deltaTime = std::clamp(float(opencover::cover->frameDuration()), 1.0f/60.0f, 1.0f/15.0f);
    float moveAmount = 1000.0f * deltaTime;
    osg::Matrix m = opencover::VRSceneGraph::instance()->getTransform()->getMatrix();

#ifdef _WIN32
    if (GetAsyncKeyState(VK_NUMPAD4) & 0x8000) m.postMult(osg::Matrix::translate(+moveAmount, 0.0, 0.0));
    if (GetAsyncKeyState(VK_NUMPAD6) & 0x8000) m.postMult(osg::Matrix::translate(-moveAmount, 0.0, 0.0));
    if (GetAsyncKeyState(VK_NUMPAD8) & 0x8000) m.postMult(osg::Matrix::translate(0.0, -moveAmount, 0.0));
    if (GetAsyncKeyState(VK_NUMPAD5) & 0x8000) m.postMult(osg::Matrix::translate(0.0, +moveAmount, 0.0));
#endif

    opencover::VRSceneGraph::instance()->getTransform()->setMatrix(m);
}
void Lamure::startMeasurement() {
    std::cout << "startMeasurement(): " << m_ui->getMeasureButton()->state() << std::endl;
    if (m_settings.measurement_segments.empty()) { std::cerr << "[Lamure] No measurement segments.\n"; return; }

    if (m_settings.use_initial_navigation || m_settings.use_initial_view) {
        applyInitialTransforms(); // setzt initial_navigation / initial_view, wenn aktiv
    }

    rendering_scheme = opencover::VRViewer::instance()->getRunFrameScheme();
    opencover::VRViewer::instance()->setRunFrameScheme(osgViewer::Viewer::CONTINUOUS);

    const std::string outFile = buildMeasurementOutputPath();
    std::cout << "[Lamure] Measurement output: " << outFile << std::endl;

    _measurement = std::make_unique<LamureMeasurement>(this, opencover::VRViewer::instance(), m_settings.measurement_segments, outFile);
}


void Lamure::stopMeasurement() {
	std::cout << "stopMeasurement(): " << m_ui->getMeasureButton()->state() << std::endl;
    if (!_measurement) return;
    if (opencover::VRViewer::instance() && opencover::VRViewer::instance()->getCamera()) {
        opencover::VRViewer::instance()->getCamera()->setPreDrawCallback(nullptr);
        opencover::VRViewer::instance()->getCamera()->setPostDrawCallback(nullptr);
    }
    _measurement->stop();
    _measurement->writeLogAndStop();
	opencover::VRViewer::instance()->setRunFrameScheme(rendering_scheme);
	_measurement.reset();
	m_ui->getMeasureButton()->setState(false);
}


struct KeyHoldHandler : public osgGA::GUIEventHandler {
    // ausreichend groß (OSG Keycodes sind < 512)
    std::bitset<512> down;

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override {
        using Key = osgGA::GUIEventAdapter;
        const int k = ea.getKey();

        // Nur Tastatur-Events behandeln
        if (ea.getEventType() == Key::KEYDOWN) {
            down.set(k, true);
            // Pfeiltasten & Numpad-Kursor vor allen anderen konsumieren
            if (k == Key::KEY_Left  || k == Key::KEY_Right ||
                k == Key::KEY_Up    || k == Key::KEY_Down  ||
                k == Key::KEY_KP_Left  || k == Key::KEY_KP_Right ||
                k == Key::KEY_KP_Up    || k == Key::KEY_KP_Down)
                return true; // niemand sonst bekommt die Taste
        }
        else if (ea.getEventType() == Key::KEYUP) {
            down.set(k, false);
            if (k == Key::KEY_Left  || k == Key::KEY_Right ||
                k == Key::KEY_Up    || k == Key::KEY_Down  ||
                k == Key::KEY_KP_Left  || k == Key::KEY_KP_Right ||
                k == Key::KEY_KP_Up    || k == Key::KEY_KP_Down)
                return true;
        }
        return false; // andere Events nicht blocken
    }

    inline bool held(int k) const { return k >= 0 && k < (int)down.size() && down.test(k); }
};

void Lamure::applyInitialTransforms(){
    auto* viewer = opencover::VRViewer::instance();
    auto* cam    = viewer ? viewer->getCamera() : nullptr;
    if(!opencover::cover || !cam) return;

    if(m_settings.use_initial_navigation){
        opencover::VRSceneGraph::instance()->getTransform()->setMatrix(m_settings.initial_navigation);
    }

    if(m_settings.use_initial_view){
        cam->setViewMatrix(m_settings.initial_view);
    }
}


std::vector<LamureMeasurement::Segment>
Lamure::parseMeasurementSegments(const std::string& cfg) const {
    std::vector<LamureMeasurement::Segment> out;
    if (cfg.empty()) return out;
    const auto trim = [](const std::string& s)->std::string {
        const char* ws = " \t\r\n";
        size_t b = s.find_first_not_of(ws), e = s.find_last_not_of(ws);
        return (b == std::string::npos) ? std::string() : s.substr(b, e - b + 1);
        };
    const auto split = [&](const std::string& s, char sep)->std::vector<std::string> {
        std::vector<std::string> v; std::string tok; std::stringstream ss(s);
        while (std::getline(ss, tok, sep)) v.push_back(trim(tok));
        return v;
        };
    const auto parseVec3 = [&](const std::string& s, osg::Vec3& v)->bool {
        auto t = split(s, ','); if (t.size() != 3) return false;
        try { v.set(std::stof(t[0]), std::stof(t[1]), std::stof(t[2])); }
        catch (...) { return false; }
        return true;
        };
    const auto parseF = [&](const std::string& s, float& f)->bool {
        try { f = std::stof(s); } catch (...) { return false; }
        return true;
        };
    const auto segs = split(cfg, ';');
    out.reserve(segs.size());
    for (const auto& segStr : segs) {
        if (segStr.empty()) continue;
        auto parts = split(segStr, '|');
        if (parts.size() != 4) { 
            std::cerr << "[Lamure] Bad segment (need 4 parts): \"" << segStr << "\"\n"; 
            continue; 
        }
        osg::Vec3 tra, rot; float vt = 0.f, vr = 0.f;
        if (!parseVec3(parts[0], tra) || !parseVec3(parts[1], rot) || !parseF(parts[2], vt) || !parseF(parts[3], vr)) {
            std::cerr << "[Lamure] Bad segment tokens: \"" << segStr << "\" (dx,dy,dz|rx,ry,rz|v_trans|v_rot)\n";
            continue;
        }
        out.push_back(LamureMeasurement::Segment{tra, rot, vt, vr});
    }

    return out;
}


std::string Lamure::buildMeasurementOutputPath() const {
    namespace fs = std::filesystem;
    fs::path dir = m_settings.measurement_dir.empty() ? fs::current_path() : fs::path(m_settings.measurement_dir);
    std::error_code ec; fs::create_directories(dir, ec); if (ec) std::cerr << "[Lamure] create_directories " << dir << " failed: " << ec.message() << "\n";

    std::string name = m_settings.measurement_name.empty() ? "measurement.txt" : m_settings.measurement_name;
    fs::path np(name);
    std::string ext = np.has_extension() ? np.extension().string() : ".txt"; if (ext.empty() || ext[0] != '.') ext = "." + ext;
    std::string stem = np.stem().string();

    // ggf. manuell gesetztes _NNNN am Ende entfernen
    stem = std::regex_replace(stem, std::regex("_(\\d+)$"), "");

    auto rxEscape = [](const std::string& s){ std::string r; r.reserve(s.size()*2);
    for(char c: s){ switch(c){ case '.': case '^': case '$': case '|': case '(': case ')': case '[': case ']': case '*': case '+': case '?': case '{': case '}': case '\\': r.push_back('\\'); default: r.push_back(c);} } return r; };

    // 1) Höchste Nummer aus ANY stem_####*.* (Frames/Summary/etc. inklusive)
    const std::regex pat("^" + rxEscape(stem) + "_(\\d+)(?:$|[^0-9].*)", std::regex::icase);
    unsigned maxN = 0;
    for (const auto& de : fs::directory_iterator(dir)) {
        if (!de.is_regular_file()) continue;
        const std::string fn = de.path().filename().string();
        std::smatch m;
        if (std::regex_search(fn, m, pat) && m.size() >= 2) {
            try { unsigned v = static_cast<unsigned>(std::stoul(m[1].str())); if (v > maxN) maxN = v; } catch (...) {}
        }
    }

    // 2) Kollisionen vermeiden: prüfe Prefix stem_#### gegen alle Dateien
    auto anyWithPrefix = [&](unsigned n)->bool{
        std::ostringstream os; os << stem << '_' << std::setw(4) << std::setfill('0') << n;
        const std::string pref = os.str();
        for (const auto& de : fs::directory_iterator(dir)) {
            if (!de.is_regular_file()) continue;
            const std::string fn = de.path().filename().string();
            if (fn.rfind(pref, 0) == 0) return true; // beginnt mit Prefix?
        }
        return false;
        };

    unsigned next = maxN + 1;
    while (anyWithPrefix(next)) ++next;

    std::ostringstream num; num << std::setw(4) << std::setfill('0') << next;
    fs::path out = dir / (stem + "_" + num.str() + ext);
    return fs::absolute(out).string();
}