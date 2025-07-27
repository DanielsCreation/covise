#include "LamureUI.h"
#include "Lamure.h"
#include <filesystem>
#include <lamure/ren/policy.h>
#include <lamure/ren/config.h>
#include <algorithm>
#include <iostream>

LamureUI::LamureUI(Lamure* plugin, const std::string& name) : opencover::ui::Owner(name, opencover::cover->ui), m_plugin(plugin) 
{

}

LamureUI::~LamureUI() 
{

}

void LamureUI::setupUi() {
    std::cout << "LamureUI::setupUi()" << std::endl;
    m_lamure_menu = new opencover::ui::Menu("Lamure", m_plugin);
    m_lamure_menu->setText("Lamure");
    m_lamure_menu->allowRelayout(true);

    // --- Gruppen ---
    m_selection_group = new opencover::ui::Group(m_lamure_menu, "Selection");
    m_model_group     = new opencover::ui::Group(m_lamure_menu, "Modelle");
    m_adaption_group  = new opencover::ui::Group(m_lamure_menu, "Adaption");
    m_rendering_group = new opencover::ui::Group(m_lamure_menu, "Rendering");
    m_shader_group    = new opencover::ui::Group(m_lamure_menu, "Shader");
    m_prov_group      = new opencover::ui::Group(m_lamure_menu, "Provenance");
    m_color_group     = new opencover::ui::Group(m_lamure_menu, "Coloring");
    m_measure_group   = new opencover::ui::Group(m_lamure_menu, "Measure");

    // --- Sichtbarkeiten / Misc ---
    m_pointcloud_button  = new opencover::ui::Button(m_selection_group, "pointcloud");
    m_boundingbox_button = new opencover::ui::Button(m_selection_group, "boundingboxes");
    m_frustum_button     = new opencover::ui::Button(m_selection_group, "frustum");
    m_sync_button        = new opencover::ui::Button(m_selection_group, "sync");
    m_notify_button      = new opencover::ui::Button(m_selection_group, "notify");
    m_text_button        = new opencover::ui::Button(m_selection_group, "text");
    m_dump_button        = new opencover::ui::Button(m_selection_group, "dump");

    m_pointcloud_button->setShared(true);
    m_boundingbox_button->setShared(true);
    m_frustum_button->setShared(true);
    m_sync_button->setShared(true);
    m_notify_button->setShared(true);
    m_text_button->setShared(true);
    m_dump_button->setShared(true);

    m_model_buttons.clear();
    m_model_visible.clear();
    for (uint16_t m_id = 0; m_id < m_plugin->getSettings().num_models; m_id++)
    {
        std::filesystem::path pathObj(m_plugin->getSettings().models[m_id]);
        std::string filename = pathObj.filename().string();
        std::string filename_strip = pathObj.stem().string();
        opencover::ui::Button *file_button = new opencover::ui::Button(m_model_group, filename_strip, nullptr, m_id);
        m_model_group->add(file_button);
        file_button->setShared(true);
        bool checked = m_plugin->getSettings().initial_selection.empty() ||
            (std::find(m_plugin->getSettings().initial_selection.begin(), 
                m_plugin->getSettings().initial_selection.end(), m_id) != m_plugin->getSettings().initial_selection.end());
        file_button->setState(checked);
        m_model_visible.push_back(checked);
        m_model_buttons.push_back(file_button);
        file_button->setCallback([this, m_id](bool state) { m_model_visible[m_id] = state; });
    }

    m_min_radius_slider = new opencover::ui::Slider(m_adaption_group, "min_radius");
    m_min_radius_slider->setText("Min. Radius");
    m_min_radius_slider->setBounds(0.0, m_plugin->getSettings().max_radius);
    m_min_radius_slider->setValue(m_plugin->getSettings().min_radius);
    m_min_radius_slider->setShared(true);
    m_min_radius_slider->setCallback([this](double value, bool /*released*/) {
        m_plugin->getSettings().min_radius = static_cast<float>(value);
        });

    m_max_radius_slider = new opencover::ui::Slider(m_adaption_group, "max_radius");
    m_max_radius_slider->setText("Max. Radius");
    m_max_radius_slider->setBounds(0.0, std::max(5.0f * m_plugin->getSettings().max_radius, 1.0f));
    m_max_radius_slider->setValue(m_plugin->getSettings().max_radius);
    m_max_radius_slider->setShared(true);
    m_max_radius_slider->setCallback([this](double value, bool released)
        { m_plugin->getSettings().max_radius = static_cast<float>(value); });

    m_scale_radius_slider = new opencover::ui::Slider(m_adaption_group, "scale_radius");
    m_scale_radius_slider->setText("Scale Radius");
    m_scale_radius_slider->setBounds(0.0001, m_plugin->getSettings().scale_radius * 5.0f);
    m_scale_radius_slider->setValue(m_plugin->getSettings().scale_radius);
    m_scale_radius_slider->setScale(opencover::ui::Slider::Logarithmic);
    m_scale_radius_slider->setShared(true);
    m_scale_radius_slider->setCallback([this](double value, bool released)
        { m_plugin->getSettings().scale_radius = static_cast<float>(value); });

    m_lod_error_slider = new opencover::ui::Slider(m_adaption_group, "lod_error");
    m_lod_error_slider->setText("LOD Error");
    m_lod_error_slider->setBounds(LAMURE_MIN_THRESHOLD, LAMURE_MAX_THRESHOLD);
    m_lod_error_slider->setValue(m_plugin->getSettings().lod_error);
    m_lod_error_slider->setShared(true);
    m_lod_error_slider->setCallback([this](double value, bool /*released*/) {
        m_plugin->getSettings().lod_error = static_cast<float>(value);
        });

    m_lod_button = new opencover::ui::Button(m_rendering_group, "lod_update");
    m_lod_button->setText("LOD");
    m_lod_button->setShared(true);
    m_lod_button->setState(m_plugin->getSettings().lod_update);
    m_lod_button->setCallback([this](bool state) {
        m_plugin->getSettings().lod_update = state;
        });

    // Shader-Typ (dynamisch basierend auf verfügbaren Daten)
    m_shader_choice = new opencover::ui::SelectionList(m_shader_group, "shader_type");
    m_shader_choice->setText("Shader");
    m_shader_choice->setShared(true);

    const auto& all_shaders = m_plugin->getRenderer()->getPclShader();
    bool prov_available = m_plugin->getSettings().provenance;

    std::vector<LamureRenderer::ShaderInfo> available_shaders;
    std::vector<std::string> ui_items;

    for (const auto& info : all_shaders) {
        if (info.name.find("Prov") == std::string::npos || prov_available) {
            available_shaders.push_back(info);
            ui_items.push_back(info.name);
        }
    }
    m_shader_choice->setList(ui_items);

    auto current_type = m_plugin->getSettings().shader_type;
    auto it = std::find_if(available_shaders.begin(), available_shaders.end(), 
        [current_type](const LamureRenderer::ShaderInfo& info){ return info.type == current_type; });

    if (it != available_shaders.end()) {
        m_shader_choice->select(std::distance(available_shaders.begin(), it));
    } else if (!available_shaders.empty()) {
        m_plugin->getSettings().shader_type = available_shaders[0].type;
        m_shader_choice->select(0);
    }

    m_shader_choice->setCallback([this, available_shaders](int idx){
        if (idx >= 0 && static_cast<size_t>(idx) < available_shaders.size()) {
            m_plugin->getSettings().shader_type = available_shaders[idx].type;
        }
        });


    // --- Modus-Auswahl (dynamisch) ---
    m_mode_choice = new opencover::ui::SelectionList(m_color_group, "mode");
    m_mode_choice->setText("Mode");
    m_mode_choice->setShared(true);
    
    std::vector<std::string> modes = {
        "color", "normals", "radius deviation", "output sensitivity"
    };
    if (prov_available) {
        for (int i = 1; i <= 6; ++i) {
                modes.push_back("prov" + std::to_string(i));
            }
    }
    m_mode_choice->setList(modes);

    auto &s = m_plugin->getSettings();
    int initial_mode_idx = 0;
    if (s.show_normals) {
        initial_mode_idx = 1;
    } else if (s.show_radius_deviation) {
        initial_mode_idx = 2;
    } else if (s.show_output_sensitivity) {
        initial_mode_idx = 3;
    } else if (s.channel >= 1 && s.channel <= 6) {
        if (prov_available) {
            initial_mode_idx = 3 + s.channel;
        }
    }
    m_mode_choice->select(initial_mode_idx);
    m_mode_choice->setCallback([this](int idx){
        auto &st = m_plugin->getSettings();
        st.show_normals            = false;
        st.show_radius_deviation   = false;
        st.show_output_sensitivity = false;
        st.channel                 = 0;
        switch(idx){
        case 0: /* color */ break;
        case 1: st.show_normals = true; break;
        case 2: st.show_radius_deviation = true; break;
        case 3: st.show_output_sensitivity = true; break;
        default:
            st.channel = idx - 3;
            break;
        }
        });

    m_accuracy_btn = new opencover::ui::Button(m_color_group, "show_accuracy");
    m_accuracy_btn->setText("accuracy");
    m_accuracy_btn->setShared(true);
    m_accuracy_btn->setState(s.show_accuracy);
    m_accuracy_btn->setCallback([this](bool b){ m_plugin->getSettings().show_accuracy = b; });

    m_heatmap_btn = new opencover::ui::Button(m_color_group, "heatmap");
    m_heatmap_btn->setText("heatmap");
    m_heatmap_btn->setShared(true);
    m_heatmap_btn->setState(m_plugin->getSettings().heatmap);
    m_heatmap_btn->setCallback([this](bool b){ m_plugin->getSettings().heatmap = b; });

    m_heatmap_min_slider = new opencover::ui::Slider(m_color_group, "heatmap_min");
    m_heatmap_min_slider->setText("heatmap min");
    m_heatmap_min_slider->setBounds(-1.0, m_plugin->getSettings().heatmap_max);
    m_heatmap_min_slider->setValue(m_plugin->getSettings().heatmap_min);
    m_heatmap_min_slider->setShared(true);
    m_heatmap_min_slider->setCallback([this](double v, bool){ m_plugin->getSettings().heatmap_min = (float)v; });

    m_heatmap_max_slider = new opencover::ui::Slider(m_color_group, "heatmap_max");
    m_heatmap_max_slider->setText("heatmap max");
    m_heatmap_max_slider->setBounds(m_plugin->getSettings().heatmap_min, m_plugin->getSettings().heatmap_min + 10.0f);
    m_heatmap_max_slider->setValue(m_plugin->getSettings().heatmap_max);
    m_heatmap_max_slider->setShared(true);
    m_heatmap_max_slider->setCallback([this](double v, bool){ m_plugin->getSettings().heatmap_max = (float)v; });

    auto addColorSliders = [this](opencover::ui::Group *grp, const std::string &prefix, scm::math::vec3f &col){
        auto *r = new opencover::ui::Slider(grp, prefix+"_r");
        r->setText(prefix+" R"); r->setBounds(0.0,1.0); r->setValue(col.x); r->setShared(true);
        r->setCallback([this,&col](double v,bool){ col.x=(float)v; });

        auto *g = new opencover::ui::Slider(grp, prefix+"_g");
        g->setText(prefix+" G"); g->setBounds(0.0,1.0); g->setValue(col.y); g->setShared(true);
        g->setCallback([this,&col](double v,bool){ col.y=(float)v; });

        auto *b = new opencover::ui::Slider(grp, prefix+"_b");
        b->setText(prefix+" B"); b->setBounds(0.0,1.0); b->setValue(col.z); b->setShared(true);
        b->setCallback([this,&col](double v,bool){ col.z=(float)v; });
        };

    addColorSliders(m_color_group, "heatmap_min_color", m_plugin->getSettings().heatmap_color_min);
    addColorSliders(m_color_group, "heatmap_max_color", m_plugin->getSettings().heatmap_color_max);

    m_measure_button = new opencover::ui::Button(m_measure_group, "measurement");
    m_measure_button->setShared(true);
    m_measure_button->setState(false);
    m_measure_button->setCallback([this](bool state) {
        if (state)
            m_plugin->startMeasurement();
        else
            m_plugin->stopMeasurement();
        });
}


