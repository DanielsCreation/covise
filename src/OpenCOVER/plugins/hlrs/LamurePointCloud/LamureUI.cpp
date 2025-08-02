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

    // --- Rendering ---

    m_rendering_group = new opencover::ui::Group(m_lamure_menu, "Rendering");

    m_pointcloud_button  = new opencover::ui::Button(m_rendering_group, "Pointcloud");
    m_boundingbox_button = new opencover::ui::Button(m_rendering_group, "BoundingBoxes");
    m_frustum_button     = new opencover::ui::Button(m_rendering_group, "Frustum");
    m_text_button        = new opencover::ui::Button(m_rendering_group, "Text");

    m_pointcloud_button->setShared(true);
    m_boundingbox_button->setShared(true);
    m_frustum_button->setShared(true);
    m_text_button->setShared(true);

    // --- Misc ---

    m_misc_group = new opencover::ui::Group(m_lamure_menu, "Misc");

    m_sync_button        = new opencover::ui::Button(m_misc_group, "Sync");
    m_notify_button      = new opencover::ui::Button(m_misc_group, "Notify");
    m_dump_button        = new opencover::ui::Button(m_misc_group, "Dump");

    m_sync_button->setShared(true);
    m_notify_button->setShared(true);
    m_dump_button->setShared(true);


    // --- Measurement ---

    m_measure_group = new opencover::ui::Group(m_lamure_menu, "Measurement");

    m_measure_button = new opencover::ui::Button(m_measure_group, "start_measurement");
    m_measure_button->setText("Start Measurement");
    m_measure_button->setShared(true);
    m_measure_button->setState(false);
    m_measure_button->setCallback([this](bool state) {
        if (state)
            m_plugin->startMeasurement();
        else
            m_plugin->stopMeasurement();
        });


    m_model_menu = new opencover::ui::Menu(m_lamure_menu, "Model");

    m_model_buttons.clear();
    m_model_visible.clear();

    for (uint16_t m_id = 0; m_id < m_plugin->getSettings().num_models; m_id++)
    {
        std::filesystem::path pathObj(m_plugin->getSettings().models[m_id]);
        std::string filename = pathObj.filename().string();
        std::string filename_strip = pathObj.stem().string();
        opencover::ui::Button *file_button = new opencover::ui::Button(m_model_menu, filename_strip, nullptr, m_id);
        //m_model_group->add(file_button);
        file_button->setShared(true);
        bool checked = m_plugin->getSettings().initial_selection.empty() ||
            (std::find(m_plugin->getSettings().initial_selection.begin(), 
                m_plugin->getSettings().initial_selection.end(), m_id) != m_plugin->getSettings().initial_selection.end());
        file_button->setState(checked);
        m_model_visible.push_back(checked);
        m_model_buttons.push_back(file_button);
        file_button->setCallback([this, m_id](bool state) { m_model_visible[m_id] = state; });
    }

    m_point_size_menu = new opencover::ui::Menu(m_lamure_menu, "PointCloud");

    m_min_radius_slider = new opencover::ui::Slider(m_point_size_menu, "min_radius");
    m_min_radius_slider->setText("Min. Radius");
    m_min_radius_slider->setBounds(0.0, m_plugin->getSettings().max_radius);
    m_min_radius_slider->setValue(m_plugin->getSettings().min_radius);
    m_min_radius_slider->setShared(true);
    m_min_radius_slider->setCallback([this](double value, bool /*released*/) {
        m_plugin->getSettings().min_radius = static_cast<float>(value);
        });

    m_max_radius_slider = new opencover::ui::Slider(m_point_size_menu, "max_radius");
    m_max_radius_slider->setText("Max. Radius");
    m_max_radius_slider->setBounds(0.0, std::max(5.0f * m_plugin->getSettings().max_radius, 1.0f));
    m_max_radius_slider->setValue(m_plugin->getSettings().max_radius);
    m_max_radius_slider->setShared(true);
    m_max_radius_slider->setCallback([this](double value, bool released)
        { m_plugin->getSettings().max_radius = static_cast<float>(value); });

    m_scale_radius_slider = new opencover::ui::Slider(m_point_size_menu, "scale_radius");
    m_scale_radius_slider->setText("Scale Radius");
    m_scale_radius_slider->setBounds(0.0001, m_plugin->getSettings().scale_radius * 5.0f);
    m_scale_radius_slider->setValue(m_plugin->getSettings().scale_radius);
    m_scale_radius_slider->setScale(opencover::ui::Slider::Logarithmic);
    m_scale_radius_slider->setShared(true);
    m_scale_radius_slider->setCallback([this](double value, bool released)
        { m_plugin->getSettings().scale_radius = static_cast<float>(value); });


    m_lod_menu = new opencover::ui::Menu(m_lamure_menu, "LOD");

    m_lod_button = new opencover::ui::Button(m_lod_menu, "lod");
    m_lod_button->setText("LOD");
    m_lod_button->setShared(true);
    m_lod_button->setState(m_plugin->getSettings().lod_update);
    m_lod_button->setCallback([this](bool state) {
        m_plugin->getSettings().lod_update = state;
        });

    m_lod_error_slider = new opencover::ui::Slider(m_lod_menu, "lod_error");
    m_lod_error_slider->setText("LOD Error");
    m_lod_error_slider->setBounds(LAMURE_MIN_THRESHOLD, LAMURE_MAX_THRESHOLD);
    m_lod_error_slider->setValue(m_plugin->getSettings().lod_error);
    m_lod_error_slider->setShared(true);
    m_lod_error_slider->setCallback([this](double value, bool /*released*/) {
        m_plugin->getSettings().lod_error = static_cast<float>(value);
        });

    m_shader_choice = new opencover::ui::SelectionList(m_lamure_menu, "shader");
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

    m_shader_choice->setCallback([this, available_shaders](int idx) {
        if (idx >= 0 && static_cast<size_t>(idx) < available_shaders.size()) {
            auto shader_type = available_shaders[idx].type;
            m_plugin->getSettings().shader_type = shader_type;
        }
        });

    m_lighting_menu = new opencover::ui::Menu(m_lamure_menu, "Lighting");

    m_ambient_light_slider = new opencover::ui::Slider(m_lighting_menu, "ambient_light");
    m_ambient_light_slider->setText("Ambient Light");
    m_ambient_light_slider->setBounds(0.0, 1.0);
    m_ambient_light_slider->setValue(m_plugin->getSettings().ambient_light_color.x);
    m_ambient_light_slider->setShared(true);
    m_ambient_light_slider->setCallback([this](double value, bool) {
        m_plugin->getSettings().ambient_light_color = scm::math::vec3f(static_cast<float>(value));
        });

    m_light_intensity_slider = new opencover::ui::Slider(m_lighting_menu, "point_light_intensity");
    m_light_intensity_slider->setText("Point Light Intensity");
    m_light_intensity_slider->setBounds(0, 1.0);
    m_light_intensity_slider->setValue(m_plugin->getSettings().point_light_color.x);
    m_light_intensity_slider->setShared(true);
    m_light_intensity_slider->setCallback([this](double v, bool) { m_plugin->getSettings().point_light_color.a = (float)v; });

    m_material_specular_slider = new opencover::ui::Slider(m_lighting_menu, "specular_intensity");
    m_material_specular_slider->setText("Specular Intensity");
    m_material_specular_slider->setBounds(0.0, 10.0);
    m_material_specular_slider->setValue(m_plugin->getSettings().material_specular.x);
    m_material_specular_slider->setShared(true);
    m_material_specular_slider->setCallback([this](double v, bool) {
        m_plugin->getSettings().material_specular.x = (float)v;
        m_plugin->getSettings().material_specular.y = (float)v;
        m_plugin->getSettings().material_specular.z = (float)v;
        });

    m_shininess_slider = new opencover::ui::Slider(m_lighting_menu, "shininess");
    m_shininess_slider->setText("Shininess");
    m_shininess_slider->setBounds(0.0, 100.0);
    m_shininess_slider->setValue(m_plugin->getSettings().material_specular[3]);
    m_shininess_slider->setShared(true);
    m_shininess_slider->setCallback([this](double value, bool) {
        m_plugin->getSettings().material_specular[3] = static_cast<float>(value);
        });

    // Light Position Sliders
    m_light_pos_x_slider = new opencover::ui::Slider(m_lighting_menu, "light_pos_x");
    m_light_pos_x_slider->setText("Light Pos X");
    m_light_pos_x_slider->setBounds(-1000.0, 1000.0);
    m_light_pos_x_slider->setValue(m_plugin->getSettings().point_light_pos.x);
    m_light_pos_x_slider->setShared(true);
    m_light_pos_x_slider->setCallback([this](double value, bool) {
        m_plugin->getSettings().point_light_pos.x = static_cast<float>(value);
        });

    m_light_pos_y_slider = new opencover::ui::Slider(m_lighting_menu, "light_pos_y");
    m_light_pos_y_slider->setText("Light Pos Y");
    m_light_pos_y_slider->setBounds(-1000.0, 1000.0);
    m_light_pos_y_slider->setValue(m_plugin->getSettings().point_light_pos.y);
    m_light_pos_y_slider->setShared(true);
    m_light_pos_y_slider->setCallback([this](double value, bool) {
        m_plugin->getSettings().point_light_pos.y = static_cast<float>(value);
        });

    m_light_pos_z_slider = new opencover::ui::Slider(m_lighting_menu, "light_pos_z");
    m_light_pos_z_slider->setText("Light Pos Z");
    m_light_pos_z_slider->setBounds(-1000.0, 1000.0);
    m_light_pos_z_slider->setValue(m_plugin->getSettings().point_light_pos.z);
    m_light_pos_z_slider->setShared(true);
    m_light_pos_z_slider->setCallback([this](double value, bool) {
        m_plugin->getSettings().point_light_pos.z = static_cast<float>(value);
        });

    m_mode_choice = new opencover::ui::SelectionList(m_lamure_menu, "Mode");
    m_mode_choice->setText("Mode");
    m_mode_choice->setShared(true);

    std::vector<std::string> modes = {
        "Color",
        "Normals",
        "Accuracy",
        "Radius Deviation",
        "Output Sensitivity",
        "Material Color"
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
    } else if (s.show_accuracy) {
        initial_mode_idx = 2;
    } else if (s.show_radius_deviation) {
        initial_mode_idx = 3;
    } else if (s.show_output_sensitivity) {
        initial_mode_idx = 4;
    } else if (s.use_material_color) {
        initial_mode_idx = 5;
    } else if (s.channel >= 1 && s.channel <= 6) {
        if (prov_available) {
            initial_mode_idx = 5 + s.channel;
        }
    }
    m_mode_choice->select(initial_mode_idx);
    m_mode_choice->setCallback([this](int idx){
        auto &st = m_plugin->getSettings();
        st.show_normals            = false;
        st.show_accuracy           = false;
        st.show_radius_deviation   = false;
        st.show_output_sensitivity = false;
        st.use_material_color     = false;
        st.channel                 = 0;
        switch (idx) {
        case 0: /* Color */                        break;
        case 1: st.show_normals            = true; break;
        case 2: st.show_accuracy           = true; break;
        case 3: st.show_radius_deviation   = true; break;
        case 4: st.show_output_sensitivity = true; break;
        case 5: st.use_material_color     = true; break;
        default:
            st.channel = idx - 5;
            break;
        }
        });

}


