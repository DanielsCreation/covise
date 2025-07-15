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

    m_selection_group = new opencover::ui::Group(m_lamure_menu, "Selection");

    m_pointcloud_button = new opencover::ui::Button(m_selection_group, "pointcloud");
    m_boundingbox_button = new opencover::ui::Button(m_selection_group, "boundingboxes");
    m_frustum_button = new opencover::ui::Button(m_selection_group, "frustum");
    m_coord_button = new opencover::ui::Button(m_selection_group, "coordinates");
    m_sync_button = new opencover::ui::Button(m_selection_group, "sync");
    m_notify_button = new opencover::ui::Button(m_selection_group, "notify");
    m_text_button = new opencover::ui::Button(m_selection_group, "text");
    m_dump_button = new opencover::ui::Button(m_selection_group, "dump");

    m_pointcloud_button->setShared(true);
    m_boundingbox_button->setShared(true);
    m_frustum_button->setShared(true);
    m_coord_button->setShared(true);
    m_sync_button->setShared(true);
    m_notify_button->setShared(true);
    m_text_button->setShared(true);
    m_dump_button->setShared(true);

    m_model_group = new opencover::ui::Group(m_lamure_menu, "Modelle");

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
    Lamure::instance();
    m_adaption_group = new opencover::ui::Group(m_lamure_menu, "Adaption");

    m_max_radius_slider = new opencover::ui::Slider(m_adaption_group, "max_radius");
    m_max_radius_slider->setText("max. radius");
    m_max_radius_slider->setBounds(0.0, m_plugin->getSettings().max_radius * 5.0f);
    m_max_radius_slider->setValue(m_plugin->getSettings().max_radius);
    m_max_radius_slider->setShared(true);
    m_max_radius_slider->setCallback([this](double value, bool released)
        { m_plugin->getSettings().max_radius = static_cast<float>(value); });

    m_scale_radius_slider = new opencover::ui::Slider(m_adaption_group, "scale_radius");
    m_scale_radius_slider->setText("scale radius");
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
    m_lod_error_slider->setCallback([this](double value, bool released)
        { m_plugin->getSettings().lod_error = static_cast<float>(value); });

    //m_upload_budget_slider = new opencover::ui::Slider(m_adaption_group, "upload_budget");
    //m_upload_budget_slider->setText("Upload Budget (MB)");
    //m_upload_budget_slider->setBounds(LAMURE_MIN_UPLOAD_BUDGET, 256);
    //m_upload_budget_slider->setValue(lamure::ren::policy::get_instance()->max_upload_budget_in_mb());
    //m_upload_budget_slider->setShared(true);
    //m_upload_budget_slider->setCallback([this](double v, bool)
    //    { lamure::ren::policy::get_instance()->set_max_upload_budget_in_mb(static_cast<size_t>(v)); });

    m_rendering_group = new opencover::ui::Group(m_lamure_menu, "Rendering");

    m_lod_button = new opencover::ui::Button(m_rendering_group, "lod_update");
    m_lod_button->setText("lod update");
    m_lod_button->setShared(true);
    m_lod_button->setState(m_plugin->getSettings().lod_update);
    m_lod_button->setCallback([this](bool state)
        { m_plugin->getSettings().surfel_shader = state; });
    m_rendering_group->add(m_lod_button);

    m_surfel_button = new opencover::ui::Button(m_rendering_group, "surfel_shader");
    m_surfel_button->setText("surfel shader");
    m_surfel_button->setShared(true);
    m_surfel_button->setState(m_plugin->getSettings().surfel_shader);
    m_surfel_button->setCallback([this](bool state)
        { m_plugin->getSettings().surfel_shader = state; });
    m_rendering_group->add(m_surfel_button);

    if (m_plugin->getProvValid()) {
        m_prov_button = new opencover::ui::Button(m_rendering_group, "provenance");
        m_prov_button->setText("provenance");
        m_prov_button->setShared(true);
        m_prov_button->setState(m_plugin->getSettings().provenance);
        m_prov_button->setCallback([this](bool state)
            { m_plugin->getSettings().provenance = (state); });
        m_rendering_group->add(m_prov_button);
    }

    m_measure_button = new opencover::ui::Button(m_rendering_group, "measurement");
    m_measure_button->setShared(true);
    m_measure_button->setState(false);
    m_rendering_group->add(m_measure_button);

    m_measure_button->setCallback([this](bool state) {
        if (state)
            m_plugin->startMeasurement();
        else
            m_plugin->stopMeasurement();
    });

}
