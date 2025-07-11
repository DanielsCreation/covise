#include "../include/PointCloudUIManager_cli.h"
#include "../../LamurePointCloud_cli.h"

#include <cover/ui/Menu.h>
#include <cover/ui/Group.h>
#include <cover/ui/Button.h>
#include <cover/ui/Slider.h>
#include <cover/ui/SelectionList.h>
#include <filesystem>

#include <cover/coVRPluginSupport.h>

PointCloudUIManager_cli::PointCloudUIManager_cli(PointCloudUIManager_cli* plugin)
    : opencover::ui::Owner("LamurePointCloud", opencover::cover->ui)
    , plugin_(plugin)
{
}

PointCloudUIManager_cli::~PointCloudUIManager_cli()
{
}

void PointCloudUIManager_cli::createUI()
{
    lamure_menu_ = new opencover::ui::Menu("Lamure", this);
    lamure_menu_->setText("Lamure");
    lamure_menu_->allowRelayout(true);

    selection_group_ = new opencover::ui::Group(lamure_menu_, "Selection");
    pointcloud_button_      = new opencover::ui::Button(selection_group_, "pointcloud");
    boundingbox_button_     = new opencover::ui::Button(selection_group_, "boundingboxes");
    frustum_button_         = new opencover::ui::Button(selection_group_, "frustum");
    coord_button_           = new opencover::ui::Button(selection_group_, "coordinates");
    sync_button_            = new opencover::ui::Button(selection_group_, "sync");
    notify_button_          = new opencover::ui::Button(selection_group_, "notify");
    text_button_            = new opencover::ui::Button(selection_group_, "text");
    dump_button_            = new opencover::ui::Button(selection_group_, "dump");

    pointcloud_button_->setShared(true);
    boundingbox_button_->setShared(true);
    frustum_button_->setShared(true);
    coord_button_->setShared(true);
    sync_button_->setShared(true);
    notify_button_->setShared(true);
    text_button_->setShared(true);
    dump_button_->setShared(true);

    pointcloud_button_->setCallback([this](bool state) { plugin_->togglePointCloudRendering(state); });
    boundingbox_button_->setCallback([this](bool state) { plugin_->toggleBoundingBoxRendering(state); });
    frustum_button_->setCallback([this](bool state) { plugin_->toggleFrustumRendering(state); });
    coord_button_->setCallback([this](bool state) { plugin_->toggleCoordsRendering(state); });
    sync_button_->setCallback([this](bool state) { plugin_->setSyncCamera(state); });
    notify_button_->setCallback([this](bool /*state*/) { /* ggf. Notify-Logik */ });
    text_button_->setCallback([this](bool state) { plugin_->toggleTextRendering(state); });
    dump_button_->setCallback([this](bool state) { plugin_->dumpSettings(); });

    pointcloud_button_->setState(true);
    sync_button_->setState(true);
    notify_button_->setState(true);
    text_button_->setState(true);

    model_group_ = new opencover::ui::Group(lamure_menu_, "Modelle");

    adaption_group_ = new opencover::ui::Group(lamure_menu_, "Adaption");
    max_radius_slider_ = new opencover::ui::Slider(adaption_group_, "max_radius");
    max_radius_slider_->setText("max. radius");
    max_radius_slider_->setBounds(0.0, plugin_->getSettings().max_radius * 5.0f);
    max_radius_slider_->setValue(plugin_->getSettings().max_radius);
    max_radius_slider_->setShared(true);
    max_radius_slider_->setCallback([this](double v, bool r) { plugin_->setMaxRadius(static_cast<float>(v)); });

    scale_radius_slider_ = new opencover::ui::Slider(adaption_group_, "scale_radius");
    scale_radius_slider_->setText("scale radius");
    scale_radius_slider_->setBounds(0.0001, plugin_->getSettings().scale_radius * 5.0f);
    scale_radius_slider_->setValue(plugin_->getSettings().scale_radius);
    scale_radius_slider_->setScale(opencover::ui::Slider::Logarithmic);
    scale_radius_slider_->setShared(true);
    scale_radius_slider_->setCallback([this](double v, bool r) { plugin_->setScaleRadius(static_cast<float>(v)); });

    lod_error_slider_ = new opencover::ui::Slider(adaption_group_, "lod_error");
    lod_error_slider_->setText("LOD Error");
    lod_error_slider_->setBounds(1.0, 10.0);
    lod_error_slider_->setValue(plugin_->getSettings().lod_error);
    lod_error_slider_->setShared(true);
    lod_error_slider_->setCallback([this](double v, bool r) { plugin_->setLodError(static_cast<float>(v)); });

    upload_budget_slider_ = new opencover::ui::Slider(adaption_group_, "upload_budget");
    upload_budget_slider_->setText("Upload Budget (MB)");
    upload_budget_slider_->setBounds(8, 256);
    upload_budget_slider_->setValue(plugin_->getSettings().upload);
    upload_budget_slider_->setShared(true);
    upload_budget_slider_->setCallback([this](double v, bool) { plugin_->setUploadBudget(static_cast<size_t>(v)); });

    rendering_group_ = new opencover::ui::Group(lamure_menu_, "Rendering");
    surfel_button_ = new opencover::ui::Button(rendering_group_, "surfel_shader");
    surfel_button_->setText("surfel shader");
    surfel_button_->setShared(true);
    surfel_button_->setState(plugin_->getSettings().surfel_shader);
    surfel_button_->setCallback([this](bool state) { plugin_->setSurfelShader(state); });

    prov_button_ = new opencover::ui::Button(rendering_group_, "provenance");
    prov_button_->setText("provenance");
    prov_button_->setShared(true);
    prov_button_->setState(plugin_->getSettings().provenance);
    prov_button_->setCallback([this](bool state) { plugin_->setProvenance(state); });

    measure_button_ = new opencover::ui::Button(rendering_group_, "measurement");
    measure_button_->setShared(true);
    measure_button_->setState(false);
    measure_button_->setCallback([this](bool state) { this->measureCallback(state); });
}

void PointCloudUIManager_cli::setModelButtons(int num_models, const std::vector<std::string>& model_names, const std::vector<uint32_t>& selection)
{
    for (int i = 0; i < num_models; ++i)
    {
        std::filesystem::path pathObj(model_names[i]);
        std::string filename = pathObj.filename().string();
        std::string filename_strip = pathObj.stem().string();
        opencover::ui::Button* btn = new opencover::ui::Button(model_group_, filename_strip, nullptr, i);
        model_group_->add(btn);
        btn->setShared(true);
        bool checked = selection.empty() || (std::find(selection.begin(), selection.end(), i) != selection.end());
        btn->setState(checked);
        plugin_->model_visible_[i] = checked;
        btn->setCallback([this, i](bool state) { plugin_->setModelVisibility(i, state); });
        model_buttons_.push_back(btn);
    }
}

void PointCloudUIManager_cli::measureCallback(bool state)
{
    if (state)
    {
        plugin_->startMeasurement();
    }
    else
    {
        plugin_->stopMeasurement();
    }
}