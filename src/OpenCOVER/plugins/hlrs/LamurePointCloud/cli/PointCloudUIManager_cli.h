#ifndef POINT_CLOUD_UI_MANAGER_CLI_H
#define POINT_CLOUD_UI_MANAGER_CLI_H

#include <cover/ui/Owner.h>
#include <vector>

namespace opencover
{
namespace ui
{
class Menu;
class Group;
class Button;
class Slider;
class SelectionList;
}
}

class LamurePointCloud_cli;

class PointCloudUIManager_cli : public opencover::ui::Owner
{
public:
    PointCloudUIManager_cli(LamurePointCloud_cli* plugin);
    ~PointCloudUIManager_cli();

    void createUI();
    void setModelButtons(int num_models, const std::vector<std::string>& model_names, const std::vector<uint32_t>& selection);

    opencover::ui::Button* getPointCloudButton() const    { return pointcloud_button_; }
    opencover::ui::Button* getBoundingBoxButton() const  { return boundingbox_button_; }
    opencover::ui::Button* getFrustumButton() const      { return frustum_button_; }
    opencover::ui::Button* getCoordButton() const        { return coord_button_; }
    opencover::ui::Button* getSyncButton() const         { return sync_button_; }
    opencover::ui::Button* getNotifyButton() const       { return notify_button_; }
    opencover::ui::Button* getTextButton() const         { return text_button_; }
    opencover::ui::Button* getDumpButton() const         { return dump_button_; }
    opencover::ui::Button* getSurfelButton() const       { return surfel_button_; }
    opencover::ui::Button* getProvButton() const         { return prov_button_; }
    opencover::ui::Button* getMeasureButton() const      { return measure_button_; }

private:
    
    void measureCallback(bool state);

    PointCloudUIManager_cli* plugin_ = nullptr;

    // UI Elements
    opencover::ui::Menu* lamure_menu_ = nullptr;
    opencover::ui::Group* selection_group_ = nullptr;
    opencover::ui::Group* model_group_ = nullptr;
    opencover::ui::Group* adaption_group_ = nullptr;
    opencover::ui::Group* rendering_group_ = nullptr;

    opencover::ui::Button* pointcloud_button_ = nullptr;
    opencover::ui::Button* boundingbox_button_ = nullptr;
    opencover::ui::Button* frustum_button_ = nullptr;
    opencover::ui::Button* coord_button_ = nullptr;
    opencover::ui::Button* sync_button_ = nullptr;
    opencover::ui::Button* notify_button_ = nullptr;
    opencover::ui::Button* text_button_ = nullptr;
    opencover::ui::Button* dump_button_ = nullptr;

    opencover::ui::Button* surfel_button_ = nullptr;
    opencover::ui::Button* prov_button_ = nullptr;
    opencover::ui::Button* measure_button_ = nullptr;
    opencover::ui::Button* face_eye_button_ = nullptr;

    std::vector<opencover::ui::Button*> model_buttons_;

    opencover::ui::Slider* max_radius_slider_ = nullptr;
    opencover::ui::Slider* scale_radius_slider_ = nullptr;
    opencover::ui::Slider* lod_error_slider_ = nullptr;
    opencover::ui::Slider* upload_budget_slider_ = nullptr;

    
};

#endif // POINT_CLOUD_UI_MANAGER_CLI_H
