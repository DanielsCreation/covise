#ifndef _LAMURE_UI_H
#define _LAMURE_UI_H

#include <string>
#include <vector>
#include <cover/ui/Owner.h>
#include <cover/ui/Menu.h>
#include <cover/ui/Group.h>
#include <cover/ui/Button.h>
#include <cover/ui/Slider.h>
#include <cover/ui/SelectionList.h>
#include <cover/ui/Label.h>

class Lamure;

class LamureUI : public opencover::ui::Owner {
public:
    LamureUI(Lamure* lamure_plugin, const std::string& name);
    ~LamureUI();

    void setupUi();
    opencover::ui::Button* getPointcloudButton() { return m_pointcloud_button; }
    opencover::ui::Button* getBoundingboxButton() { return m_boundingbox_button; }
    opencover::ui::Button* getFrustumButton() { return m_frustum_button; }
    opencover::ui::Button* getCoordButton() { return m_coord_button; }
    opencover::ui::Button* getSyncButton() { return m_sync_button; }
    opencover::ui::Button* getTextButton() { return m_text_button; }
    opencover::ui::Button* getDumpButton() { return m_dump_button; }
    opencover::ui::Button* getProvButton() { return m_prov_button; }
    opencover::ui::Button* getMeasureButton() { return m_measure_button; }
    opencover::ui::Button* getNotifyButton() { return m_notify_button; }

    std::vector<bool> getModelVisibility() { return m_model_visible; }
private:
    Lamure* m_plugin;

    // Modelle
    std::vector<opencover::ui::Button*> m_model_buttons;
    std::vector<bool>                   m_model_visible;

    // Haupt-Buttons
    opencover::ui::Button* m_pointcloud_button   = nullptr;
    opencover::ui::Button* m_boundingbox_button  = nullptr;
    opencover::ui::Button* m_frustum_button      = nullptr;
    opencover::ui::Button* m_coord_button        = nullptr;
    opencover::ui::Button* m_sync_button         = nullptr;
    opencover::ui::Button* m_notify_button       = nullptr;
    opencover::ui::Button* m_text_button         = nullptr;
    opencover::ui::Button* m_dump_button         = nullptr;
    opencover::ui::Button* m_prov_button         = nullptr;
    opencover::ui::Button* m_measure_button      = nullptr;

    // Gruppen und Men�
    opencover::ui::Menu*   m_lamure_menu        = nullptr;
    opencover::ui::Group*  m_selection_group    = nullptr;
    opencover::ui::Group*  m_model_group        = nullptr;
    opencover::ui::Group*  m_adaption_group     = nullptr;
    opencover::ui::Group*  m_rendering_group    = nullptr;

    // Slider
    opencover::ui::Slider* m_max_radius_slider         = nullptr;
    opencover::ui::Slider* m_scale_radius_slider       = nullptr;
    opencover::ui::Slider* m_lod_error_slider          = nullptr;
    opencover::ui::Slider* m_upload_budget_slider      = nullptr;
    opencover::ui::Slider* m_video_memory_budget_slider= nullptr;

    // Shader Button
    opencover::ui::Button* m_surfel_button = nullptr;
};

#endif // _LAMURE_UI_H
