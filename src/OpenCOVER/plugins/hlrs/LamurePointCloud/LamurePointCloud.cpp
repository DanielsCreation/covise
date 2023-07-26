//local
#include "LamurePointCloud.h"

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

//boost
#include <boost/assign/list_of.hpp>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>

//schism
#include <scm/time.h>
//#include <scm/core.h>
#include <scm/core/math.h>
#include <scm/core/io/tools.h>
#include <scm/core/pointer_types.h>
//#include <scm/core/platform/platform.h>
//#include <scm/core/utilities/platform_warning_disable.h>

//lamure
#include <lamure/pvs/pvs_database.h>
#include <lamure/prov/prov_aux.h>
#include <lamure/vt/pre/AtlasFile.h>
#include <lamure/prov/octree.h>
#include <lamure/vt/VTConfig.h>
#include <lamure/vt/ren/CutDatabase.h>
#include <lamure/vt/ren/CutUpdate.h>

#include <config/coConfigConstants.h>
#include <config/coConfigLog.h>
#include <config/coConfig.h>
#include <config/coConfigString.h>
#include <config/coConfigEntryString.h>

#include <C:\src\covise\src\3rdparty\deskvox/virvo/virvo/vvtoolshed.h>
#include <cover/ui/SelectionList.h>
#include <cover/coVRStatsDisplay.h>
#include <cover/VRSceneGraph.h>
#include "cover/OpenCOVER.h"
#include <cover/VRWindow.h>
#include <cover/VRViewer.h>

#include "Points.h"
#include "LamureDrawable.h"
//#include "LamureDevice.h"
//#include "LamureContext.h"

#include <util/coExport.h>
#include <PluginUtil/FeedbackManager.h>
#include <PluginUtil/ModuleInteraction.h>
#include <OpenVRUI/coButtonInteraction.h>
#include <config/CoviseConfig.h>

#include <lamure/utils.h>
#include "lamure/ren/data_provenance.h"
#include "lamure/ren/controller.h"
#include <lamure/config.h>
#include <lamure/ren/cut.h>
#include <lamure/ren/camera.h>
#include <scm/gl_util/primitives/primitives_fwd.h>
#include <scm/gl_util/primitives.h>

#include <GLFW/glfw3.h>

#define ASSERT(x) if (!(x)) __debugbreak();
#define GLCall(x) GLClearError();\
    x;\
    ASSERT(GLLogCall(#x,__FILE__, __LINE__))

//#ifdef __cplusplus
//extern "C" {
//#endif
//
//    __declspec(dllexport) DWORD NvOptimusEnablement = 1;
//    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
//
//#ifdef __cplusplus
//}
//#endif



static osg::Vec3f vecConv(scm::math::vec3f& v);
static osg::Vec3d vecConv(scm::math::vec3d& v);
static osg::Matrixf matConv(scm::math::mat4f& m);
static osg::Matrixd matConv(scm::math::mat4d& m);
static scm::math::mat4f matConvF(osg::Matrixd& m);
static scm::math::mat4d matConvD(osg::Matrixd& m);
static void sync_osg_cam(boost::shared_ptr<lamure::ren::camera> cam_, osg::Camera* osg_cam_);

osg::Vec3f vecConv(scm::math::vec3f& v) {
    osg::Vec3f vec_osg = osg::Vec3f(v[0], v[1], v[2]);
    return vec_osg;
}

osg::Vec3d vecConv(scm::math::vec3d& v) {
    osg::Vec3d vec_osg = osg::Vec3d(v[0], v[1], v[2]);
    return vec_osg;
}

osg::Matrixf matConv(scm::math::mat4f& m) {
    osg::Matrix mat_osg = osg::Matrixf(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15]);
    return mat_osg;
}

osg::Matrixd matConv(scm::math::mat4d& m) {
    osg::Matrixd mat_osg = osg::Matrixd(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15]);
    return mat_osg;
}

scm::math::mat4f matConvF(osg::Matrixd& m) {
    scm::math::mat4f mat_scm = scm::math::mat4f(m(0, 0), m(0, 1), m(0, 2), m(0, 3),
        m(1, 0), m(1, 1), m(1, 2), m(1, 3),
        m(2, 0), m(2, 1), m(2, 2), m(2, 3),
        m(3, 0), m(3, 1), m(3, 2), m(3, 3));
    return mat_scm;
}

scm::math::mat4d matConvD(osg::Matrixd& m) {
    scm::math::mat4d mat_scm = scm::math::mat4d(m(0, 0), m(0, 1), m(0, 2), m(0, 3),
        m(1, 0), m(1, 1), m(1, 2), m(1, 3),
        m(2, 0), m(2, 1), m(2, 2), m(2, 3),
        m(3, 0), m(3, 1), m(3, 2), m(3, 3));
    return mat_scm;
}

void sync_osg_cam(boost::shared_ptr<lamure::ren::camera> cam_, osg::Camera* osg_cam_) {
    double fovy, aspectRatio, zNear, zFar;
    bool proj_success = osg_cam_->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
    osg::Matrixd& m = osg_cam_->getViewMatrix();
    scm::math::mat4d view_mat = matConvD(m);

    cam_->set_projection_matrix(fovy, aspectRatio, zNear, zFar);
    cam_->set_view_matrix(view_mat);
}

boost::mutex m;
int32_t render_width_;
int32_t render_height_;

int32_t num_models_ = 0;
std::vector<scm::math::mat4d> model_transformations_;

scm::gl::render_device_ptr     device_;
scm::gl::render_context_ptr    context_;
scm::gl::quad_geometry_ptr screen_quad_;
boost::shared_ptr<lamure::ren::camera> camera_ = NULL;
osg::ref_ptr<osg::Camera> osg_camera_;


struct settings {
    int32_t width_{ 1800 };
    int32_t height_{ 1000 };
    int32_t frame_div_{ 1 };
    int32_t vram_{ 1024 };
    int32_t ram_{ 4096 };
    int32_t upload_{ 32 };
    bool provenance_{ 0 };
    bool create_aux_resources_{ 0 };
    double near_plane_{ 0.001f };
    double far_plane_{ 1000.0f };
    double fov_{ 30.0f };
    bool splatting_{ 0 };
    bool gamma_correction_{ 0 };
    int32_t gui_{ 1 };
    int32_t travel_{ 2 };
    float travel_speed_{ 20.5f };
    int32_t max_brush_size_{ 4096 };
    bool lod_update_{ 1 };
    bool use_pvs_{ 0 };
    bool pvs_culling_{ 0 };
    float lod_point_scale_{ 1.0f };
    float aux_point_size_{ 1.0f };
    float aux_point_distance_{ 0.5f };
    float aux_point_scale_{ 1.0f };
    float aux_focal_length_{ 1.0f };
    int32_t vis_{ 0 };
    int32_t show_normals_{ 0 };
    bool show_accuracy_{ 0 };
    bool show_radius_deviation_{ 0 };
    bool show_output_sensitivity_{ 0 };
    bool show_sparse_{ 1 };
    bool show_views_{ 0 };
    bool show_photos_{ 0 };
    bool show_octrees_{ 0 };
    bool show_bvhs_{ 0 };
    bool show_pvs_{ 0 };
    int32_t channel_{ 0 };
    float lod_error_{ LAMURE_DEFAULT_THRESHOLD };
    bool enable_lighting_{ 1 };
    bool use_material_color_{ 0 };
    scm::math::vec3f material_diffuse_{ 0.6f, 0.6f, 0.6f };
    scm::math::vec4f material_specular_{ 0.4f, 0.4f, 0.4f, 1000.0f };
    scm::math::vec3f ambient_light_color_{ 0.1f, 0.1f, 0.1f };
    scm::math::vec4f point_light_color_{ 1.0f, 1.0f, 1.0f, 1.2f };
    bool heatmap_{ 0 };
    float heatmap_min_{ 0.0f };
    float heatmap_max_{ 0.05f };
    //scm::math::vec3f background_color_{ LAMURE_DEFAULT_COLOR_R, LAMURE_DEFAULT_COLOR_G, LAMURE_DEFAULT_COLOR_B };
    scm::math::vec3f background_color_{ 0.5f, 0.5f, 0.5f };
    scm::math::vec3f heatmap_color_min_{ 68.0f / 255.0f, 0.0f, 84.0f / 255.0f };
    scm::math::vec3f heatmap_color_max_{ 251.f / 255.f, 231.f / 255.f, 35.f / 255.f };
    std::string atlas_file_{ "" };
    std::string json_{ "" };
    std::string pvs_{ "" };
    std::string background_image_{ "" };
    int32_t use_view_tf_{ 0 };
    scm::math::mat4d view_tf_{ scm::math::mat4d::identity() };
    std::vector<std::string> models_;
    std::map<uint32_t, scm::math::mat4d> transforms_;
    std::map<uint32_t, std::shared_ptr<lamure::prov::octree>> octrees_;
    std::map<uint32_t, std::vector<lamure::prov::aux::view>> views_;
    std::map<uint32_t, std::string> aux_;
    std::string selection_{ "" };
    float max_radius_{ std::numeric_limits<float>::max() };
    bool imgui{ 0 };
};
settings settings_;

struct gui {
    bool selection_settings_{ false };
    bool view_settings_{ false };
    bool visual_settings_{ false };
    bool provenance_settings_{ false };
    scm::math::mat4f ortho_matrix_;
};
gui gui_;


struct xyz {
    scm::math::vec3f pos_;
    uint8_t r_;
    uint8_t g_;
    uint8_t b_;
    uint8_t a_;
    float rad_;
    scm::math::vec3f nml_;
};


struct vertex {
    scm::math::vec3f pos_;
    scm::math::vec2f uv_;
};


struct selection {
    int32_t selected_model_ = -1;
    int32_t selected_view_ = -1;
    std::vector<xyz> brush_;
    std::set<uint32_t> selected_views_;
    int64_t brush_end_{ 0 };
};
selection selection_;

struct input {
    float trackball_x_ = 0.f;
    float trackball_y_ = 0.f;
    scm::math::vec2i mouse_;
    scm::math::vec2i prev_mouse_;
    bool brush_mode_ = 0;
    bool brush_clear_ = 0;
    bool gui_lock_ = false;
    lamure::ren::camera::mouse_state mouse_state_;
    bool keys_[3] = { 0, 0, 0 };
};
input input_;


struct provenance {
    uint32_t num_views_{ 0 };
};
std::map<uint32_t, provenance> provenance_;


double fps_ = 0.0;
uint64_t rendered_splats_ = 0;
uint64_t rendered_nodes_ = 0;

struct Window {
    Window() {
        _mouse_button_state = MouseButtonState::IDLE;
    }
    unsigned int _width;
    unsigned int _height;
    GLFWwindow* _glfw_window;
    enum MouseButtonState {
        LEFT = 0,
        WHEEL = 1,
        RIGHT = 2,
        IDLE = 3
    };
    MouseButtonState _mouse_button_state;
};

void glut_resize(int32_t w, int32_t h) {
    settings_.width_ = w;
    settings_.height_ = h;
    
      //create_framebuffers();

      //lamure::ren::policy* policy = lamure::ren::policy::get_instance();
      //policy->set_window_width(render_width_);
      //policy->set_window_height(render_height_);

      //context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(render_width_, render_height_)));
    
    camera_->set_projection_matrix(settings_.fov_, float(settings_.width_) / float(settings_.height_), settings_.near_plane_, settings_.far_plane_);

    gui_.ortho_matrix_ =
        scm::math::make_ortho_matrix(0.0f, static_cast<float>(settings_.width_),
            0.0f, static_cast<float>(settings_.height_), -1.0f, 1.0f);
}

void update_input_keys(input& input_, uint8_t k)
{
    ImGuiIO& io = ImGui::GetIO();
    if (k == 'Q')
        input_.keys_[0] = io.KeysDown[k];
    if (k == 'W')
        input_.keys_[1] = io.KeysDown[k];
    if (k == 'E')
        input_.keys_[2] = io.KeysDown[k];
}

void gui_status_screen() {
    static bool status_screen = false;

    ImGui::SetNextWindowPos(ImVec2(20, 20));
    ImGui::SetNextWindowSize(ImVec2(500.0f, 275.0f));
    ImGui::Begin("lamure_vis GUI", &status_screen, ImGuiWindowFlags_MenuBar);
    ImGui::Text("fps %d", (int32_t)fps_);

    double f = (rendered_splats_ / 1000000.0);

    std::stringstream stream;
    stream << std::setprecision(2) << f;
    std::string s = stream.str();

    ImGui::Text("# points %s mio.", s.c_str());
    ImGui::Text("# nodes %d", (uint64_t)rendered_nodes_);
    ImGui::Text("# models %d", num_models_);

    ImGui::Spacing(); ImGui::Spacing(); ImGui::Spacing();

    ImGui::Checkbox("Selection", &gui_.selection_settings_);
    ImGui::Checkbox("View / LOD Settings", &gui_.view_settings_);
    ImGui::Checkbox("Visual Settings", &gui_.visual_settings_);
    if (settings_.provenance_) {
        ImGui::Checkbox("Provenance Settings", &gui_.provenance_settings_);
    }
    else {
        ImGui::Text("No provenance file");
    }
    //if (gui_.selection_settings_) {
    //    gui_selection_settings(settings_);
    //}
    //if (gui_.view_settings_) {
    //    gui_view_settings();
    //}
    //if (gui_.visual_settings_) {
    //    gui_visual_settings();
    //}
    //if (settings_.provenance_ && gui_.provenance_settings_ && settings_.create_aux_resources_) {
    //    gui_provenance_settings();
    //}
    ImGui::End();
}

void glut_keyboard(unsigned char key, int32_t x, int32_t y) {
    uint8_t k = (uint8_t)key;
    switch (k) {
    case 27:
        exit(0);
        break;
    case 'C':
        std::cout << "cam_pos: " << std::endl;
        //std::cout << camera_->get_cam_pos() << "\n" << std::endl;
        break;
    case 'P':
        std::cout << "projection_matrix: " << std::endl;
        std::cout << camera_->get_projection_matrix() << "\n" << std::endl;
        break;
    case 'M':
        std::cout << "cam_matrix: " << std::endl;
        //std::cout << camera_->get_cam_matrix() << "\n" << std::endl;
        break;
    case 'V':
        std::cout << "view_matrix: " << std::endl;
        std::cout << camera_->get_view_matrix() << "\n" << std::endl;
        break;
    case 'F':
    {
        ++settings_.travel_;
        if (settings_.travel_ > 4) {
            settings_.travel_ = 0;
        }
        settings_.travel_speed_ = (settings_.travel_ == 0 ? 0.5f
            : settings_.travel_ == 1 ? 5.5f
            : settings_.travel_ == 2 ? 20.5f
            : settings_.travel_ == 3 ? 100.5f
            : 300.5f);
        camera_->set_dolly_sens_(settings_.travel_speed_);
    }
    break;
    case '0':
        selection_.selected_model_ = -1;
        break;
    case '-':
        if (--selection_.selected_model_ < 0) selection_.selected_model_ = num_models_ - 1;
        break;
    case '=':
        if (++selection_.selected_model_ >= num_models_) selection_.selected_model_ = 0;
        break;
    case 'Z': //deutsche Tastaturauslegung: 'Y'
        std::cout << "view_tf: " << std::endl;
        std::cout << camera_->get_high_precision_view_matrix() << "\n" << std::endl;
        break;
        //case ' ':
        //    settings_.gui_ = !settings_.gui_;
        //    break;
    case 'B':
        //save_brush();
        break;
    }
}

std::list<Window*> _windows;
Window* _current_context = nullptr;


class EventHandler {
public:
    static void on_error(int _err_code, const char* err_msg) { throw std::runtime_error(err_msg); }

    static void on_window_resize(GLFWwindow* glfw_window, int width, int height) {
        Window* window = (Window*)glfwGetWindowUserPointer(glfw_window);
        window->_height = (uint32_t)height;
        window->_width = (uint32_t)width;
        glut_resize(width, height);
    }

    static void on_window_key_press(GLFWwindow* glfw_window, int key, int scancode, int action, int mods) {
        uint8_t k = (uint8_t)key;
        if (action == GLFW_PRESS) {
            ImGui_ImplGlfwGL3_KeyCallback(glfw_window, key, scancode, action, mods);
            update_input_keys(input_, k);
            switch (k) {
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(glfw_window, GL_TRUE);
                break;
            default:
                glut_keyboard(k, 0, 0);
                break;
            }
        }
        if (action == GLFW_RELEASE)
        {
            ImGui_ImplGlfwGL3_KeyCallback(glfw_window, key, scancode, action, mods);
            update_input_keys(input_, k);
        }
    }

    static void on_window_char(GLFWwindow* glfw_window, unsigned int codepoint) {
        Window* window = (Window*)glfwGetWindowUserPointer(glfw_window);
        ImGui_ImplGlfwGL3_CharCallback(glfw_window, codepoint);
    }

    static void on_window_button_press(GLFWwindow* glfw_window, int button, int action, int mods) {
        Window* window = (Window*)glfwGetWindowUserPointer(glfw_window);
        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
            window->_mouse_button_state = Window::MouseButtonState::LEFT;
        }
        else if (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS) {
            window->_mouse_button_state = Window::MouseButtonState::WHEEL;
        }
        else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
            window->_mouse_button_state = Window::MouseButtonState::RIGHT;
        }
        else {
            window->_mouse_button_state = Window::MouseButtonState::IDLE;
        }

        if (action == GLFW_RELEASE) {
            input_.gui_lock_ = false;
        }

        ImGui_ImplGlfwGL3_MouseButtonCallback(glfw_window, button, action, mods);
    }

    static void on_window_move_cursor(GLFWwindow* glfw_window, double x, double y) {
        Window* window = (Window*)glfwGetWindowUserPointer(glfw_window);

        if (input_.gui_lock_) {
            input_.prev_mouse_ = scm::math::vec2i(x, y);
            input_.mouse_ = scm::math::vec2i(x, y);
            return;
        }

        input_.prev_mouse_ = input_.mouse_;
        input_.mouse_ = scm::math::vec2i(x, y);

        if (!input_.brush_mode_ && !input_.keys_[0] && !input_.keys_[1] && !input_.keys_[2])
        {
            camera_->update_trackball(x, y, settings_.width_, settings_.height_, input_.mouse_state_);
        }
        else if (!input_.brush_mode_ && (input_.keys_[0] || input_.keys_[1] || input_.keys_[2]))
        {
            //camera_->update_camera(x, y, settings_.width_, settings_.height_, input_.mouse_state_, input_.keys_);
        }
        else {
            //brush();
        }

        input_.mouse_state_.lb_down_ = (window->_mouse_button_state == Window::MouseButtonState::LEFT) ? true : false;
        input_.mouse_state_.mb_down_ = (window->_mouse_button_state == Window::MouseButtonState::WHEEL) ? true : false;
        input_.mouse_state_.rb_down_ = (window->_mouse_button_state == Window::MouseButtonState::RIGHT) ? true : false;

        input_.prev_mouse_ = input_.mouse_;
        input_.mouse_ = scm::math::vec2i(x, y);

        if (!input_.brush_mode_)
        {
            input_.trackball_x_ = 2.f * float(x - (settings_.width_ / 2)) / float(settings_.width_);
            input_.trackball_y_ = 2.f * float(settings_.height_ - y - (settings_.height_ / 2)) / float(settings_.height_);

            camera_->update_trackball_mouse_pos(input_.trackball_x_, input_.trackball_y_);
        }
        else
        {
            //brush();
        }
    }

    static void on_window_scroll(GLFWwindow* glfw_window, double xoffset, double yoffset) {
        Window* window = (Window*)glfwGetWindowUserPointer(glfw_window);
        ImGui_ImplGlfwGL3_ScrollCallback(glfw_window, xoffset, yoffset);
    }

    static void on_window_enter(GLFWwindow* glfw_window, int entered) {
        Window* window = (Window*)glfwGetWindowUserPointer(glfw_window);
    }
};


void make_context_current(Window* _window) {
    if (_window != nullptr) {
        glfwMakeContextCurrent(_window->_glfw_window);
        _current_context = _window;
    }
}

Window* create_window(unsigned int width, unsigned int height, const std::string& title, GLFWmonitor* monitor, Window* share) {
    Window* previous_context = _current_context;

    Window* new_window = new Window();

    new_window->_glfw_window = nullptr;
    new_window->_width = width;
    new_window->_height = height;

    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_ANY_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, 1);
    glfwWindowHint(GLFW_FOCUSED, 1);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, true);
    //glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, false);


    if (share != nullptr) {
        new_window->_glfw_window = glfwCreateWindow(width, height, title.c_str(), monitor, share->_glfw_window);
    }
    else {
        new_window->_glfw_window = glfwCreateWindow(width, height, title.c_str(), monitor, nullptr);
    }

    if (new_window->_glfw_window == nullptr) {
        std::runtime_error("GLFW window creation failed");
    }

    make_context_current(new_window);
    glfwSetKeyCallback(new_window->_glfw_window, &EventHandler::on_window_key_press);
    glfwSetCharCallback(new_window->_glfw_window, &EventHandler::on_window_char);
    glfwSetMouseButtonCallback(new_window->_glfw_window, &EventHandler::on_window_button_press);
    glfwSetCursorPosCallback(new_window->_glfw_window, &EventHandler::on_window_move_cursor);
    glfwSetScrollCallback(new_window->_glfw_window, &EventHandler::on_window_scroll);
    glfwSetCursorEnterCallback(new_window->_glfw_window, &EventHandler::on_window_enter);
    glfwSetWindowSizeCallback(new_window->_glfw_window, &EventHandler::on_window_resize);
    glfwSetWindowUserPointer(new_window->_glfw_window, new_window);
    _windows.push_back(new_window);
    make_context_current(previous_context);
    return new_window;
}



bool should_close() {
    if (_windows.empty())
        return true;

    std::list<Window*> to_delete;
    for (const auto& window : _windows) {
        if (glfwWindowShouldClose(window->_glfw_window)) {
            to_delete.push_back(window);
        }
    }

    if (!to_delete.empty()) {
        for (auto& window : to_delete) {
            ImGui_ImplGlfwGL3_Shutdown();

            glfwDestroyWindow(window->_glfw_window);

            delete window;

            _windows.remove(window);
        }
    }
    return _windows.empty();
}
//*/


lamure::ren::Data_Provenance data_provenance_;
float height_divided_by_top_minus_bottom_ = 0.f;
coVRShader* pointShader = coVRShaderList::instance()->get("Points");

scm::gl::program_ptr vis_xyz_shader_;
scm::gl::program_ptr vis_xyz_pass1_shader_;
scm::gl::program_ptr vis_xyz_pass2_shader_;
scm::gl::program_ptr vis_xyz_pass3_shader_;
scm::gl::program_ptr vis_xyz_lighting_shader_;
scm::gl::program_ptr vis_xyz_pass2_lighting_shader_;
scm::gl::program_ptr vis_xyz_pass3_lighting_shader_;
scm::gl::program_ptr vis_xyz_qz_shader_;
scm::gl::program_ptr vis_xyz_qz_pass1_shader_;
scm::gl::program_ptr vis_xyz_qz_pass2_shader_;
scm::gl::program_ptr vis_quad_shader_;
scm::gl::program_ptr vis_line_shader_;
scm::gl::program_ptr vis_triangle_shader_;
scm::gl::program_ptr vis_vt_shader_;
scm::gl::frame_buffer_ptr fbo_;
scm::gl::texture_2d_ptr fbo_color_buffer_;
scm::gl::texture_2d_ptr fbo_depth_buffer_;
scm::gl::frame_buffer_ptr pass1_fbo_;
scm::gl::frame_buffer_ptr pass2_fbo_;
scm::gl::texture_2d_ptr pass1_depth_buffer_;
scm::gl::texture_2d_ptr pass2_color_buffer_;
scm::gl::texture_2d_ptr pass2_normal_buffer_;
scm::gl::texture_2d_ptr pass2_view_space_pos_buffer_;
scm::gl::texture_2d_ptr pass2_depth_buffer_;
scm::gl::depth_stencil_state_ptr depth_state_disable_;
scm::gl::depth_stencil_state_ptr depth_state_less_;
scm::gl::depth_stencil_state_ptr depth_state_without_writing_;
scm::gl::rasterizer_state_ptr no_backface_culling_rasterizer_state_;
scm::gl::blend_state_ptr color_blending_state_;
scm::gl::blend_state_ptr color_no_blending_state_;
scm::gl::sampler_state_ptr filter_linear_;
scm::gl::sampler_state_ptr filter_nearest_;
scm::gl::sampler_state_ptr vt_filter_linear_;
scm::gl::sampler_state_ptr vt_filter_nearest_;
scm::gl::texture_2d_ptr bg_texture_;

scm::time::accum_timer<scm::time::high_res_timer> frame_time_;


struct GeoGl : public osg::Group
{
    GeoGl()
        : _triangle_geode(new osg::Geode),
        _line_geode(new osg::Geode),
        _point_geode(new osg::Geode)
    {
        addChild(_triangle_geode.get());
        addChild(_line_geode.get());
        addChild(_point_geode.get());
    }

    void addTriangleGeo(osg::Stats* viewerStats)
    {
        _triangle_geode->addDrawable(new TriangleGeometry(this, viewerStats));
    }

    void removeTriangleGeo(osg::Stats* viewerStats)
    {
        uint8_t num_drawables = _triangle_geode->getNumDrawables();
        _triangle_geode->removeDrawables(0, _triangle_geode->getNumDrawables());
    }

    void addLineGeo(osg::Stats* viewerStats)
    {
        _line_geode->addDrawable(new LinesGeometry(this, viewerStats));
    }

    void addPointGeo(osg::Stats* viewerStats)
    {
        _point_geode->addDrawable(new PointsGeometry(this, viewerStats));
    }
    osg::ref_ptr<osg::Geode> _triangle_geode;
    osg::ref_ptr<osg::Geode> _line_geode;
    osg::ref_ptr<osg::Geode> _point_geode;

protected:
    struct TriangleGeometry : public osg::Geometry
    {
        TriangleGeometry(struct GeoGl* geo_gl, osg::Stats* viewerStats)
        {
            std::cout << "protected struct TriangleGeometry()" << std::endl;
            setUseDisplayList(true);
            setUseVertexBufferObjects(false);
            setDrawCallback(new TriangleUpdateCallback(geo_gl, viewerStats));
        }
    };
    struct TriangleUpdateCallback : public virtual osg::Drawable::DrawCallback
    {
        TriangleUpdateCallback(GeoGl* geo_gl, osg::Stats* viewerStats)
            : _geo_gl(geo_gl),
            _viewerStats(viewerStats)
        {
            std::cout << "protected struct TriangleUpdateCallback()" << std::endl;
        }
        /** do customized draw code.*/
        virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const
        {
            std::cout << "protected struct TriangleUpdateCallback(), virtual void drawImplementation()" << std::endl;

            glBegin(GL_TRIANGLES);
            {
                glVertex3f(-500.0f, 0.0f, -500.0f);
                glVertex3f(500.0f, 0.0f, 500.0f);
                glVertex3f(500.0f, 0.0f, -500);
            }
            glEnd();
            drawable->drawImplementation(renderInfo);
        }
        GeoGl* _geo_gl = nullptr;
        osg::Stats* _viewerStats;
    };

    struct LinesGeometry : public osg::Geometry
    {
        LinesGeometry(struct GeoGl* geo_gl, osg::Stats* viewerStats)
        {
            std::cout << "LinesGeometry()" << std::endl;
            setDrawCallback(new LinesUpdateCallback(geo_gl, viewerStats));
        }
        virtual void drawImplementation(osg::RenderInfo& renderInfo) const
        {

        }

    };
    struct LinesUpdateCallback : public virtual osg::Drawable::DrawCallback
    {
        LinesUpdateCallback(GeoGl* geo_gl, osg::Stats* viewerStats)
            : _geo_gl(geo_gl),
            _viewerStats(viewerStats)
        {
            std::cout << "LinesUpdateCallback()" << std::endl;
        }

        /** do customized draw code.*/
        virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const
        {
            std::cout << "LinesUpdateCallback, drawImplementation()" << std::endl;
        }
        GeoGl* _geo_gl = nullptr;
        osg::Stats* _viewerStats;
    };

    struct PointsGeometry : public osg::Geometry
    {
        PointsGeometry(struct GeoGl* geo_gl, osg::Stats* viewerStats)
        {
            std::cout << "PointsGeometry()" << std::endl;
            setDrawCallback(new PointsUpdateCallback(geo_gl, viewerStats));
        }
    };
    struct PointsUpdateCallback : public virtual osg::Drawable::DrawCallback
    {
        PointsUpdateCallback(GeoGl* geo_gl, osg::Stats* viewerStats)
            : _geo_gl(geo_gl),
            _viewerStats(viewerStats)
        {
            std::cout << "protected struct PointsUpdateCallback()" << std::endl;
        }
        /** do customized draw code.*/
        virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const
        {
            std::cout << "PointsUpdateCallback(), drawImplementation()" << std::endl;
            //bLamurePointCloudPlugin::instance()->lamure_display();
        }
        GeoGl* _geo_gl = nullptr;
        osg::Stats* _viewerStats;
    };
};


struct resource {
    uint64_t num_primitives_{ 0 };
    scm::gl::buffer_ptr buffer_;
    scm::gl::vertex_array_ptr array_;
};

resource brush_resource_;
resource pvs_resource_;
std::map<uint32_t, resource> bvh_resources_;
std::map<uint32_t, resource> sparse_resources_;
std::map<uint32_t, resource> frusta_resources_;
std::map<uint32_t, resource> octree_resources_;
std::map<uint32_t, resource> image_plane_resources_;


struct vt_info {
    uint32_t texture_id_;
    uint16_t view_id_;
    uint16_t context_id_;
    uint64_t cut_id_;
    vt::CutUpdate* cut_update_;
    std::vector<scm::gl::texture_2d_ptr> index_texture_hierarchy_;
    scm::gl::texture_2d_ptr physical_texture_;
    scm::math::vec2ui physical_texture_size_;
    scm::math::vec2ui physical_texture_tile_size_;
    size_t size_feedback_;
    int32_t* feedback_lod_cpu_buffer_;
    uint32_t* feedback_count_cpu_buffer_;
    scm::gl::buffer_ptr feedback_lod_storage_;
    scm::gl::buffer_ptr feedback_count_storage_;
    int toggle_visualization_;
    bool enable_hierarchy_;
};
vt_info vt_;



using namespace osg;
using namespace std;
using covise::coCoviseConfig;
using vrui::coInteraction;

COVERPLUGIN(LamurePointCloudPlugin)
LamurePointCloudPlugin* LamurePointCloudPlugin::plugin = nullptr;

static FileHandler handler = 
    { NULL, 
      LamurePointCloudPlugin::loadLMR, 
      LamurePointCloudPlugin::unloadLMR, 
      "lmr"
};

// Constructor
LamurePointCloudPlugin::LamurePointCloudPlugin(): ui::Owner("LamurePointCloud", cover->ui)
{
    coVRFileManager::instance()->registerFileHandler(&handler);
    plugin = this;
}



int LamurePointCloudPlugin::loadLMR(const char* filename, osg::Group* parent, const char* covise_key)
{
    std::printf("loadLMR()\n");

    const osg::GraphicsContext::Traits* traits = coVRConfig::instance()->windows[0].context->getTraits();

    putenv((char*)"__GL_SYNC_TO_VBLANK=0");

    assert(plugin);
    std::string lmr_file = std::string(filename);
    plugin->load_settings(lmr_file);
    settings_.vis_ = settings_.show_normals_ ? 1
        : settings_.show_accuracy_ ? 2
        : settings_.show_output_sensitivity_ ? 3
        : settings_.channel_ > 0 ? 3 + settings_.channel_
        : 0;

    if (settings_.provenance_ && settings_.json_ != "") {
        std::cout << "json: " << settings_.json_ << std::endl;
        data_provenance_ = lamure::ren::Data_Provenance::parse_json(settings_.json_);
        std::cout << "size of provenance: " << data_provenance_.get_size_in_bytes() << std::endl;
    }

    render_width_ = settings_.width_ / settings_.frame_div_;
    render_height_ = settings_.height_ / settings_.frame_div_;

    char str[200];
    sprintf(str, "COVER.WindowConfig.Window:%d", 0);

    //std::printf("render_width_: %03" PRId32 "\n", traits->width);
    //std::printf("render_height_: %03" PRId32 "\n", traits->height);

    std::printf("render_width_: %03" PRId32 "\n", render_width_);
    std::printf("render_height_: %03" PRId32 "\n", render_height_);

    lamure::ren::policy* policy = lamure::ren::policy::get_instance();
    policy->set_max_upload_budget_in_mb(settings_.upload_);
    policy->set_render_budget_in_mb(settings_.vram_);
    policy->set_out_of_core_budget_in_mb(settings_.ram_);
    policy->set_window_width(traits->width);
    policy->set_window_height(traits->height);

    lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
    for (const auto& input_file : settings_.models_) {
        lamure::model_t model_id = database->add_model(input_file, std::to_string(num_models_));
        model_transformations_.push_back(settings_.transforms_[num_models_] * scm::math::mat4d(scm::math::make_translation(database->get_model(num_models_)->get_bvh()->get_translation())));
        ++num_models_;
    }
    std::cout << osg::DisplaySettings::instance()->getDisplayType() << std::endl;

    //std::cout << "(const char*)glGetString(GL_VERSION):" << std::endl;
    //std::cout << (GLubyte*)glGetString(GL_VERSION) << std::endl;

    //HWND hwnd_ = FindWindow(NULL, "COVER");
    //std::cout << "FindWindow(NULL, 'COVER'): " << FindWindow(NULL, "COVER") << std::endl;
    //HWND hwnd___ = FindWindow(NULL, "OpenCOVER");
    //std::cout << "FindWindow(NULL, 'OpenCOVER'): " << FindWindow(NULL, "OpenCOVER") << std::endl;
    //HDC hdc_covise = GetDC(FindWindow(NULL, "COVER"));
    //HDC hdc_opencover = GetDC(FindWindow(NULL, "OpenCOVER"));
    //std::cout << "Vergleich der Windows: " << std::endl;
    //std::cout << "WindowFromDC(GetDC(FindWindow(NULL, COVER)): " << WindowFromDC(GetDC(FindWindow(NULL, "COVER"))) << std::endl;
    //std::cout << "WindowFromDC(GetDC(FindWindow(NULL, OpenCOVER))): " << WindowFromDC(GetDC(FindWindow(NULL, "OpenCOVER"))) << std::endl;
    //std::cout << "WindowFromDC(wglGetCurrentDC()): " << WindowFromDC(wglGetCurrentDC()) << std::endl;
    //std::cout << "Vergleich der DCs: " << std::endl;
    //std::cout << "GetDC(FindWindow(NULL, COVER)): " << GetDC(FindWindow(NULL, "COVER")) << std::endl;
    //std::cout << "GetDC(FindWindow(NULL, OpenCOVER)): " << GetDC(FindWindow(NULL, "OpenCOVER")) << std::endl;
    //std::cout << "wglGetCurrentDC(): " << wglGetCurrentDC() << std::endl;
    //SwapBuffers(hdc);
    //HWND hwnd = FindWindow(NULL, "COVER");
    //HDC hdc = GetDC(hwnd);
    //HGLRC hglrc = wglCreateContext(hdc);
    //wglMakeCurrent(hdc, hglrc);

    return 1;
}


bool LamurePointCloudPlugin::init()
{
    std::cout << "init()" << std::endl;

    std::cout << "getConfigEntry(COVER.Plugin.LamurePointCloud).c_str(): " << getConfigEntry("COVER.Plugin.LamurePointCloud").c_str() << std::endl;
    plugin->file = coVRFileManager::instance()->loadFile(getConfigEntry("COVER.Plugin.LamurePointCloud").c_str());

    //cover->addPlugin("Move");
    //cover->addPlugin("Annotation");

    plugin->current_context = wglGetCurrentContext();
    plugin->hwnd = FindWindow(NULL, "OpenCOVER");
    plugin->hdc = GetDC(plugin->hwnd);

    std::cerr << "hostname: " << covise::coConfigConstants::getHostname() << std::endl;

    //Create main menu button
    //lamureMenu = new ui::Menu("LamureMenu", this);
    //lamureMenu->setText("LamurePlugin");
    //selectionGroup = new ui::Group(lamureMenu, "Selection");
    //selectionButtonGroup = new ui::ButtonGroup(selectionGroup, "SelectionGroup");
    //selectionButtonGroup->enableDeselect(true);
    //fileButtonGroup = new ui::ButtonGroup(selectionGroup, "FileButtonGroup");
    //fileButtonGroup->enableDeselect(true);

    plugin->LamureGroup = new osg::Group();
    plugin->LamureGroup->setName("LamureGroup");
    cover->getObjectsRoot()->addChild(plugin->LamureGroup);

    plugin->transform = new osg::MatrixTransform();
    plugin->LamureGroup->addChild(plugin->file);
    plugin->LamureGroup->addChild(plugin->transform);

    plugin->geode = new osg::Geode();
    plugin->geode->setName("LamureGeode");
    plugin->transform->addChild(plugin->geode);

    plugin->geo_gl = new GeoGl();
    plugin->LamureGroup->addChild(plugin->geo_gl);

    
    /*plugin->pointSet = new PointSet[1];
    plugin->pointSet[0].colors = new Color[1024];
    plugin->pointSet[0].points = new ::Point[1024];
    plugin->pointSet[0].size = 1024;

    for (int n = 0; n < 1024; ++n)
    {
        plugin->pointSet[0].points[n].coordinates.x() = -500 + n;
        plugin->pointSet[0].points[n].coordinates.y() = -500 + n;
        plugin->pointSet[0].points[n].coordinates.z() = -500 + n;

        plugin->pointSet[0].colors[n].r = n / 1024.0;
        plugin->pointSet[0].colors[n].g = (n + 500) / 2048.0;
        plugin->pointSet[0].colors[n].b = (n + 500) / 4096.0;
    }

    plugin->geometry = new LamureGeometry(&plugin->pointSet[0]);
    plugin->geode->addDrawable(plugin->geometry.get());*/

    //std::cout << covise::coConfigDefaultPaths::getDefaultTransformFileName() << std::endl;
    //std::cout << covise::coConfigDefaultPaths::getDefaultLocalConfigFileName() << std::endl;
    //std::cout << covise::coConfigDefaultPaths::getDefaultGlobalConfigFileName() << std::endl;

    //VRViewer::instance()->statsDisplay->showStats(coVRStatsDisplay::VIEWER_SCENE_STATS, VRViewer::instance());

    //VRSceneGraph::instance()->viewAll();

    GLenum err = glfwInit();

    device_.reset(new scm::gl::render_device());
    if (!device_) {
        std::cout << "error creating device" << std::endl;
    }
    context_ = device_->main_context();
    if (!context_) {
        std::cout << "error creating context" << std::endl;
    }
    glfwSetErrorCallback(EventHandler::on_error);
    std::cout << err << std::endl;
    Window* primary_window = create_window(render_width_, render_height_, "lamure_vis", nullptr, nullptr);
    make_context_current(primary_window);

    std::cout << (*device_);
    plugin->init_lamure_shader();
    plugin->create_framebuffers();
    //plugin->create_aux_resources_buffered();
    plugin->init_render_states();
    plugin->init_camera();
  

    /*GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);
    std::cout << mode << std::endl;
    int* x_pos;
    int* y_pos;
    int* width;
    int* height;
    glfwGetMonitorPos(monitor, x_pos, y_pos);
    glfwGetMonitorPhysicalSize(monitor, width, height);
    std::cout << *x_pos << " " << *y_pos << " " << *width << " " << *height << " " << std::endl;
    std::cout << " " << std::endl;
    glfwGetWindowPos(primary_window->_glfw_window, x_pos, y_pos);
    glfwGetWindowSize(primary_window->_glfw_window, width, height);
    std::cout << x_pos << " " << y_pos << " " << width << " " << height << " " << std::endl;
    std::cout << *x_pos << " " << *y_pos << " " << *width << " " << *height << " " << std::endl;
    std::cout << " " << std::endl;*/
    
    glfwSwapInterval(1);
    make_context_current(primary_window);
    plugin->lamure_display();
    glewExperimental = GL_TRUE;
    if (GLEW_OK != err) {
        std::cout << "GLEW error: " << glewGetErrorString(err) << std::endl;
    }
    std::cout << "using GLEW " << glewGetString(GLEW_VERSION) << std::endl;
    ImGui_ImplGlfwGL3_Init(primary_window->_glfw_window, false);
    ImGui_ImplGlfwGL3_CreateDeviceObjects();
    while (!should_close()) {
        glfwPollEvents();
        for (const auto& window : _windows) {
            make_context_current(window);
            if (window == primary_window) {
                plugin->lamure_display();
                if (settings_.gui_) {
                    ImGui_ImplGlfwGL3_NewFrame();
                    gui_status_screen();
                    ImGui::Render();
                }
            }
            glfwSwapBuffers(window->_glfw_window);
        }
    }
    
    return 1;
}



size_t LamurePointCloudPlugin::query_video_memory_in_mb() {
    int size_in_kb;
    glGetIntegerv(GL_GPU_MEMORY_INFO_CURRENT_AVAILABLE_VIDMEM_NVX, &size_in_kb);
    return size_t(size_in_kb) / 1024;
}



struct DrawableNode : public osg::Drawable {
    DrawableNode()
    {
    }
    virtual void drawImplementation(osg::RenderInfo& renderInfo) {
        std::cout << "Drawables drawImplementation, every loop?" << std::endl;
    }
};

struct Event : public virtual osg::Drawable::EventCallback
{
    Event() {
        std::cout << "Event worked" << std::endl;
    }

    virtual void drawImplementation(osg::RenderInfo& renderInfo) const
    {
        std::cout << "drawImplementation Event worked" << std::endl;
        drawImplementation(renderInfo);
    }
};

struct Draw : public virtual osg::Drawable::DrawCallback
{
    Draw() {
        std::cout << "draw worked" << std::endl;
    }

    virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const
    {
        drawable->drawImplementation(renderInfo);
        std::cout << "Drawcallbacks drawImplementation" << std::endl;

        glBegin(GL_TRIANGLES);
        {
            glVertex3f(-500.0f, 0.0f, -500.0f);
            glVertex3f(500.0f, 0.0f, 500.0f);
            glVertex3f(500.0f, 0.0f, -500);
        }
        glEnd();
    }
};


unsigned int counter = 0;
void LamurePointCloudPlugin::preFrame() {
    if (cover->getPointerButton()->getState() == 1 && counter == 0) {
        //std::cout << "preFrame(), " << "counter == 0" << std::endl;
        //device_.reset(new scm::gl::render_device());
        //if (!device_) {
        //    std::cout << "error creating device" << std::endl;
        //}
        //context_ = device_->main_context();
        //if (!context_) {
        //    std::cout << "error creating context" << std::endl;
        //}
        //std::cout << (*device_);

        //plugin->init_lamure_shader();
        //plugin->create_framebuffers();
        //plugin->create_aux_resources_buffered();
        //plugin->init_render_states();
        //plugin->init_camera();

        //osg::ref_ptr<struct DrawableNode> maindrawable(new DrawableNode());
        //plugin->fix_geode->addDrawable(maindrawable);

        //GeoGl* geo_gl = new GeoGl();
        //LamureGroup->addChild(geo_gl);
        //geo_gl->addTriangleGeo(VRViewer::instance()->getViewerStats());
        //geo_gl->addPointGeo(VRViewer::instance()->getViewerStats());
        //geo_gl->addLineGeo(VRViewer::instance()->getViewerStats());

        //plugin->lamure_display();
        //counter = counter + 1;
    }

    
}

bool LamurePointCloudPlugin::update()
{
    if (cover->getPointerButton()->getState() == 1 /*|| cover->getPointerButton()->getState() == 0*/) {
        if (counter == 0) {
            /*std::cout << "preFrame(), " << "counter == 0" << std::endl;
            device_.reset(new scm::gl::render_device());
            if (!device_) {
                std::cout << "error creating device" << std::endl;
            }
            context_ = device_->main_context();
            if (!context_) {
                std::cout << "error creating context" << std::endl;
            }
            //std::cout << (*device_);

            plugin->init_lamure_shader();
            plugin->create_framebuffers();
            plugin->create_aux_resources_buffered();
            plugin->init_render_states();
            plugin->init_camera();
            plugin->lamure_display();
            //osg::ref_ptr<struct DrawableNode> maindrawable(new DrawableNode());
            //plugin->fix_geode->addDrawable(maindrawable);

            plugin->geo_gl->addPointGeo(VRViewer::instance()->getViewerStats());
            plugin->geo_gl->addTriangleGeo(VRViewer::instance()->getViewerStats());
            plugin->geo_gl->addLineGeo(VRViewer::instance()->getViewerStats());

            glfwSetErrorCallback(EventHandler::on_error);

            if (!glfwInit()) {
                std::runtime_error("GLFW initialisation failed");
            }
            make_context_current(primary_window);
            glfwSwapInterval(1);
            make_context_current(primary_window);
            lamure_display();
            glewExperimental = GL_TRUE;
            GLenum err = glewInit();
            if (GLEW_OK != err) {
                std::cout << "GLEW error: " << glewGetErrorString(err) << std::endl;
            }
            std::cout << "using GLEW " << glewGetString(GLEW_VERSION) << std::endl;
            ImGui_ImplGlfwGL3_Init(primary_window->_glfw_window, false);
            ImGui_ImplGlfwGL3_CreateDeviceObjects();*/
            

            //lamure_display();
            //counter = counter + 1;
        }
        
        /*
        glfwPollEvents();

        for (const auto& window : _windows) {
            make_context_current(window);

            if (window == primary_window) {
                plugin->lamure_display();

                if (settings_.gui_) {
                    ImGui_ImplGlfwGL3_NewFrame();
                    gui_status_screen();
                    ImGui::Render();
                }

            }
            glfwSwapBuffers(window->_glfw_window);
        }
        GeoGl* geo_gl = new GeoGl();
        LamureGroup->addChild(geo_gl);
        geo_gl->addPointGeo(VRViewer::instance()->getViewerStats());
        geo_gl->addTriangleGeo(VRViewer::instance()->getViewerStats());
        geo_gl->addLineGeo(VRViewer::instance()->getViewerStats());
        */

    };

    return 1;
}

void LamurePointCloudPlugin::lamure_display() {
    const osg::GraphicsContext::Traits* traits = coVRConfig::instance()->windows[0].context->getTraits();
    osg::Camera* osg_cam_ = VRViewer::instance()->getCamera();
    sync_osg_cam(camera_, osg_cam_);
    if (rendering_) { return; }
    rendering_ = true;
    lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
    lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
    lamure::ren::controller* controller = lamure::ren::controller::get_instance();
    lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();
    if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
        controller->reset_system(data_provenance_);
    }
    else {
        controller->reset_system();
    }
    lamure::context_t context_id = controller->deduce_context_id(0);
    for (lamure::model_t model_id = 0; model_id < num_models_; ++model_id) {
        lamure::model_t m_id = controller->deduce_model_id(std::to_string(model_id));
        cuts->send_transform(context_id, m_id, scm::math::mat4f(model_transformations_[m_id]));
        cuts->send_threshold(context_id, m_id, settings_.lod_error_);
        cuts->send_rendered(context_id, m_id);
        database->get_model(m_id)->set_transform(scm::math::mat4f(model_transformations_[m_id]));
    }
    lamure::view_t cam_id = controller->deduce_view_id(context_id, camera_->view_id());
    cuts->send_camera(context_id, cam_id, *camera_);
    std::vector<scm::math::vec3d> corner_values = camera_->get_frustum_corners();
    double top_minus_bottom = scm::math::length((corner_values[2]) - (corner_values[0]));
    height_divided_by_top_minus_bottom_ = lamure::ren::policy::get_instance()->window_height() / top_minus_bottom;
    cuts->send_height_divided_by_top_minus_bottom(context_id, cam_id, height_divided_by_top_minus_bottom_);

    if (settings_.use_pvs_) {
        scm::math::mat4f cm = scm::math::inverse(scm::math::mat4f(camera_->trackball_matrix()));
        scm::math::vec3d cam_pos = scm::math::vec3d(cm[12], cm[13], cm[14]);
        pvs->set_viewer_position(cam_pos);
    }

    if (settings_.lod_update_) {
        if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
            controller->dispatch(context_id, device_, data_provenance_);
        }
        else {
            controller->dispatch(context_id, device_);
        }
    }
    lamure::view_t view_id = controller->deduce_view_id(context_id, camera_->view_id());
    context_->set_rasterizer_state(no_backface_culling_rasterizer_state_);

    if (settings_.splatting_) {
        //2 pass splatting
        //PASS 1
        context_->clear_color_buffer(pass1_fbo_, 0, scm::math::vec4f(.0f, .0f, .0f, 0.0f));
        context_->clear_depth_stencil_buffer(pass1_fbo_);
        context_->set_frame_buffer(pass1_fbo_);
        context_->bind_program(vis_xyz_pass1_shader_);
        context_->set_blend_state(color_no_blending_state_);
        context_->set_depth_stencil_state(depth_state_less_);

        set_uniforms(vis_xyz_pass1_shader_);
        context_->set_viewport(scm::gl::viewport(scm::math::vec2f(traits->x, traits->y), scm::math::vec2f(traits->width, traits->height), scm::math::vec2f(0, 1)));
        context_->apply();

        draw_all_models(context_id, view_id, vis_xyz_pass1_shader_);
        draw_brush(vis_xyz_pass1_shader_);

        //PASS 2
        context_->clear_color_buffer(pass2_fbo_, 0, scm::math::vec4f(.0f, .0f, .0f, 0.0f));
        context_->clear_color_buffer(pass2_fbo_, 1, scm::math::vec4f(.0f, .0f, .0f, 0.0f));
        context_->clear_color_buffer(pass2_fbo_, 2, scm::math::vec4f(.0f, .0f, .0f, 0.0f));
        context_->set_frame_buffer(pass2_fbo_);
        context_->set_blend_state(color_blending_state_);
        context_->set_depth_stencil_state(depth_state_without_writing_);
        context_->set_rasterizer_state(no_backface_culling_rasterizer_state_);

        auto selected_pass2_shading_program = vis_xyz_pass2_shader_;
        if (settings_.enable_lighting_) {
            selected_pass2_shading_program = vis_xyz_pass2_lighting_shader_;
        }

        //context_->bind_program(selected_pass2_shading_program);
        plugin->set_uniforms(selected_pass2_shading_program);
        context_->set_viewport(scm::gl::viewport(scm::math::vec2f(traits->x, traits->y), scm::math::vec2f(traits->width, traits->height), scm::math::vec2f(0, 1)));
        context_->apply();

        plugin->draw_all_models(context_id, view_id, selected_pass2_shading_program);
        plugin->draw_brush(selected_pass2_shading_program);

        //PASS 3
        context_->clear_color_buffer(fbo_, 0, scm::math::vec4f(0.0, 0.0, 0.0, 1.0f));
        context_->clear_depth_stencil_buffer(fbo_);
        context_->set_frame_buffer(fbo_);
        context_->set_depth_stencil_state(depth_state_disable_);

        auto selected_pass3_shading_program = vis_xyz_pass3_shader_;

        if (settings_.enable_lighting_) {
            selected_pass3_shading_program = vis_xyz_pass3_lighting_shader_;
        }

        context_->bind_program(selected_pass3_shading_program);
        plugin->set_uniforms(selected_pass3_shading_program);
        selected_pass3_shading_program->uniform("background_color", settings_.background_color_);
        selected_pass3_shading_program->uniform_sampler("in_color_texture", 0);
        context_->bind_texture(pass2_color_buffer_, filter_nearest_, 0);

        if (settings_.enable_lighting_) {
            context_->bind_texture(pass2_normal_buffer_, filter_nearest_, 1);
            context_->bind_texture(pass2_view_space_pos_buffer_, filter_nearest_, 2);
        }

        context_->set_viewport(scm::gl::viewport(scm::math::vec2f(traits->x, traits->y), scm::math::vec2f(traits->width, traits->height), scm::math::vec2f(0, 1)));
        context_->apply();
        screen_quad_->draw(context_);
        context_->clear_color_buffer(pass1_fbo_, 0, scm::math::vec4f(.0f, .0f, .0f, 0.0f));
    }
    else {
        //single pass
        context_->clear_color_buffer(fbo_, 0, scm::math::vec4f(settings_.background_color_.x, settings_.background_color_.y, settings_.background_color_.z, 1.0f));
        context_->clear_depth_stencil_buffer(fbo_);
        context_->set_frame_buffer(fbo_);

        auto selected_single_pass_shading_program = vis_xyz_shader_;
        if (settings_.enable_lighting_) {
            selected_single_pass_shading_program = vis_xyz_lighting_shader_;
        }

        context_->bind_program(selected_single_pass_shading_program);
        context_->set_blend_state(color_no_blending_state_);
        context_->set_depth_stencil_state(depth_state_less_);

        set_uniforms(selected_single_pass_shading_program);
        if (settings_.background_image_ != "") {
            context_->bind_texture(bg_texture_, filter_linear_, 0);
            selected_single_pass_shading_program->uniform("background_image", true);
        }

        context_->set_viewport(scm::gl::viewport(scm::math::vec2f(traits->x, traits->y), scm::math::vec2f(traits->width, traits->height), scm::math::vec2f(0, 1)));
        context_->apply();

        plugin->draw_all_models(context_id, view_id, selected_single_pass_shading_program);
        context_->bind_program(vis_xyz_shader_);
        //plugin->draw_brush(vis_xyz_shader_);
        //plugin->draw_resources(context_id, view_id);
    }

    //PASS 4: fullscreen quad
    context_->clear_default_depth_stencil_buffer();
    context_->clear_default_color_buffer();
    context_->set_default_frame_buffer();
    context_->set_depth_stencil_state(depth_state_disable_);
    context_->bind_program(vis_quad_shader_);
    context_->bind_texture(fbo_color_buffer_, filter_linear_, 0);
    vis_quad_shader_->uniform("gamma_correction", (bool)settings_.gamma_correction_);
    context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(settings_.width_, settings_.height_)));
    context_->set_viewport(scm::gl::viewport(scm::math::vec2f(traits->x, traits->y), scm::math::vec2f(traits->width, traits->height), scm::math::vec2f(0, 1)));
    context_->apply();
    screen_quad_->draw(context_);

    rendering_ = false;
    frame_time_.stop();
    frame_time_.start();
    //schism bug ? time::to_seconds yields milliseconds
    if (scm::time::to_seconds(frame_time_.accumulated_duration()) > 100.0) {
        fps_ = 1000.0f / scm::time::to_seconds(frame_time_.average_duration());
        frame_time_.reset();
    }
}

void LamurePointCloudPlugin::draw_all_models(const lamure::context_t context_id, const lamure::view_t view_id, scm::gl::program_ptr shader) {

    const osg::GraphicsContext::Traits* traits = coVRConfig::instance()->windows[0].context->getTraits();
    lamure::ren::controller* controller = lamure::ren::controller::get_instance();
    lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
    lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
    lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();

    if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
        context_->bind_vertex_array(controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, device_, data_provenance_));
    }
    else {
        context_->bind_vertex_array(controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, device_));
    }
    context_->apply();
    for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
        if (selection_.selected_model_ != -1) {
            model_id = selection_.selected_model_;
        }
        bool draw = true;
        if (settings_.show_sparse_ && sparse_resources_[model_id].num_primitives_ > 0) {
            if (selection_.selected_model_ != -1) break;
            //else continue; //don't show lod when sparse is already shown
            else draw = false;
        }
        lamure::model_t m_id = controller->deduce_model_id(std::to_string(model_id));
        lamure::ren::cut& cut = cuts->get_cut(context_id, view_id, m_id);
        std::vector<lamure::ren::cut::node_slot_aggregate> renderable = cut.complete_set();
        const lamure::ren::bvh* bvh = database->get_model(m_id)->get_bvh();
        if (bvh->get_primitive() != lamure::ren::bvh::primitive_type::POINTCLOUD) {
            if (selection_.selected_model_ != -1) break;
            //else continue;
            else draw = false;
        }

        //uniforms per model
        scm::math::mat4d model_matrix = model_transformations_[model_id];
        scm::math::mat4d projection_matrix = scm::math::mat4d(camera_->get_projection_matrix());
        scm::math::mat4d view_matrix = camera_->get_high_precision_view_matrix();
        scm::math::mat4d model_view_matrix = view_matrix * model_matrix;
        scm::math::mat4d model_view_projection_matrix = projection_matrix * model_view_matrix;

        shader->uniform("mvp_matrix", scm::math::mat4f(model_view_projection_matrix));
        shader->uniform("model_matrix", scm::math::mat4f(model_matrix));
        shader->uniform("model_view_matrix", scm::math::mat4f(model_view_matrix));
        shader->uniform("inv_mv_matrix", scm::math::mat4f(scm::math::transpose(scm::math::inverse(model_view_matrix))));

        const scm::math::mat4d viewport_scale = scm::math::make_scale(traits->width * 0.5, traits->width * 0.5, 0.5);
        const scm::math::mat4d viewport_translate = scm::math::make_translation(1.0, 1.0, 1.0);
        const scm::math::mat4d model_to_screen = viewport_scale * viewport_translate * model_view_projection_matrix;
        shader->uniform("model_to_screen_matrix", scm::math::mat4f(model_to_screen));

        //scm::math::vec4d x_unit_vec = scm::math::vec4d(1.0,0.0,0.0,0.0);
        //float model_radius_scale = scm::math::length(scm::math::vec3d(model_matrix * x_unit_vec));
        //shader->uniform("model_radius_scale", model_radius_scale);
        shader->uniform("model_radius_scale", 1.f);

        size_t surfels_per_node = database->get_primitives_per_node();
        std::vector<scm::gl::boxf>const& bounding_box_vector = bvh->get_bounding_boxes();
        scm::gl::frustum frustum_by_model = camera_->get_frustum_by_model(scm::math::mat4f(model_matrix));

        //osg::ref_ptr<osg::State> state = new osg::State();
        //state->setVertexPointer(renderable.size(), GL_POINT, 32, nullptr, GL_FALSE);
        //osg::BufferTemplate<std::vector<lamure::ren::dataset::serialized_surfel>>* bt(new osg::BufferTemplate<std::vector<lamure::ren::dataset::serialized_surfel>>);
        //ref_ptr<osg::Geometry> points(new osg::Geometry);
        //unsigned int num = points->getNumPrimitiveSets();
        //for (unsigned int i = 0; i < num; i++) {
        //    points->removePrimitiveSet(i);
        //};

        //plugin->transform->addChild(points);
        osg::ref_ptr<osg::Vec3Array> p(new osg::Vec3Array());
        osg::ref_ptr<osg::Vec3uiArray> c(new osg::Vec3uiArray());
        osg::ref_ptr<osg::Vec3Array> n(new osg::Vec3Array());

        //osg::ref_ptr<osg::Vec3Array> pp(new osg::Vec3Array());
        //osg::ref_ptr<osg::Vec3Array> cc(new osg::Vec3Array());
        //osg::ref_ptr<osg::Vec2Array> nn(new osg::Vec2Array());

        for (auto const& node_slot_aggregate : renderable) {
            uint32_t node_culling_result = camera_->cull_against_frustum(frustum_by_model, bounding_box_vector[node_slot_aggregate.node_id_]);
            if (node_culling_result != 1) {
                if (settings_.use_pvs_ && pvs->is_activated() && settings_.pvs_culling_
                    && !lamure::pvs::pvs_database::get_instance()->get_viewer_visibility(model_id, node_slot_aggregate.node_id_)) {
                    continue;
                }
                if (settings_.show_accuracy_) {
                    const float accuracy = 1.0 - (bvh->get_depth_of_node(node_slot_aggregate.node_id_) * 1.0) / (bvh->get_depth() - 1);
                    shader->uniform("accuracy", accuracy);
                }
                if (settings_.show_radius_deviation_) {
                    shader->uniform("average_radius", bvh->get_avg_primitive_extent(node_slot_aggregate.node_id_));
                }
                context_->apply();

                if (draw) {
                    lamure::ren::ooc_cache* ooc_cache = lamure::ren::ooc_cache::get_instance();
                    bool loaded = ooc_cache->is_node_resident_and_aquired(model_id, node_slot_aggregate.node_id_);
                    lamure::ren::dataset::serialized_surfel* surfels = (lamure::ren::dataset::serialized_surfel*)ooc_cache->node_data(model_id, node_slot_aggregate.node_id_);
                    //lamure::ren::dataset::serialized_vertex* prov = (lamure::ren::dataset::serialized_vertex*)ooc_cache->node_data_provenance(model_id, node_slot_aggregate.node_id_);
                    for (uint32_t i = 0; i < 30; ++i) {
                        auto s = surfels[i];
                        p->push_back(osg::Vec3f(surfels[i].x, surfels[i].y, surfels[i].z));
                        c->push_back(osg::Vec3ui(surfels[i].r, surfels[i].g, surfels[i].b));
                        n->push_back(osg::Vec3f(surfels[i].nx, surfels[i].ny, surfels[i].nz));
                        //pp->push_back(osg::Vec3f(prov[i].v_x_, prov[i].v_y_, prov[i].v_z_));
                        //cc->push_back(osg::Vec3f(prov[i].n_x_, prov[i].n_y_, prov[i].n_z_));
                        //nn->push_back(osg::Vec2f(prov[i].c_x_, prov[i].c_y_));
                    };
                    //points->setVertexArray(p);
                    //points->setColorArray(c);
                    //points->setNormalArray(n);
                    //points->setColorBinding(osg::Geometry::BIND_OVERALL);
                    //points->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(10.0f), osg::StateAttribute::ON);
                    //points->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, surfels_per_node));
                    context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, (node_slot_aggregate.slot_id_) * (GLsizei)surfels_per_node, surfels_per_node);

                    osg::GLExtensions* glapi = new osg::GLExtensions(0);
                    scm::gl::vertex_array_ptr vap = context_->current_vertex_array();
                    //scm::gl::render_context::binding_state_type bst = context_->binding_state_type;

                    osg::Uniform* uni = new osg::Uniform();

                    unsigned int id = 0;
                    //new osg::GLBufferObject(id, vap.get(), id);



                    scm::gl::frame_buffer_ptr fbp = context_->current_frame_buffer();

                    rendered_splats_ += surfels_per_node;
                    ++rendered_nodes_;
                }
            }
        }
        if (selection_.selected_model_ != -1) {
            break;
        }
    }
}

void LamurePointCloudPlugin::create_aux_resources_buffered() {
    osg::Geometry* aux_geo = new osg::Geometry();
    plugin->geode->addDrawable(aux_geo);
    if (!settings_.create_aux_resources_) { return; }
    //create bvh representation
    for (uint32_t model_id = 0; model_id < num_models_; ++model_id) {
        const auto& bounding_boxes = lamure::ren::model_database::get_instance()->get_model(model_id)->get_bvh()->get_bounding_boxes();

        aux_geo->setUseDisplayList(false);
        aux_geo->setUseVertexBufferObjects(true);
        aux_geo->setUseVertexArrayObject(false);

        //osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(bounding_boxes.size()*2);
        osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_LINES, bounding_boxes.size() * 24);
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(bounding_boxes.size() * 8);

        for (uint64_t node_id = 0; node_id < bounding_boxes.size(); ++node_id) {
            scm::math::vec3f min_vertex = bounding_boxes[node_id].min_vertex();
            scm::math::vec3f max_vertex = bounding_boxes[node_id].max_vertex();

            (*vertices)[(node_id * 8) + 0] = osg::Vec3f(min_vertex.x, min_vertex.y, min_vertex.z);
            (*vertices)[(node_id * 8) + 1] = osg::Vec3f(max_vertex.x, min_vertex.y, min_vertex.z);
            (*vertices)[(node_id * 8) + 2] = osg::Vec3f(max_vertex.x, min_vertex.y, max_vertex.z);
            (*vertices)[(node_id * 8) + 3] = osg::Vec3f(min_vertex.x, min_vertex.y, max_vertex.z);

            (*vertices)[(node_id * 8) + 4] = osg::Vec3f(min_vertex.x, max_vertex.y, min_vertex.z);
            (*vertices)[(node_id * 8) + 5] = osg::Vec3f(max_vertex.x, max_vertex.y, min_vertex.z);
            (*vertices)[(node_id * 8) + 6] = osg::Vec3f(max_vertex.x, max_vertex.y, max_vertex.z);
            (*vertices)[(node_id * 8) + 7] = osg::Vec3f(min_vertex.x, max_vertex.y, max_vertex.z);

            (*indices)[(node_id * 24) + 0] = (node_id * 8) + 0;   (*indices)[(node_id * 24) + 1] = (node_id * 8) + 1;
            (*indices)[(node_id * 24) + 2] = (node_id * 8) + 1;   (*indices)[(node_id * 24) + 3] = (node_id * 8) + 2;
            (*indices)[(node_id * 24) + 4] = (node_id * 8) + 2;   (*indices)[(node_id * 24) + 5] = (node_id * 8) + 3;
            (*indices)[(node_id * 24) + 6] = (node_id * 8) + 3;   (*indices)[(node_id * 24) + 7] = (node_id * 8) + 0;

            (*indices)[(node_id * 24) + 8] = (node_id * 8) + 4;   (*indices)[(node_id * 24) + 9] = (node_id * 8) + 5;
            (*indices)[(node_id * 24) + 10] = (node_id * 8) + 5;  (*indices)[(node_id * 24) + 11] = (node_id * 8) + 6;
            (*indices)[(node_id * 24) + 12] = (node_id * 8) + 6;  (*indices)[(node_id * 24) + 13] = (node_id * 8) + 7;
            (*indices)[(node_id * 24) + 14] = (node_id * 8) + 7;  (*indices)[(node_id * 24) + 15] = (node_id * 8) + 4;

            (*indices)[(node_id * 24) + 16] = (node_id * 8) + 0;  (*indices)[(node_id * 24) + 17] = (node_id * 8) + 4;
            (*indices)[(node_id * 24) + 18] = (node_id * 8) + 1;  (*indices)[(node_id * 24) + 19] = (node_id * 8) + 5;
            (*indices)[(node_id * 24) + 20] = (node_id * 8) + 2;  (*indices)[(node_id * 24) + 21] = (node_id * 8) + 6;
            (*indices)[(node_id * 24) + 22] = (node_id * 8) + 3;  (*indices)[(node_id * 24) + 23] = (node_id * 8) + 7;
        }
        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
        colors->push_back(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
        aux_geo->setVertexArray(vertices.get());
        aux_geo->setColorArray(colors.get());
        aux_geo->setColorBinding(osg::Geometry::BIND_OVERALL);
        aux_geo->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(1.0f), osg::StateAttribute::ON);
        aux_geo->addPrimitiveSet(indices.get());
    }
}

void LamurePointCloudPlugin::create_aux_resources() {
    osg::Geometry* aux_geo = new osg::Geometry();
    plugin->geode->addDrawable(aux_geo);
    if (!settings_.create_aux_resources_) { return; }

    //create bvh representation
    /*for (uint32_t model_id = 0; model_id < num_models_; ++model_id) {
        const auto& bounding_boxes = lamure::ren::model_database::get_instance()->get_model(model_id)->get_bvh()->get_bounding_boxes();

        aux_geo->setUseDisplayList(false);
        aux_geo->setUseVertexBufferObjects(true);
        aux_geo->setUseVertexArrayObject(false);

        osg::ref_ptr<osg::Vec3Array> lines = new osg::Vec3Array();
        for (uint64_t node_id = 0; node_id < bounding_boxes.size(); ++node_id) {
            const auto& node = bounding_boxes[node_id];
            lines_from_min_max(node.min_vertex(), node.max_vertex(), lines);
        }
        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
        colors->push_back(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
        aux_geo->setVertexArray(lines.get());
        aux_geo->setColorArray(colors.get());
        aux_geo->setColorBinding(osg::Geometry::BIND_OVERALL);
        aux_geo->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(1.0f), osg::StateAttribute::ON);
        aux_geo->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, bounding_boxes.size() * 12));
    }*/
    
    //create bvh representation
    for (uint32_t model_id = 0; model_id < num_models_; ++model_id) {
        const auto& bounding_boxes = lamure::ren::model_database::get_instance()->get_model(model_id)->get_bvh()->get_bounding_boxes();

        resource bvh_line_resource;
        bvh_line_resource.buffer_.reset();
        bvh_line_resource.array_.reset();

        std::vector<scm::math::vec3f> bvh_lines_to_upload;
        for (uint64_t node_id = 0; node_id < bounding_boxes.size(); ++node_id) {
            const auto& node = bounding_boxes[node_id];

            lines_from_min_max(node.min_vertex(), node.max_vertex(), bvh_lines_to_upload);

        }

        bvh_line_resource.buffer_ = device_->create_buffer(scm::gl::BIND_VERTEX_BUFFER, 
            scm::gl::USAGE_STATIC_DRAW, (sizeof(float) * 3) * bvh_lines_to_upload.size(), &bvh_lines_to_upload[0]);
        bvh_line_resource.array_ = device_->create_vertex_array(scm::gl::vertex_format
            (0, 0, scm::gl::TYPE_VEC3F, sizeof(float) * 3), 
            boost::assign::list_of(bvh_line_resource.buffer_));

        bvh_line_resource.num_primitives_ = bvh_lines_to_upload.size();
        bvh_resources_[model_id] = bvh_line_resource;
    } 
     
    /*
    //create auxiliary representations
    for (const auto& aux_file : settings_.aux_) {
        if (aux_file.second != "") {
            uint32_t model_id = aux_file.first;
            std::cout << "aux: " << aux_file.second << std::endl;
            lamure::prov::aux aux(aux_file.second);
            provenance_[model_id].num_views_ = aux.get_num_views();
            std::cout << "aux: " << aux.get_num_views() << " views" << std::endl;
            std::cout << "aux: " << aux.get_num_sparse_points() << " points" << std::endl;
            std::cout << "aux: " << aux.get_atlas().atlas_width_ << ", " << aux.get_atlas().atlas_height_ << " is it rotated? : " << aux.get_atlas().rotated_ << std::endl;
            std::cout << "aux: " << aux.get_num_atlas_tiles() << " atlas tiles" << std::endl;
            std::vector<xyz> points_to_upload;
            for (uint32_t i = 0; i < aux.get_num_views(); ++i) {
                const auto& view = aux.get_view(i);
                points_to_upload.push_back(
                    xyz{ view.position_,
                      (uint8_t)255, (uint8_t)240, (uint8_t)0, (uint8_t)255,
                      settings_.aux_point_size_,
                      scm::math::vec3f(1.0, 0.0, 0.0) } //placeholder
                );
                settings_.views_[model_id].push_back(view);
            }

            for (uint32_t i = 0; i < aux.get_num_sparse_points(); ++i) {
                const auto& point = aux.get_sparse_point(i);
                points_to_upload.push_back(
                    xyz{ point.pos_,
                      point.r_, point.g_, point.b_, point.a_,
                      settings_.aux_point_size_,
                      scm::math::vec3f(1.0, 0.0, 0.0) } //placeholder
                );
            }

            resource points_resource;
            points_resource.num_primitives_ = points_to_upload.size();
            points_resource.buffer_.reset();
            points_resource.array_.reset();

            points_resource.buffer_ = device_->create_buffer(
                scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, sizeof(xyz) * points_to_upload.size(), &points_to_upload[0]);
            points_resource.array_ = device_->create_vertex_array(scm::gl::vertex_format
            (0, 0, scm::gl::TYPE_VEC3F, sizeof(xyz))
                (0, 1, scm::gl::TYPE_UBYTE, sizeof(xyz), scm::gl::INT_FLOAT_NORMALIZE)
                (0, 2, scm::gl::TYPE_UBYTE, sizeof(xyz), scm::gl::INT_FLOAT_NORMALIZE)
                (0, 3, scm::gl::TYPE_UBYTE, sizeof(xyz), scm::gl::INT_FLOAT_NORMALIZE)
                (0, 4, scm::gl::TYPE_UBYTE, sizeof(xyz), scm::gl::INT_FLOAT_NORMALIZE)
                (0, 5, scm::gl::TYPE_FLOAT, sizeof(xyz))
                (0, 6, scm::gl::TYPE_VEC3F, sizeof(xyz)),
                boost::assign::list_of(points_resource.buffer_));

            sparse_resources_[model_id] = points_resource;

            //init octree
            settings_.octrees_[model_id] = aux.get_octree();
            std::cout << "Octree loaded (" << settings_.octrees_[model_id]->get_num_nodes() << " nodes)" << std::endl;

            //init octree buffers
            resource octree_resource;
            octree_resource.buffer_.reset();
            octree_resource.array_.reset();

            std::vector<scm::math::vec3f> octree_lines_to_upload;
            for (uint64_t i = 0; i < settings_.octrees_[model_id]->get_num_nodes(); ++i) {
                const auto& node = settings_.octrees_[model_id]->get_node(i);
                lines_from_min_max(node.get_min(), node.get_max(), octree_lines_to_upload);
            }

            octree_resource.buffer_ = device_->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                scm::gl::USAGE_STATIC_DRAW, (sizeof(float) * 3) * octree_lines_to_upload.size(), &octree_lines_to_upload[0]);
            octree_resource.array_ = device_->create_vertex_array(scm::gl::vertex_format
            (0, 0, scm::gl::TYPE_VEC3F, sizeof(float) * 3),
                boost::assign::list_of(octree_resource.buffer_));

            octree_resource.num_primitives_ = octree_lines_to_upload.size();
            octree_resources_[model_id] = octree_resource;

            auto root_bb = model_database::get_instance()->get_model(model_id)->get_bvh()->get_bounding_boxes()[0];
            auto root_bb_min = scm::math::mat4f(model_transformations_[model_id]) * root_bb.min_vertex();
            auto root_bb_max = scm::math::mat4f(model_transformations_[model_id]) * root_bb.max_vertex();
            auto model_dim = scm::math::length(root_bb_max - root_bb_min);

            //for image planes
            if (!settings_.atlas_file_.empty()) {
                if (aux.get_num_atlas_tiles() != aux.get_num_views()) {
                    throw std::runtime_error(
                        "Number of atlas_tiles (" + std::to_string(aux.get_num_atlas_tiles()) + ") "
                        + "does not match number of views (" + std::to_string(aux.get_num_views()) + ")");
                }

                std::vector<vertex> triangles_to_upload;
                for (uint32_t i = 0; i < aux.get_num_views(); ++i) {
                    const auto& view = aux.get_view(i);
                    const auto& atlas_tile = aux.get_atlas_tile(i);

                    float aspect_ratio = view.image_height_ / (float)view.image_width_;
                    float img_w_half = (settings_.aux_focal_length_) * 0.5f;
                    float img_h_half = img_w_half * aspect_ratio;
                    float focal_length = settings_.aux_focal_length_;

                    float atlas_width = aux.get_atlas().atlas_width_;
                    float atlas_height = aux.get_atlas().atlas_height_;

                    // scale factor from image space to vt atlas space
                    float factor = get_atlas_scale_factor();

                    // positions in vt atlas space coordinate system
                    float tile_height = (float)atlas_tile.width_ / atlas_width * factor;
                    float tile_width = (float)atlas_tile.width_ / atlas_height * factor;

                    float tile_pos_x = (float)atlas_tile.x_ / atlas_height * factor;
                    float tile_pos_y = (float)atlas_tile.y_ / atlas_tile.height_ * tile_height + (1 - factor);


                    vertex p1;
                    p1.pos_ = view.transform_ * scm::math::vec3f(-img_w_half, img_h_half, -focal_length);
                    p1.uv_ = scm::math::vec2f(tile_pos_x + tile_width, tile_pos_y);

                    vertex p2;
                    p2.pos_ = view.transform_ * scm::math::vec3f(img_w_half, img_h_half, -focal_length);
                    p2.uv_ = scm::math::vec2f(tile_pos_x, tile_pos_y);

                    vertex p3;
                    p3.pos_ = view.transform_ * scm::math::vec3f(-img_w_half, -img_h_half, -focal_length);
                    p3.uv_ = scm::math::vec2f(tile_pos_x + tile_width, tile_pos_y + tile_height);

                    vertex p4;
                    p4.pos_ = view.transform_ * scm::math::vec3f(img_w_half, -img_h_half, -focal_length);
                    p4.uv_ = scm::math::vec2f(tile_pos_x, tile_pos_y + tile_height);

                    // left quad triangle
                    triangles_to_upload.push_back(p1);
                    triangles_to_upload.push_back(p4);
                    triangles_to_upload.push_back(p3);

                    // right quad triangle
                    triangles_to_upload.push_back(p2);
                    triangles_to_upload.push_back(p4);
                    triangles_to_upload.push_back(p1);
                }

                //init triangle buffer
                resource triangles_resource;
                triangles_resource.buffer_.reset();
                triangles_resource.array_.reset();

                triangles_resource.buffer_ = device_->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                    scm::gl::USAGE_STATIC_DRAW,
                    (sizeof(vertex)) * triangles_to_upload.size(),
                    &triangles_to_upload[0]);

                triangles_resource.array_ = device_->create_vertex_array(scm::gl::vertex_format
                (0, 0, scm::gl::TYPE_VEC3F, sizeof(vertex))
                    (0, 1, scm::gl::TYPE_VEC2F, sizeof(vertex)),
                    boost::assign::list_of(triangles_resource.buffer_));


                triangles_resource.num_primitives_ = triangles_to_upload.size();

                image_plane_resources_[model_id] = triangles_resource;
            }

            //init line buffers
            resource lines_resource;
            lines_resource.buffer_.reset();
            lines_resource.array_.reset();

            std::vector<scm::math::vec3f> lines_to_upload;
            for (uint32_t i = 0; i < aux.get_num_views(); ++i) {
                const auto& view = aux.get_view(i);

                float aspect_ratio = view.image_height_ / (float)view.image_width_;
                float img_w_half = (settings_.aux_focal_length_) * 0.5f;
                float img_h_half = img_w_half * aspect_ratio;
                float focal_length = settings_.aux_focal_length_;

                lines_to_upload.push_back(view.transform_ * scm::math::vec3f(-img_w_half, img_h_half, -focal_length));
                lines_to_upload.push_back(view.transform_ * scm::math::vec3f(img_w_half, img_h_half, -focal_length));

                lines_to_upload.push_back(view.transform_ * scm::math::vec3f(img_w_half, img_h_half, -focal_length));
                lines_to_upload.push_back(view.transform_ * scm::math::vec3f(img_w_half, -img_h_half, -focal_length));

                lines_to_upload.push_back(view.transform_ * scm::math::vec3f(img_w_half, -img_h_half, -focal_length));
                lines_to_upload.push_back(view.transform_ * scm::math::vec3f(-img_w_half, -img_h_half, -focal_length));

                lines_to_upload.push_back(view.transform_ * scm::math::vec3f(-img_w_half, -img_h_half, -focal_length));
                lines_to_upload.push_back(view.transform_ * scm::math::vec3f(-img_w_half, img_h_half, -focal_length));

                lines_to_upload.push_back(view.transform_ * scm::math::vec3f(0.f));
                lines_to_upload.push_back(view.transform_ * scm::math::vec3f(-img_w_half, img_h_half, -focal_length));

                lines_to_upload.push_back(view.transform_ * scm::math::vec3f(0.f));
                lines_to_upload.push_back(view.transform_ * scm::math::vec3f(img_w_half, img_h_half, -focal_length));

                lines_to_upload.push_back(view.transform_ * scm::math::vec3f(0.f));
                lines_to_upload.push_back(view.transform_ * scm::math::vec3f(img_w_half, -img_h_half, -focal_length));

                lines_to_upload.push_back(view.transform_ * scm::math::vec3f(0.f));
                lines_to_upload.push_back(view.transform_ * scm::math::vec3f(-img_w_half, -img_h_half, -focal_length));

            }

            lines_resource.buffer_ = device_->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                scm::gl::USAGE_STATIC_DRAW, (sizeof(float) * 3) * lines_to_upload.size(), &lines_to_upload[0]);
            lines_resource.array_ = device_->create_vertex_array(scm::gl::vertex_format
            (0, 0, scm::gl::TYPE_VEC3F, sizeof(float) * 3),
                boost::assign::list_of(lines_resource.buffer_));

            lines_resource.num_primitives_ = lines_to_upload.size();

            frusta_resources_[model_id] = lines_resource;
        }
    }*/
}


void LamurePointCloudPlugin::lines_from_min_max(const scm::math::vec3f& min_vertex, const scm::math::vec3f& max_vertex, std::vector<scm::math::vec3f> lines) {

    lines.push_back(scm::math::vec3f(min_vertex.x, min_vertex.y, min_vertex.z));
    lines.push_back(scm::math::vec3f(max_vertex.x, min_vertex.y, min_vertex.z));

    lines.push_back(scm::math::vec3f(max_vertex.x, min_vertex.y, min_vertex.z));
    lines.push_back(scm::math::vec3f(max_vertex.x, min_vertex.y, max_vertex.z));

    lines.push_back(scm::math::vec3f(max_vertex.x, min_vertex.y, max_vertex.z));
    lines.push_back(scm::math::vec3f(min_vertex.x, min_vertex.y, max_vertex.z));

    lines.push_back(scm::math::vec3f(min_vertex.x, min_vertex.y, max_vertex.z));
    lines.push_back(scm::math::vec3f(min_vertex.x, min_vertex.y, min_vertex.z));


    lines.push_back(scm::math::vec3f(min_vertex.x, max_vertex.y, min_vertex.z));
    lines.push_back(scm::math::vec3f(max_vertex.x, max_vertex.y, min_vertex.z));

    lines.push_back(scm::math::vec3f(max_vertex.x, max_vertex.y, min_vertex.z));
    lines.push_back(scm::math::vec3f(max_vertex.x, max_vertex.y, max_vertex.z));

    lines.push_back(scm::math::vec3f(max_vertex.x, max_vertex.y, max_vertex.z));
    lines.push_back(scm::math::vec3f(min_vertex.x, max_vertex.y, max_vertex.z));

    lines.push_back(scm::math::vec3f(min_vertex.x, max_vertex.y, max_vertex.z));
    lines.push_back(scm::math::vec3f(min_vertex.x, max_vertex.y, min_vertex.z));


    lines.push_back(scm::math::vec3f(min_vertex.x, min_vertex.y, min_vertex.z));
    lines.push_back(scm::math::vec3f(min_vertex.x, max_vertex.y, min_vertex.z));

    lines.push_back(scm::math::vec3f(max_vertex.x, min_vertex.y, min_vertex.z));
    lines.push_back(scm::math::vec3f(max_vertex.x, max_vertex.y, min_vertex.z));

    lines.push_back(scm::math::vec3f(max_vertex.x, min_vertex.y, max_vertex.z));
    lines.push_back(scm::math::vec3f(max_vertex.x, max_vertex.y, max_vertex.z));

    lines.push_back(scm::math::vec3f(min_vertex.x, min_vertex.y, max_vertex.z));
    lines.push_back(scm::math::vec3f(min_vertex.x, max_vertex.y, max_vertex.z));
}


void LamurePointCloudPlugin::lines_from_min_max_buffered(const scm::math::vec3f& min_vertex, const scm::math::vec3f& max_vertex, osg::ref_ptr<osg::Vec3Array>& lines) {

    lines->push_back(osg::Vec3f(min_vertex.x, min_vertex.y, min_vertex.z));
    lines->push_back(osg::Vec3f(max_vertex.x, min_vertex.y, min_vertex.z));

    lines->push_back(osg::Vec3f(max_vertex.x, min_vertex.y, min_vertex.z));
    lines->push_back(osg::Vec3f(max_vertex.x, min_vertex.y, max_vertex.z));

    lines->push_back(osg::Vec3f(max_vertex.x, min_vertex.y, max_vertex.z));
    lines->push_back(osg::Vec3f(min_vertex.x, min_vertex.y, max_vertex.z));

    lines->push_back(osg::Vec3f(min_vertex.x, min_vertex.y, max_vertex.z));
    lines->push_back(osg::Vec3f(min_vertex.x, min_vertex.y, min_vertex.z));


    lines->push_back(osg::Vec3f(min_vertex.x, max_vertex.y, min_vertex.z));
    lines->push_back(osg::Vec3f(max_vertex.x, max_vertex.y, min_vertex.z));

    lines->push_back(osg::Vec3f(max_vertex.x, max_vertex.y, min_vertex.z));
    lines->push_back(osg::Vec3f(max_vertex.x, max_vertex.y, max_vertex.z));

    lines->push_back(osg::Vec3f(max_vertex.x, max_vertex.y, max_vertex.z));
    lines->push_back(osg::Vec3f(min_vertex.x, max_vertex.y, max_vertex.z));

    lines->push_back(osg::Vec3f(min_vertex.x, max_vertex.y, max_vertex.z));
    lines->push_back(osg::Vec3f(min_vertex.x, max_vertex.y, min_vertex.z));


    lines->push_back(osg::Vec3f(min_vertex.x, min_vertex.y, min_vertex.z));
    lines->push_back(osg::Vec3f(min_vertex.x, max_vertex.y, min_vertex.z));

    lines->push_back(osg::Vec3f(max_vertex.x, min_vertex.y, min_vertex.z));
    lines->push_back(osg::Vec3f(max_vertex.x, max_vertex.y, min_vertex.z));

    lines->push_back(osg::Vec3f(max_vertex.x, min_vertex.y, max_vertex.z));
    lines->push_back(osg::Vec3f(max_vertex.x, max_vertex.y, max_vertex.z));

    lines->push_back(osg::Vec3f(min_vertex.x, min_vertex.y, max_vertex.z));
    lines->push_back(osg::Vec3f(min_vertex.x, max_vertex.y, max_vertex.z));
}


void LamurePointCloudPlugin::init_camera() {
    const osg::GraphicsContext::Traits* traits = coVRConfig::instance()->windows[0].context->getTraits();
    osg::Camera* osg_cam_ = VRViewer::instance()->getCamera();

    auto root_bb = lamure::ren::model_database::get_instance()->get_model(0)->get_bvh()->get_bounding_boxes()[0];
    auto root_bb_min = scm::math::mat4f(model_transformations_[0]) * root_bb.min_vertex();
    auto root_bb_max = scm::math::mat4f(model_transformations_[0]) * root_bb.max_vertex();
    scm::math::vec3f center = (root_bb_min + root_bb_max) / 2.f;

    double fovy, aspectRatio, zNear, zFar;
    bool proj_success = osg_cam_->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);

    osg::Matrixd& view_mat_osg = osg_cam_->getViewMatrix();
    scm::math::mat4f view_mat = matConvF(view_mat_osg);

    osg::Matrixd& proj_mat_osg = osg_cam_->getProjectionMatrix();
    scm::math::mat4f proj_mat = matConvF(proj_mat_osg);

    /*camera_ = boost::shared_ptr<lamure::ren::camera>(new lamure::ren::camera(
        0,
        zNear,
        zFar,
        view_mat,
        proj_mat,
        make_look_at_matrix(center + scm::math::vec3f(0.f, 0.1f, -0.01f), center, scm::math::vec3f(0.f, 1.f, 0.f)),
        length(root_bb_max - root_bb_min),
        false,
        false));*/
    camera_ = boost::shared_ptr<lamure::ren::camera>(new lamure::ren::camera());
    screen_quad_.reset(new scm::gl::quad_geometry(device_, scm::math::vec2f(-1.0f, -1.0f), scm::math::vec2f(1.0f, 1.0f)));

    //camera_ = new camera(0, make_look_at_matrix(center + scm::math::vec3f(0.f, 0.1f, -0.01f), center, scm::math::vec3f(0.f, 1.f, 0.f)), length(root_bb_max - root_bb_min));
    //camera_->set_dolly_sens_(settings_.travel_speed_);
    //if (settings_.use_view_tf_) {
    //    camera_->set_view_matrix(settings_.view_tf_);
    //    std::cout << "view_tf:" << std::endl;
    //    std::cout << camera_->get_high_precision_view_matrix() << std::endl;
    //    camera_->set_dolly_sens_(settings_.travel_speed_);
    //}
}

bool LamurePointCloudPlugin::read_shader(std::string const& path_string, std::string& shader_string, bool keep_optional_shader_code = false) {
    if (!boost::filesystem::exists(path_string)) {
        std::cout << "WARNING: File " << path_string << "does not exist." << std::endl;
        return false;
    }
    std::ifstream shader_source(path_string, std::ios::in);
    std::string line_buffer;
    std::string include_prefix("INCLUDE");
    std::string optional_begin_prefix("OPTIONAL_BEGIN");
    std::string optional_end_prefix("OPTIONAL_END");
    std::size_t slash_position = path_string.find_last_of("/\\");
    std::string const base_path = path_string.substr(0, slash_position + 1);

    bool disregard_code = false;
    while (std::getline(shader_source, line_buffer)) {
        line_buffer = strip_whitespace(line_buffer);
        if (parse_prefix(line_buffer, include_prefix)) {
            if (!disregard_code || keep_optional_shader_code) {
                std::string filename_string = line_buffer;
                read_shader(base_path + filename_string, shader_string);
            }
        }
        else if (parse_prefix(line_buffer, optional_begin_prefix)) {
            disregard_code = true;
        }
        else if (parse_prefix(line_buffer, optional_end_prefix)) {
            disregard_code = false;
        }
        else {
            if ((!disregard_code) || keep_optional_shader_code) {
                shader_string += line_buffer + "\n";
            }
        }
    }
    return true;
}

void LamurePointCloudPlugin::init_lamure_shader()
{   
    std::cout << "init_lamure_shader()" << std::endl;
    try
    {
        std::string vis_quad_vs_source;
        std::string vis_quad_fs_source;
        std::string vis_line_vs_source;
        std::string vis_line_fs_source;
        std::string vis_triangle_vs_source;
        std::string vis_triangle_fs_source;
        std::string vis_vt_vs_source;
        std::string vis_vt_fs_source;
        std::string vis_xyz_vs_source;
        std::string vis_xyz_gs_source;
        std::string vis_xyz_fs_source;
        std::string vis_xyz_pass1_vs_source;
        std::string vis_xyz_pass1_gs_source;
        std::string vis_xyz_pass1_fs_source;
        std::string vis_xyz_pass2_vs_source;
        std::string vis_xyz_pass2_gs_source;
        std::string vis_xyz_pass2_fs_source;
        std::string vis_xyz_pass3_vs_source;
        std::string vis_xyz_pass3_fs_source;
        std::string vis_xyz_qz_vs_source;
        std::string vis_xyz_qz_pass1_vs_source;
        std::string vis_xyz_qz_pass2_vs_source;

        /* parsed with optional lighting code */
        std::string vis_xyz_vs_lighting_source;
        std::string vis_xyz_gs_lighting_source;
        std::string vis_xyz_fs_lighting_source;
        std::string vis_xyz_pass2_vs_lighting_source;
        std::string vis_xyz_pass2_gs_lighting_source;
        std::string vis_xyz_pass2_fs_lighting_source;
        std::string vis_xyz_pass3_vs_lighting_source;
        std::string vis_xyz_pass3_fs_lighting_source;

        std::string shader_root_path = LAMURE_SHADERS_DIR;
        if (!read_shader(shader_root_path + "/vis/vis_quad.glslv", vis_quad_vs_source)
            || !read_shader(shader_root_path + "/vis/vis_quad.glslf", vis_quad_fs_source)
            || !read_shader(shader_root_path + "/vis/vis_line.glslv", vis_line_vs_source)
            || !read_shader(shader_root_path + "/vis/vis_line.glslf", vis_line_fs_source)
            || !read_shader(shader_root_path + "/vis/vis_triangle.glslv", vis_triangle_vs_source)
            || !read_shader(shader_root_path + "/vis/vis_triangle.glslf", vis_triangle_fs_source)

            || !read_shader(shader_root_path + "/vt/virtual_texturing.glslv", vis_vt_vs_source)
            || !read_shader(shader_root_path + "/vt/virtual_texturing_hierarchical.glslf", vis_vt_fs_source)

            || !read_shader(shader_root_path + "/vis/vis_xyz.glslv", vis_xyz_vs_source)
            || !read_shader(shader_root_path + "/vis/vis_xyz.glslg", vis_xyz_gs_source)
            || !read_shader(shader_root_path + "/vis/vis_xyz.glslf", vis_xyz_fs_source)
            || !read_shader(shader_root_path + "/vis/vis_xyz_pass1.glslv", vis_xyz_pass1_vs_source)
            || !read_shader(shader_root_path + "/vis/vis_xyz_pass1.glslg", vis_xyz_pass1_gs_source)
            || !read_shader(shader_root_path + "/vis/vis_xyz_pass1.glslf", vis_xyz_pass1_fs_source)
            || !read_shader(shader_root_path + "/vis/vis_xyz_pass2.glslv", vis_xyz_pass2_vs_source)
            || !read_shader(shader_root_path + "/vis/vis_xyz_pass2.glslg", vis_xyz_pass2_gs_source)
            || !read_shader(shader_root_path + "/vis/vis_xyz_pass2.glslf", vis_xyz_pass2_fs_source)
            || !read_shader(shader_root_path + "/vis/vis_xyz_pass3.glslv", vis_xyz_pass3_vs_source)
            || !read_shader(shader_root_path + "/vis/vis_xyz_pass3.glslf", vis_xyz_pass3_fs_source)
            || !read_shader(shader_root_path + "/vis/vis_xyz_qz.glslv", vis_xyz_qz_vs_source)
            || !read_shader(shader_root_path + "/vis/vis_xyz_qz_pass1.glslv", vis_xyz_qz_pass1_vs_source)
            || !read_shader(shader_root_path + "/vis/vis_xyz_qz_pass2.glslv", vis_xyz_qz_pass2_vs_source)

            || !read_shader(shader_root_path + "/vis/vis_xyz.glslv", vis_xyz_vs_lighting_source, true)
            || !read_shader(shader_root_path + "/vis/vis_xyz.glslg", vis_xyz_gs_lighting_source, true)
            || !read_shader(shader_root_path + "/vis/vis_xyz.glslf", vis_xyz_fs_lighting_source, true)
            || !read_shader(shader_root_path + "/vis/vis_xyz_pass2.glslv", vis_xyz_pass2_vs_lighting_source, true)
            || !read_shader(shader_root_path + "/vis/vis_xyz_pass2.glslg", vis_xyz_pass2_gs_lighting_source, true)
            || !read_shader(shader_root_path + "/vis/vis_xyz_pass2.glslf", vis_xyz_pass2_fs_lighting_source, true)
            || !read_shader(shader_root_path + "/vis/vis_xyz_pass3.glslv", vis_xyz_pass3_vs_lighting_source, true)
            || !read_shader(shader_root_path + "/vis/vis_xyz_pass3.glslf", vis_xyz_pass3_fs_lighting_source, true)
            ) {
            std::cout << "error reading shader files" << std::endl;
            exit(1);
        }

        /*auto myshader = device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_quad_vs_source);
        scm::gl::shader& obj = *myshader;
        std::cout << obj << std::endl;*/
        vis_quad_shader_ = device_->create_program(
            boost::assign::list_of
            (device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_quad_vs_source))
            (device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_quad_fs_source)));

        if (!vis_quad_shader_) {
            std::cout << "error creating shader vis_quad_shader_ program" << std::endl;
            exit(1);
        }
        vis_line_shader_ = device_->create_program(
            boost::assign::list_of
            (device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_line_vs_source))
            (device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_line_fs_source)));

        if (!vis_line_shader_) {
            std::cout << "error creating shader vis_line_shader_ program" << std::endl;
            exit(1);
        }
        vis_triangle_shader_ = device_->create_program(
            boost::assign::list_of
            (device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_triangle_vs_source))
            (device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_triangle_fs_source)));

        if (!vis_triangle_shader_) {
            std::cout << "error creating shader vis_triangle_shader_ program" << std::endl;
            std::exit(1);
        }
        vis_vt_shader_ = device_->create_program(
            boost::assign::list_of
            (device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_vt_vs_source))
            (device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_vt_fs_source)));

        if (!vis_vt_shader_) {
            std::cout << "error creating shader vis_vt_shader_ program" << std::endl;
            std::exit(1);
        }
        vis_xyz_shader_ = device_->create_program(
            boost::assign::list_of
            (device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_xyz_vs_source))
            (device_->create_shader(scm::gl::STAGE_GEOMETRY_SHADER, vis_xyz_gs_source))
            (device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_xyz_fs_source)));

        if (!vis_xyz_shader_) {
            //scm::err() << scm::log::error << scm::log::end;
            std::cout << "error creating shader vis_xyz_shader_ program" << std::endl;
            exit(1);
        }
        vis_xyz_pass1_shader_ = device_->create_program(
            boost::assign::list_of
            (device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_xyz_pass1_vs_source))
            (device_->create_shader(scm::gl::STAGE_GEOMETRY_SHADER, vis_xyz_pass1_gs_source))
            (device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_xyz_pass1_fs_source)));

        if (!vis_xyz_pass1_shader_) {
            std::cout << "error creating vis_xyz_pass1_shader_ program" << std::endl;
            exit(1);
        }
        vis_xyz_pass2_shader_ = device_->create_program(
            boost::assign::list_of
            (device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_xyz_pass2_vs_source))
            (device_->create_shader(scm::gl::STAGE_GEOMETRY_SHADER, vis_xyz_pass2_gs_source))
            (device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_xyz_pass2_fs_source)));

        if (!vis_xyz_pass2_shader_) {
            std::cout << "error creating vis_xyz_pass2_shader_ program" << std::endl;
            exit(1);
        }
        vis_xyz_pass3_shader_ = device_->create_program(
            boost::assign::list_of
            (device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_xyz_pass3_vs_source))
            (device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_xyz_pass3_fs_source)));

        if (!vis_xyz_pass3_shader_) {
            std::cout << "error creating vis_xyz_pass3_shader_ program" << std::endl;
            exit(1);
        }
        vis_xyz_lighting_shader_ = device_->create_program(
            boost::assign::list_of
            (device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_xyz_vs_lighting_source))
            (device_->create_shader(scm::gl::STAGE_GEOMETRY_SHADER, vis_xyz_gs_lighting_source))
            (device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_xyz_fs_lighting_source)));

        if (!vis_xyz_lighting_shader_) {
            std::cout << "error creating vis_xyz_lighting_shader_ program" << std::endl;
            exit(1);
        }
        vis_xyz_pass2_lighting_shader_ = device_->create_program(
            boost::assign::list_of
            (device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_xyz_pass2_vs_lighting_source))
            (device_->create_shader(scm::gl::STAGE_GEOMETRY_SHADER, vis_xyz_pass2_gs_lighting_source))
            (device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_xyz_pass2_fs_lighting_source)));

        if (!vis_xyz_pass2_lighting_shader_) {
            std::cout << "error creating vis_xyz_pass2_lighting_shader_ program" << std::endl;
            exit(1);
        }
        vis_xyz_pass3_lighting_shader_ = device_->create_program(
            boost::assign::list_of
            (device_->create_shader(scm::gl::STAGE_VERTEX_SHADER, vis_xyz_pass3_vs_lighting_source))
            (device_->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, vis_xyz_pass3_fs_lighting_source)));

        if (!vis_xyz_pass3_lighting_shader_) {
            std::cout << "error creating vis_xyz_pass3_lighting_shader_ program" << std::endl;
            exit(1);
        }
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
}

void LamurePointCloudPlugin::create_framebuffers() {

    fbo_.reset();
    fbo_color_buffer_.reset();
    fbo_depth_buffer_.reset();
    pass1_fbo_.reset();
    pass1_depth_buffer_.reset();
    pass2_fbo_.reset();
    pass2_color_buffer_.reset();
    pass2_normal_buffer_.reset();
    pass2_view_space_pos_buffer_.reset();

    fbo_ = device_->create_frame_buffer();
    fbo_color_buffer_ = device_->create_texture_2d(scm::math::vec2ui(render_width_, render_height_), scm::gl::FORMAT_RGBA_32F, 1, 1, 1);
    fbo_depth_buffer_ = device_->create_texture_2d(scm::math::vec2ui(render_width_, render_height_), scm::gl::FORMAT_D24, 1, 1, 1);
    fbo_->attach_color_buffer(0, fbo_color_buffer_);
    fbo_->attach_depth_stencil_buffer(fbo_depth_buffer_);

    pass1_fbo_ = device_->create_frame_buffer();
    pass1_depth_buffer_ = device_->create_texture_2d(scm::math::vec2ui(render_width_, render_height_), scm::gl::FORMAT_D24, 1, 1, 1);
    pass1_fbo_->attach_depth_stencil_buffer(pass1_depth_buffer_);

    pass2_fbo_ = device_->create_frame_buffer();
    pass2_color_buffer_ = device_->create_texture_2d(scm::math::vec2ui(render_width_, render_height_), scm::gl::FORMAT_RGBA_32F, 1, 1, 1);
    pass2_fbo_->attach_color_buffer(0, pass2_color_buffer_);
    pass2_fbo_->attach_depth_stencil_buffer(pass1_depth_buffer_);

    pass2_normal_buffer_ = device_->create_texture_2d(scm::math::vec2ui(render_width_, render_height_), scm::gl::FORMAT_RGB_32F, 1, 1, 1);
    pass2_fbo_->attach_color_buffer(1, pass2_normal_buffer_);
    pass2_view_space_pos_buffer_ = device_->create_texture_2d(scm::math::vec2ui(render_width_, render_height_), scm::gl::FORMAT_RGB_32F, 1, 1, 1);
    pass2_fbo_->attach_color_buffer(2, pass2_view_space_pos_buffer_);
}

void LamurePointCloudPlugin::init_render_states() {
    color_blending_state_ = device_->create_blend_state(true, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE,
        scm::gl::FUNC_ONE, scm::gl::EQ_FUNC_ADD, scm::gl::EQ_FUNC_ADD);
    color_no_blending_state_ = device_->create_blend_state(false);

    depth_state_less_ = device_->create_depth_stencil_state(true, true, scm::gl::COMPARISON_LESS);
    auto no_depth_test_descriptor = depth_state_less_->descriptor();
    no_depth_test_descriptor._depth_test = false;
    depth_state_disable_ = device_->create_depth_stencil_state(no_depth_test_descriptor);
    depth_state_without_writing_ = device_->create_depth_stencil_state(true, false, scm::gl::COMPARISON_LESS_EQUAL);

    no_backface_culling_rasterizer_state_ = device_->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false, false, 0.0, false, false);

    filter_linear_ = device_->create_sampler_state(scm::gl::FILTER_ANISOTROPIC, scm::gl::WRAP_CLAMP_TO_EDGE, 16u);
    filter_nearest_ = device_->create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);

    vt_filter_linear_ = device_->create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);
    vt_filter_nearest_ = device_->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);
}

void LamurePointCloudPlugin::set_uniforms(scm::gl::program_ptr shader) {
    const osg::GraphicsContext::Traits* traits = coVRConfig::instance()->windows[0].context->getTraits();
    shader->uniform("win_size", scm::math::vec2f(traits->width, traits->height));
    shader->uniform("near_plane", settings_.near_plane_);
    shader->uniform("far_plane", settings_.far_plane_);
    shader->uniform("point_size_factor", settings_.lod_point_scale_);
    shader->uniform("show_normals", (bool)settings_.show_normals_);
    shader->uniform("show_accuracy", (bool)settings_.show_accuracy_);
    shader->uniform("show_radius_deviation", (bool)settings_.show_radius_deviation_);
    shader->uniform("show_output_sensitivity", (bool)settings_.show_output_sensitivity_);
    shader->uniform("channel", settings_.channel_);
    shader->uniform("heatmap", (bool)settings_.heatmap_);
    shader->uniform("face_eye", false);
    shader->uniform("max_radius", settings_.max_radius_);
    shader->uniform("heatmap_min", settings_.heatmap_min_);
    shader->uniform("heatmap_max", settings_.heatmap_max_);
    shader->uniform("heatmap_min_color", settings_.heatmap_color_min_);
    shader->uniform("heatmap_max_color", settings_.heatmap_color_max_);
    if (settings_.enable_lighting_) {
        shader->uniform("use_material_color", settings_.use_material_color_);
        shader->uniform("material_diffuse", settings_.material_diffuse_);
        shader->uniform("material_specular", settings_.material_specular_);
        shader->uniform("ambient_light_color", settings_.ambient_light_color_);
        shader->uniform("point_light_color", settings_.point_light_color_);
    }
}


void LamurePointCloudPlugin::draw_resources(const lamure::context_t context_id, const lamure::view_t view_id) {
    const osg::GraphicsContext::Traits* traits = NULL;
    traits = coVRConfig::instance()->windows[0].context->getTraits();
    if (sparse_resources_.size() > 0) {
        if ((settings_.show_sparse_ || settings_.show_views_) && sparse_resources_.size() > 0) {

            context_->bind_program(vis_xyz_shader_);
            context_->set_blend_state(color_no_blending_state_);
            context_->set_depth_stencil_state(depth_state_less_);

            set_uniforms(vis_xyz_shader_);

            scm::math::mat4d model_matrix = scm::math::mat4d::identity();
            scm::math::mat4d projection_matrix = scm::math::mat4d(camera_->get_projection_matrix());
            scm::math::mat4d view_matrix = camera_->get_high_precision_view_matrix();
            scm::math::mat4d model_view_matrix = view_matrix * model_matrix;
            scm::math::mat4d model_view_projection_matrix = projection_matrix * model_view_matrix;

            vis_xyz_shader_->uniform("mvp_matrix", scm::math::mat4f(model_view_projection_matrix));
            vis_xyz_shader_->uniform("model_matrix", scm::math::mat4f(model_matrix));
            vis_xyz_shader_->uniform("model_view_matrix", scm::math::mat4f(model_view_matrix));
            vis_xyz_shader_->uniform("inv_mv_matrix", scm::math::mat4f(scm::math::transpose(scm::math::inverse(model_view_matrix))));

            vis_xyz_shader_->uniform("point_size_factor", settings_.aux_point_scale_);

            vis_xyz_shader_->uniform("model_to_screen_matrix", scm::math::mat4f::identity());
            vis_xyz_shader_->uniform("model_radius_scale", 1.f);

            scm::math::mat4f inv_view = scm::math::inverse(scm::math::mat4f(view_matrix));
            scm::math::vec3f eye = scm::math::vec3f(inv_view[12], inv_view[13], inv_view[14]);

            vis_xyz_shader_->uniform("eye", eye);
            vis_xyz_shader_->uniform("face_eye", true);

            vis_xyz_shader_->uniform("show_normals", false);
            vis_xyz_shader_->uniform("show_accuracy", false);
            vis_xyz_shader_->uniform("show_radius_deviation", false);
            vis_xyz_shader_->uniform("show_output_sensitivity", false);
            vis_xyz_shader_->uniform("channel", 0);

            for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
                if (selection_.selected_model_ != -1) {
                    model_id = selection_.selected_model_;
                }

                auto s_res = sparse_resources_[model_id];
                if (s_res.num_primitives_ > 0) {
                    context_->bind_vertex_array(s_res.array_);
                    //context_->apply();

                    uint32_t num_views = provenance_[model_id].num_views_;

                    if (settings_.show_views_) {
                        if (selection_.selected_views_.empty()) {
                            context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 0, num_views);
                        }
                        else {
                            for (const auto view : selection_.selected_views_) {
                                context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, view, 1);
                            }
                        }
                    }
                    if (settings_.show_sparse_) {
                        context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, num_views,
                            s_res.num_primitives_ - num_views);
                    }

                }

                if (selection_.selected_model_ != -1) {
                    break;
                }
            }

        }

        // draw image_plane resources with vt system
        if (settings_.show_photos_ && !settings_.atlas_file_.empty()) {
            context_->bind_program(vis_vt_shader_);

            uint64_t color_cut_id =
                (((uint64_t)vt_.texture_id_) << 32) | ((uint64_t)vt_.view_id_ << 16) | ((uint64_t)vt_.context_id_);
            //uint32_t max_depth_level_color = (*vt::CutDatabase::get_instance().get_cut_map())[color_cut_id]->get_atlas()->getDepth() - 1;

            scm::math::mat4f view_matrix = camera_->get_view_matrix();
            scm::math::mat4f projection_matrix = scm::math::mat4f(camera_->get_projection_matrix());

            vis_vt_shader_->uniform("model_view_matrix", view_matrix);
            vis_vt_shader_->uniform("projection_matrix", projection_matrix);

            vis_vt_shader_->uniform("physical_texture_dim", vt_.physical_texture_size_);
            //vis_vt_shader_->uniform("max_level", max_depth_level_color);
            vis_vt_shader_->uniform("tile_size", scm::math::vec2((uint32_t)vt::VTConfig::get_instance().get_size_tile()));
            vis_vt_shader_->uniform("tile_padding", scm::math::vec2((uint32_t)vt::VTConfig::get_instance().get_size_padding()));

            vis_vt_shader_->uniform("enable_hierarchy", vt_.enable_hierarchy_);
            vis_vt_shader_->uniform("toggle_visualization", vt_.toggle_visualization_);

            for (uint32_t i = 0; i < vt_.index_texture_hierarchy_.size(); ++i) {
                std::string texture_string = "hierarchical_idx_textures";
                vis_vt_shader_->uniform(texture_string, i, int((i)));
            }

            vis_vt_shader_->uniform("physical_texture_array", 17);

            //context_->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(render_width_, render_height_)));
            context_->set_depth_stencil_state(depth_state_less_);
            context_->set_rasterizer_state(no_backface_culling_rasterizer_state_);
            context_->set_blend_state(color_no_blending_state_);
            //context_->sync();

            //apply_vt_cut_update();

            for (uint16_t i = 0; i < vt_.index_texture_hierarchy_.size(); ++i) {
                context_->bind_texture(vt_.index_texture_hierarchy_.at(i), vt_filter_nearest_, i);
            }
            context_->bind_texture(vt_.physical_texture_, vt_filter_linear_, 17);
            context_->bind_storage_buffer(vt_.feedback_lod_storage_, 0);
            context_->bind_storage_buffer(vt_.feedback_count_storage_, 1);
            //context_->apply();

            for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
                if (selection_.selected_model_ != -1) {
                    model_id = selection_.selected_model_;
                }
                auto t_res = image_plane_resources_[model_id];
                if (t_res.num_primitives_ > 0) {
                    context_->bind_vertex_array(t_res.array_);
                    //context_->apply();
                    if (selection_.selected_views_.empty()) {
                        context_->draw_arrays(scm::gl::PRIMITIVE_TRIANGLE_LIST, 0, t_res.num_primitives_);
                    }
                    else {
                        for (const auto view : selection_.selected_views_) {
                            context_->draw_arrays(scm::gl::PRIMITIVE_TRIANGLE_LIST, view * 6, 6);
                        }
                    }
                }
                if (selection_.selected_model_ != -1) {
                    break;
                }
            }
            //context_->sync();
        }

        if (settings_.show_views_ || settings_.show_octrees_) {
            context_->bind_program(vis_line_shader_);

            scm::math::mat4f projection_matrix = scm::math::mat4f(camera_->get_projection_matrix());
            scm::math::mat4f view_matrix = camera_->get_view_matrix();
            vis_line_shader_->uniform("model_matrix", scm::math::mat4f::identity());
            vis_line_shader_->uniform("view_matrix", view_matrix);
            vis_line_shader_->uniform("projection_matrix", projection_matrix);

            for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
                if (selection_.selected_model_ != -1) {
                    model_id = selection_.selected_model_;
                }

                if (settings_.show_views_) {
                    uint32_t num_views = provenance_[model_id].num_views_;
                    auto f_res = frusta_resources_[model_id];
                    if (f_res.num_primitives_ > 0) {
                        context_->bind_vertex_array(f_res.array_);
                        //context_->apply();
                        if (selection_.selected_views_.empty()) {
                            context_->draw_arrays(scm::gl::PRIMITIVE_LINE_LIST, 0, f_res.num_primitives_);
                        }
                        else {
                            for (const auto view : selection_.selected_views_) {
                                context_->draw_arrays(scm::gl::PRIMITIVE_LINE_LIST, view * 16, 16);
                            }
                        }
                    }
                }

                if (settings_.show_octrees_) {
                    auto o_res = octree_resources_[model_id];
                    if (o_res.num_primitives_ > 0) {
                        context_->bind_vertex_array(o_res.array_);
                        //context_->apply();
                        context_->draw_arrays(scm::gl::PRIMITIVE_LINE_LIST, 0, o_res.num_primitives_);
                    }
                }
                if (selection_.selected_model_ != -1) {
                    break;
                }
            }
        }
    }

    if (settings_.show_bvhs_) {

        lamure::ren::controller* controller = lamure::ren::controller::get_instance();
        lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
        lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
        //lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();

        context_->bind_program(vis_line_shader_);

        scm::math::mat4f projection_matrix = scm::math::mat4f(camera_->get_projection_matrix());
        scm::math::mat4f view_matrix = camera_->get_view_matrix();

        vis_line_shader_->uniform("view_matrix", view_matrix);
        vis_line_shader_->uniform("projection_matrix", projection_matrix);

        for (int32_t model_id = 0; model_id < num_models_; ++model_id) {
            if (selection_.selected_model_ != -1) {
                model_id = selection_.selected_model_;
            }

            bool draw = true;
            lamure::model_t m_id = controller->deduce_model_id(std::to_string(model_id));
            lamure::ren::cut& cut = cuts->get_cut(context_id, view_id, m_id);
            std::vector<lamure::ren::cut::node_slot_aggregate> renderable = cut.complete_set();
            const lamure::ren::bvh* bvh = database->get_model(m_id)->get_bvh();
            if (bvh->get_primitive() != lamure::ren::bvh::primitive_type::POINTCLOUD) {
                if (selection_.selected_model_ != -1) break;
                else draw = false;
            }

            if (draw) {
                //uniforms per model
                scm::math::mat4d model_matrix = model_transformations_[model_id];
                vis_line_shader_->uniform("model_matrix", scm::math::mat4f(model_matrix));
                std::vector<scm::gl::boxf>const& bounding_box_vector = bvh->get_bounding_boxes();
                scm::gl::frustum frustum_by_model = camera_->get_frustum_by_model(scm::math::mat4f(model_matrix));
                auto bvh_res = bvh_resources_[model_id];
                if (bvh_res.num_primitives_ > 0) {
                    context_->bind_vertex_array(bvh_res.array_);
                    //context_->apply();

                    for (auto const& node_slot_aggregate : renderable) {
                        uint32_t node_culling_result = camera_->cull_against_frustum(
                            frustum_by_model,
                            bounding_box_vector[node_slot_aggregate.node_id_]);

                        if (node_culling_result != 1) {

                            /*if (settings_.use_pvs_ && pvs->is_activated() && settings_.pvs_culling_ && !lamure::pvs::pvs_database::get_instance()->get_viewer_visibility(model_id, node_slot_aggregate.node_id_)) {
                                continue;
                            }*/

                            context_->draw_arrays(scm::gl::PRIMITIVE_LINE_LIST, node_slot_aggregate.node_id_ * 24, 24);
                        }
                    }
                }
            }
            if (selection_.selected_model_ != -1) {
                break;
            }
        }
    }

    if (settings_.pvs_ != "" && settings_.show_pvs_) {
        if (pvs_resource_.num_primitives_ > 0) {
            context_->bind_program(vis_line_shader_);

            scm::math::mat4f projection_matrix = scm::math::mat4f(camera_->get_projection_matrix());
            scm::math::mat4f view_matrix = camera_->get_view_matrix();
            vis_line_shader_->uniform("model_matrix", scm::math::mat4f::identity());
            vis_line_shader_->uniform("view_matrix", view_matrix);
            vis_line_shader_->uniform("projection_matrix", projection_matrix);

            context_->bind_vertex_array(pvs_resource_.array_);
            //context_->apply();
            context_->draw_arrays(scm::gl::PRIMITIVE_LINE_LIST, 0, pvs_resource_.num_primitives_);
        }
    }
}

void LamurePointCloudPlugin::draw_brush(scm::gl::program_ptr shader) {

    if (selection_.brush_end_ > 0) {
        set_uniforms(shader);

        scm::math::mat4d model_matrix = scm::math::mat4d::identity();
        scm::math::mat4d projection_matrix = scm::math::mat4d(camera_->get_projection_matrix());
        scm::math::mat4d view_matrix = camera_->get_high_precision_view_matrix();
        scm::math::mat4d model_view_matrix = view_matrix * model_matrix;
        scm::math::mat4d model_view_projection_matrix = projection_matrix * model_view_matrix;

        shader->uniform("mvp_matrix", scm::math::mat4f(model_view_projection_matrix));
        shader->uniform("model_matrix", scm::math::mat4f(model_matrix));
        shader->uniform("model_view_matrix", scm::math::mat4f(model_view_matrix));
        shader->uniform("inv_mv_matrix", scm::math::mat4f(scm::math::transpose(scm::math::inverse(model_view_matrix))));

        shader->uniform("point_size_factor", settings_.aux_point_scale_);

        shader->uniform("model_to_screen_matrix", scm::math::mat4f::identity());
        shader->uniform("model_radius_scale", 1.f);

        shader->uniform("show_normals", false);
        shader->uniform("show_accuracy", false);
        shader->uniform("show_radius_deviation", false);
        shader->uniform("show_output_sensitivity", false);
        shader->uniform("channel", 0);

        shader->uniform("face_eye", false);

        context_->bind_vertex_array(brush_resource_.array_);
        //context_->apply();
        context_->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 0, selection_.brush_end_);

    }
}


std::string const LamurePointCloudPlugin::strip_whitespace(std::string const& in_string) {
    return boost::regex_replace(in_string, boost::regex("^ +| +$|( ) +"), "$1");
}

scm::gl::data_format get_tex_format() {
    switch (vt::VTConfig::get_instance().get_format_texture()) {
    case vt::VTConfig::R8:
        return scm::gl::FORMAT_R_8;
    case vt::VTConfig::RGB8:
        return scm::gl::FORMAT_RGB_8;
    case vt::VTConfig::RGBA8:
    default:
        return scm::gl::FORMAT_RGBA_8;
    }
}

string LamurePointCloudPlugin::getConfigEntry(string scope) {
    std::cout << "getConfigEntry(scope): ";
    coCoviseConfig::ScopeEntries entries = coCoviseConfig::getScopeEntries(scope);
    for (const auto& entry : entries)
    {
        return entry.second;
    }
    return "";
}
string LamurePointCloudPlugin::getConfigEntry(string scope, string name) {
    std::cout << "getConfigEntry(scope, name): ";
    coCoviseConfig::ScopeEntries entries = coCoviseConfig::getScopeEntries(scope);
    for (const auto& entry : entries) {
        std::cout << entry.first << " " << entry.second << " ";
        if (name == entry.first)
        {
            return entry.second;
        }
    }
    return "";
}
const char* LamurePointCloudPlugin::stringToConstChar(string str) {
    const char* cstr = str.c_str();
    return cstr;
}

LamurePointCloudPlugin* LamurePointCloudPlugin::instance()
{
    return plugin;
}


static void GLClearError()
{
    while (glGetError() != GL_NO_ERROR);
}


static bool GLLogError()
{
    while (GLenum error = glGetError())
    {
        std::cout << "[OpenGL Error] (" << error << ")" << std::endl;
    }
    return true;
}


void LamurePointCloudPlugin::load_settings(std::string const& filename) {
    std::cout << "load_settings()" << std::endl;
    std::cout << filename << std::endl;
    std::ifstream lmr_file(filename.c_str());
    if (!lmr_file.is_open()) {
        std::cout << "could not open lmr file" << std::endl;
        exit(-1);
    }
    else {
        lamure::model_t model_id = 0;
        std::string line;
        while (std::getline(lmr_file, line)) {
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

                    settings_.models_.push_back(model);
                    settings_.transforms_[model_id] = scm::math::mat4d::identity();
                    settings_.aux_[model_id] = "";
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
                            settings_.transforms_[address] = load_matrix(value);
                            std::cout << "found transform for model id " << address << std::endl;
                        }
                        else if (key == "aux") {
                            settings_.aux_[address] = value;
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
                        settings_.width_ = std::max(atoi(value.c_str()), 64);
                    }
                    else if (key == "height") {
                        settings_.height_ = std::max(atoi(value.c_str()), 64);
                    }
                    else if (key == "frame_div") {
                        settings_.frame_div_ = std::max(atoi(value.c_str()), 1);
                    }
                    else if (key == "vram") {
                        settings_.vram_ = std::max(atoi(value.c_str()), 8);
                    }
                    else if (key == "ram") {
                        settings_.ram_ = std::max(atoi(value.c_str()), 8);
                    }
                    else if (key == "upload") {
                        settings_.upload_ = std::max(atoi(value.c_str()), 8);
                    }
                    else if (key == "near") {
                        settings_.near_plane_ = std::max(atof(value.c_str()), 0.0);
                    }
                    else if (key == "far") {
                        settings_.far_plane_ = std::max(atof(value.c_str()), 0.1);
                    }
                    else if (key == "fov") {
                        settings_.fov_ = std::max(atof(value.c_str()), 9.0);
                    }
                    else if (key == "splatting") {
                        settings_.splatting_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "gamma_correction") {
                        settings_.gamma_correction_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "gui") {
                        settings_.gui_ = std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "speed") {
                        settings_.travel_speed_ = std::min(std::max(atof(value.c_str()), 0.0), 400.0);
                    }
                    else if (key == "pvs_culling") {
                        settings_.pvs_culling_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "use_pvs") {
                        settings_.use_pvs_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "lod_point_scale") {
                        settings_.lod_point_scale_ = std::min(std::max(atof(value.c_str()), 0.0), 10.0);
                    }
                    else if (key == "aux_point_size") {
                        settings_.aux_point_size_ = std::min(std::max(atof(value.c_str()), 0.00001), 1.0);
                    }
                    else if (key == "aux_point_distance") {
                        settings_.aux_point_distance_ = std::min(std::max(atof(value.c_str()), 0.00001), 1.0);
                    }
                    else if (key == "aux_focal_length") {
                        settings_.aux_focal_length_ = std::min(std::max(atof(value.c_str()), 0.001), 10.0);
                    }
                    else if (key == "max_brush_size") {
                        settings_.max_brush_size_ = std::min(std::max(atoi(value.c_str()), 64), 1024 * 1024);
                    }
                    else if (key == "lod_error") {
                        settings_.lod_error_ = std::min(std::max(atof(value.c_str()), 0.0), 10.0);
                    }
                    else if (key == "provenance") {
                        settings_.provenance_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "create_aux_resources") {
                        settings_.create_aux_resources_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_normals") {
                        settings_.show_normals_ = std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_accuracy") {
                        settings_.show_accuracy_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_radius_deviation") {
                        settings_.show_radius_deviation_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_output_sensitivity") {
                        settings_.show_output_sensitivity_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_sparse") {
                        settings_.show_sparse_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_views") {
                        settings_.show_views_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_photos") {
                        settings_.show_photos_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_octrees") {
                        settings_.show_octrees_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_bvhs") {
                        settings_.show_bvhs_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "show_pvs") {
                        settings_.show_pvs_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "channel") {
                        settings_.channel_ = std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "enable_lighting") {
                        settings_.enable_lighting_ = (bool)std::min(std::max(atoi(value.c_str()), 0), 1);
                    }
                    else if (key == "use_material_color") {
                        settings_.use_material_color_ = (bool)std::min(std::max(atoi(value.c_str()), 0), 1);
                    }
                    else if (key == "material_diffuse_r") {
                        settings_.material_diffuse_.x = std::max(atof(value.c_str()), 0.0);
                    }
                    else if (key == "material_diffuse_g") {
                        settings_.material_diffuse_.y = std::max(atof(value.c_str()), 0.0);
                    }
                    else if (key == "material_diffuse_b") {
                        settings_.material_diffuse_.z = std::max(atof(value.c_str()), 0.0);
                    }
                    else if (key == "material_specular_r") {
                        settings_.material_specular_.x = std::max(atof(value.c_str()), 0.0);
                    }
                    else if (key == "material_specular_g") {
                        settings_.material_specular_.y = std::max(atof(value.c_str()), 0.0);
                    }
                    else if (key == "material_specular_b") {
                        settings_.material_specular_.z = std::max(atof(value.c_str()), 0.0);
                    }
                    else if (key == "material_specular_exponent") {
                        settings_.material_specular_.w = std::min(std::max(atof(value.c_str()), 0.0), 10000.0);
                    }
                    else if (key == "ambient_light_color_r") {
                        settings_.ambient_light_color_.r = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
                    }
                    else if (key == "ambient_light_color_g") {
                        settings_.ambient_light_color_.g = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
                    }
                    else if (key == "ambient_light_color_b") {
                        settings_.ambient_light_color_.b = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
                    }
                    else if (key == "point_light_color_r") {
                        settings_.point_light_color_.r = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
                    }
                    else if (key == "point_light_color_g") {
                        settings_.point_light_color_.g = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
                    }
                    else if (key == "point_light_color_b") {
                        settings_.point_light_color_.b = std::min(std::max(atof(value.c_str()), 0.0), 1.0);
                    }
                    else if (key == "point_light_intensity") {
                        settings_.point_light_color_.w = std::min(std::max(atof(value.c_str()), 0.0), 10000.0);
                    }
                    else if (key == "background_color_r") {
                        //settings_.background_color_.x = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
                        settings_.background_color_.x = 0.1f;
                    }
                    else if (key == "background_color_g") {
                        //settings_.background_color_.y = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
                        settings_.background_color_.y = 0.1f;
                    }
                    else if (key == "background_color_b") {
                        //settings_.background_color_.z = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
                        settings_.background_color_.z = 0.1f;
                    }
                    else if (key == "heatmap") {
                        settings_.heatmap_ = (bool)std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "heatmap_min") {
                        settings_.heatmap_min_ = std::max(atof(value.c_str()), 0.0);
                    }
                    else if (key == "heatmap_max") {
                        settings_.heatmap_max_ = std::max(atof(value.c_str()), 0.0);
                    }
                    else if (key == "heatmap_min_r") {
                        settings_.heatmap_color_min_.x = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
                    }
                    else if (key == "heatmap_min_g") {
                        settings_.heatmap_color_min_.y = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
                    }
                    else if (key == "heatmap_min_b") {
                        settings_.heatmap_color_min_.z = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
                    }
                    else if (key == "heatmap_max_r") {
                        settings_.heatmap_color_max_.x = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
                    }
                    else if (key == "heatmap_max_g") {
                        settings_.heatmap_color_max_.y = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
                    }
                    else if (key == "heatmap_max_b") {
                        settings_.heatmap_color_max_.z = std::min(std::max(atoi(value.c_str()), 0), 255) / 255.f;
                    }
                    else if (key == "atlas_file") {
                        settings_.atlas_file_ = value;
                    }
                    else if (key == "json") {
                        settings_.json_ = value;
                    }
                    else if (key == "pvs") {
                        settings_.pvs_ = value;
                    }
                    else if (key == "selection") {
                        settings_.selection_ = value;
                    }
                    else if (key == "background_image") {
                        settings_.background_image_ = value;
                    }
                    else if (key == "use_view_tf") {
                        settings_.use_view_tf_ = std::max(atoi(value.c_str()), 0);
                    }
                    else if (key == "view_tf") {
                        settings_.view_tf_ = load_matrix(value);
                    }
                    else if (key == "max_radius") {
                        settings_.max_radius_ = std::max(atof(value.c_str()), 0.0);
                    }
                    else {
                        std::cout << "unrecognized key: " << key << std::endl;
                        exit(-1);
                    }

                    std::cout << key << " : " << value << std::endl;
                }
            }
        }
        lmr_file.close();
    }
    //assertions
    if (settings_.provenance_ != 0) {
        if (settings_.json_.size() > 0) {
            if (settings_.json_.substr(settings_.json_.size() - 5) != ".json") {
                std::cout << "unsupported json file" << std::endl;
                exit(-1);
            }
        }
    }
    if (settings_.models_.empty()) {
        std::cout << "error: no model filename specified" << std::endl;
        exit(-1);
    }
    if (settings_.pvs_.size() > 0) {
        if (settings_.pvs_.substr(settings_.pvs_.size() - 4) != ".pvs") {
            std::cout << "unsupported pvs file" << std::endl;
            exit(-1);
        }
    }
}


scm::math::mat4d LamurePointCloudPlugin::load_matrix(const std::string& filename) {
    std::ifstream file(filename.c_str());
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



bool LamurePointCloudPlugin::parse_prefix(std::string& in_string, std::string const& prefix) {
    uint32_t num_prefix_characters = prefix.size();
    bool prefix_found
        = (!(in_string.size() < num_prefix_characters)
            && strncmp(in_string.c_str(), prefix.c_str(), num_prefix_characters) == 0);
    if (prefix_found) {
        in_string = in_string.substr(num_prefix_characters);
        in_string = strip_whitespace(in_string);
    }
    return prefix_found;
}

void LamurePointCloudPlugin::readMenuConfigData(const char* menu, vector<ImageFileEntry>& menulist, ui::Group* subMenu)
{
    coCoviseConfig::ScopeEntries entries = coCoviseConfig::getScopeEntries(menu);
    for (const auto& entry : entries)
    {
        ui::Button* temp = new ui::Button(subMenu, entry.second);
        temp->setCallback([this, entry](bool state)
            {
                if (state)
                std::printf("createGeodes(planetTrans, entry.second)");
            });
        menulist.push_back(ImageFileEntry(entry.first.c_str(), entry.second.c_str(), (ui::Element*)temp));
    }
}

void LamurePointCloudPlugin::selectedMenuButton(ui::Element* menuItem)
{
    std::string filename;

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


int LamurePointCloudPlugin::unloadLMR(const char* filename, const char* covise_key)
{
    return 1;
}


LamurePointCloudPlugin::~LamurePointCloudPlugin()
{
    fprintf(stderr, "LamurePlugin::~LamurePlugin\n");
    coVRFileManager::instance()->unregisterFileHandler(&handler);
    cover->getObjectsRoot()->removeChild(LamureGroup);
}


void debug_callback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* param) {
    switch (severity) {
    case GL_DEBUG_SEVERITY_HIGH: {
        fprintf(stderr, "GL_DEBUG_SEVERITY_HIGH: %s type = 0x%x, message = %s\n", (type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : ""), type, message);
    }
                               break;
    case GL_DEBUG_SEVERITY_MEDIUM: {
        fprintf(stderr, "GL_DEBUG_SEVERITY_MEDIUM: %s type = 0x%x, message = %s\n", (type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : ""), type, message);
    }
                                 break;
    case GL_DEBUG_SEVERITY_LOW: {
        fprintf(stderr, "GL_DEBUG_SEVERITY_LOW: %s type = 0x%x, message = %s\n", (type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : ""), type, message);
    }
                              break;
    default:
        break;
    }
}

