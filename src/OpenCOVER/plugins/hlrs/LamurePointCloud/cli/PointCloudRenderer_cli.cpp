#include "../include/PointCloudRenderer_cli.h"
#include "../../LamurePointCloud_cli.h"

#include <lamure/ren/model_database.h>
#include <lamure/ren/cut_database.h>
#include <lamure/ren/controller.h>
#include <lamure/ren/bvh.h>
#include <cover/VRViewer.h>
#include <iostream>
#include <fstream>
#include <sstream>

PointCloudRenderer::PointCloudRenderer(LamurePointCloud_cli* plugin)
    : plugin_(plugin)
    , traits_(opencover::coVRConfig::instance()->windows[0].context->getTraits())
{}

PointCloudRenderer::~PointCloudRenderer() {
    // Release OpenGL resources if needed
}

void PointCloudRenderer::initShaders()
{
    initPointShader();
    initSurfelShader();
    initLineShader();
}

void PointCloudRenderer::initResources()
{
    // Bounding Box Resources
    glGenVertexArrays(1, &box_resource_.vao_);
    glBindVertexArray(box_resource_.vao_);

    glGenBuffers(1, &box_resource_.vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, box_resource_.vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 24, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glGenBuffers(1, &box_resource_.ibo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, box_resource_.ibo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
        sizeof(unsigned short) * box_resource_.idx_.size(),
        box_resource_.idx_.data(),
        GL_STATIC_DRAW);

    // Frustum Resources
    glGenVertexArrays(1, &frustum_resource_.vao_);
    glBindVertexArray(frustum_resource_.vao_);

    glGenBuffers(1, &frustum_resource_.vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, frustum_resource_.vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 24, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glGenBuffers(1, &frustum_resource_.ibo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, frustum_resource_.ibo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
        sizeof(unsigned short) * frustum_resource_.idx_.size(),
        frustum_resource_.idx_.data(),
        GL_STATIC_DRAW);

    // Coord Resources
    glGenVertexArrays(1, &coord_resource_.vao_);
    glBindVertexArray(coord_resource_.vao_);

    glGenBuffers(1, &coord_resource_.vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, coord_resource_.vbo_);
    glBufferData(GL_ARRAY_BUFFER,
        sizeof(float) * coord_resource_.vertices_.size(),
        coord_resource_.vertices_.data(),
        GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glGenBuffers(1, &coord_resource_.ibo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, coord_resource_.ibo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
        sizeof(unsigned short) * coord_resource_.idx_.size(),
        coord_resource_.idx_.data(),
        GL_STATIC_DRAW);

    glBindVertexArray(0);
}

void PointCloudRenderer::drawPointCloud(osg::RenderInfo& renderInfo) const
{
    GLState stateSaver;
    glDisable(GL_CULL_FACE);

    osg::State* state = renderInfo.getState();
    state->setCheckForGLErrors(osg::State::CheckForGLErrors::ONCE_PER_ATTRIBUTE);

    scm::math::mat4 viewMatrix     = matConv4F(state->getModelViewMatrix());
    scm::math::mat4 projectionMatrix = matConv4F(state->getProjectionMatrix());
    scm::math::mat4 scaleMatrix    = matConv4F(opencover::cover->getObjectsScale()->getMatrix());

    auto* database = lamure::ren::model_database::get_instance();
    auto* cuts     = lamure::ren::cut_database::get_instance();
    auto* controller = lamure::ren::controller::get_instance();
    auto* pvs       = lamure::pvs::pvs_database::get_instance();

    if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
        controller->reset_system(data_provenance_);
    } else {
        controller->reset_system();
    }

    lamure::context_t contextId = controller->deduce_context_id(lmr_ctx_);
    lamure::view_t    viewId    = controller->deduce_view_id(contextId, scm_camera_->view_id());
    size_t surfelsPerNode       = database->get_primitives_per_node();

    for (lamure::model_t id = 0; id < plugin_->getNumModels(); ++id) {
        lamure::model_t modelId = controller->deduce_model_id(std::to_string(id));
        cuts->send_transform(contextId, modelId,
            scm::math::mat4(plugin_->getModelTransforms().at(modelId)));
        cuts->send_threshold(contextId, modelId, plugin_->getSettings().lod_error_);
        cuts->send_rendered(contextId, modelId);
        database->get_model(modelId)->set_transform(
            scm::math::mat4(plugin_->getModelTransforms().at(modelId)));
    }

    cuts->send_camera(contextId, viewId, *scm_camera_);
    auto corners = scm_camera_->get_frustum_corners();
    double delta = scm::math::length(corners[2] - corners[0]);
    height_divided_by_top_minus_bottom_ = traits_->height / delta;
    cuts->send_height_divided_by_top_minus_bottom(contextId, viewId,
        height_divided_by_top_minus_bottom_);

    if (plugin_->getSettings().use_pvs_) {
        auto camPos = scm_camera_->get_cam_pos();
        pvs->set_viewer_position(camPos);
    }

    if (plugin_->getSettings().lod_update_) {
        if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
            controller->dispatch(contextId, device_, data_provenance_);
        } else {
            controller->dispatch(contextId, device_);
        }
    }

    if (pointcloud_vao_) {
        glBindVertexArray(pointcloud_vao_);
    }
    context_->apply_vertex_input();
    if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
        context_->bind_vertex_array(
            controller->get_context_memory(contextId,
                lamure::ren::bvh::primitive_type::POINTCLOUD,
                device_,
                data_provenance_));
    } else {
        context_->bind_vertex_array(
            controller->get_context_memory(contextId,
                lamure::ren::bvh::primitive_type::POINTCLOUD,
                device_));
    }

    scm::math::mat4d vpScale     = scm::math::make_scale(traits_->width * 0.5,
        traits_->height * 0.5, 0.5);
    scm::math::mat4d vpTranslate = scm::math::make_translation(1.0, 1.0, 1.0);
    scm::math::mat4 modelToScreen = scm::math::mat4(vpScale * vpTranslate);
    scm::math::vec3 eyePos        = scm::math::vec3f(scm_camera_->get_cam_pos());
    scm::math::vec2 viewportSize  = scm::math::vec2f(traits_->width, traits_->height);

    if (!plugin_->getSettings().surfel_shader_ && !plugin_->getSettings().provenance_) {
        glUseProgram(point_shader_program_);
        glEnable(GL_POINT_SMOOTH);
        glEnable(GL_PROGRAM_POINT_SIZE);
        glUniform1f(point_uniforms_.max_radius_, plugin_->getSettings().max_radius_);
        glUniform1f(point_uniforms_.scale_radius_, plugin_->getSettings().scale_radius_);
        glUniform1f(point_uniforms_.point_size_factor_,
            plugin_->getSettings().point_size_factor_ * opencover::cover->getScale());
        glUniform1f(point_uniforms_.proj_scale_,
            viewportSize.y * 0.5f * projectionMatrix.data_array[5]);
    } else if (plugin_->getSettings().surfel_shader_ && !plugin_->getSettings().provenance_) {
        glUseProgram(surfel_shader_program_);
        glUniform1f(surfel_uniforms_.max_radius_, plugin_->getSettings().max_radius_);
        glUniform1f(surfel_uniforms_.scale_radius_, plugin_->getSettings().scale_radius_);
        glUniform1f(surfel_uniforms_.surfel_size_factor_,
            plugin_->getSettings().surfel_size_factor_);
        glUniform1f(surfel_uniforms_.proj_scale_,
            viewportSize.y * 0.5f * projectionMatrix.data_array[5]);
        glUniform2f(surfel_uniforms_.viewport_, viewportSize.x, viewportSize.y);
    }

    uint64_t splatCount = 0;
    uint64_t nodeCount  = 0;

    for (uint16_t id = 0; id < plugin_->getNumModels(); ++id) {
        if (!plugin_->getModelVisibility()[id]) continue;
        auto& cut = cuts->get_cut(contextId, lmr_ctx_, id);
        auto renderSet = cut.complete_set();
        const auto* bvh = database->get_model(id)->get_bvh();
        const auto& bb   = bvh->get_bounding_boxes();

        scm::math::mat4 modelMatrix   = scm::math::mat4(plugin_->getModelTransforms().at(id));
        scm::math::mat4 mvMatrix      = viewMatrix * modelMatrix;
        scm::math::mat4 mvpMatrix     = projectionMatrix * mvMatrix;
        scm::gl::frustum frustumModel = scm_camera_->get_frustum_by_model(modelMatrix);

        if (!plugin_->getSettings().surfel_shader_ && !plugin_->getSettings().provenance_) {
            glUniformMatrix4fv(point_uniforms_.mvp_matrix_, 1, GL_FALSE, mvpMatrix.data_array);
        } else {
            glUniformMatrix4fv(surfel_uniforms_.model_view_matrix_, 1, GL_FALSE, mvMatrix.data_array);
            glUniformMatrix4fv(surfel_uniforms_.mvp_matrix_,        1, GL_FALSE, mvpMatrix.data_array);
        }

        for (auto const& slot : renderSet) {
            if (scm_camera_->cull_against_frustum(frustumModel, bb[slot.node_id_]) != 1) {
                glDrawArrays(scm::gl::PRIMITIVE_POINT_LIST,
                    slot.slot_id_ * (GLsizei)surfelsPerNode,
                    surfelsPerNode);
                splatCount += surfelsPerNode;
                ++nodeCount;
            }
        }
    }

    if (plugin_->dump_button_->state()) {
        plugin_->debugPrintSettings();
        plugin_->dump_button_->setState(false);
    }

    render_info_.rendered_splats_   = splatCount;
    render_info_.rendered_nodes_    = nodeCount;

    stateSaver.restore();
}

… // (Similarly rename drawBoundingBoxes, drawFrustum, drawCoords and shader init calls)

bool PointCloudRenderer::readShader(const std::string& path, std::string& source) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Failed to open shader file: " << path << std::endl;
        return false;
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    source = buffer.str();
    return true;
}

GLuint PointCloudRenderer::compileShader(GLenum type, const std::string& source) const {
    GLuint shader = glCreateShader(type);
    const char* src = source.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        GLchar infoLog[512];
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        std::cerr << "Shader compilation error: " << infoLog << std::endl;
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

GLuint PointCloudRenderer::linkProgram(const std::vector<GLuint>& shaders) const {
    GLuint program = glCreateProgram();
    for (GLuint shader : shaders) {
        glAttachShader(program, shader);
    }
    glLinkProgram(program);

    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        GLchar infoLog[512];
        glGetProgramInfoLog(program, 512, nullptr, infoLog);
        std::cerr << "Shader linking error: " << infoLog << std::endl;
        glDeleteProgram(program);
        return 0;
    }

    for (GLuint shader : shaders) {
        glDetachShader(program, shader);
        glDeleteShader(shader);
    }
    return program;
}

void PointCloudRenderer::initPointShader() {
    std::string vsSrc, fsSrc;
    if (!readShader(shader_root_path_ + "point_vs.glsl", vsSrc) ||
        !readShader(shader_root_path_ + "point_fs.glsl", fsSrc)) {
        return;
    }
    GLuint vs = compileShader(GL_VERTEX_SHADER, vsSrc);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fsSrc);
    if (!vs || !fs) return;

    point_shader_program_ = linkProgram({vs, fs});
    point_uniforms_.mvp_matrix_    = glGetUniformLocation(point_shader_program_, "mvp_matrix");
    point_uniforms_.max_radius_    = glGetUniformLocation(point_shader_program_, "max_radius");
    point_uniforms_.scale_radius_  = glGetUniformLocation(point_shader_program_, "scale_radius");
    point_uniforms_.point_size_factor_ = glGetUniformLocation(point_shader_program_, "point_size_factor");
    point_uniforms_.proj_scale_    = glGetUniformLocation(point_shader_program_, "proj_scale");
}

void PointCloudRenderer::initSurfelShader() {
    std::string vsSrc, gsSrc, fsSrc;
    if (!readShader(shader_root_path_ + "surfel_vs.glsl", vsSrc) ||
        !readShader(shader_root_path_ + "surfel_gs.glsl", gsSrc) ||
        !readShader(shader_root_path_ + "surfel_fs.glsl", fsSrc)) {
        return;
    }
    GLuint vs = compileShader(GL_VERTEX_SHADER, vsSrc);
    GLuint gs = compileShader(GL_GEOMETRY_SHADER, gsSrc);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fsSrc);
    if (!vs || !gs || !fs) return;

    surfel_shader_program_ = linkProgram({vs, gs, fs});
    surfel_uniforms_.mvp_matrix_         = glGetUniformLocation(surfel_shader_program_, "mvp_matrix");
    surfel_uniforms_.model_view_matrix_ = glGetUniformLocation(surfel_shader_program_, "model_view_matrix");
    surfel_uniforms_.max_radius_         = glGetUniformLocation(surfel_shader_program_, "max_radius");
    surfel_uniforms_.scale_radius_       = glGetUniformLocation(surfel_shader_program_, "scale_radius");
    surfel_uniforms_.surfel_size_factor_ = glGetUniformLocation(surfel_shader_program_, "surfel_size_factor");
    surfel_uniforms_.proj_scale_         = glGetUniformLocation(surfel_shader_program_, "proj_scale");
    surfel_uniforms_.viewport_           = glGetUniformLocation(surfel_shader_program_, "viewport");
}

void PointCloudRenderer::initLineShader() {
    std::string vsSrc, fsSrc;
    if (!readShader(shader_root_path_ + "line_vs.glsl", vsSrc) ||
        !readShader(shader_root_path_ + "line_fs.glsl", fsSrc)) {
        return;
    }
    GLuint vs = compileShader(GL_VERTEX_SHADER, vsSrc);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fsSrc);
    if (!vs || !fs) return;

    line_shader_program_ = linkProgram({vs, fs});
    line_uniforms_.mvp_matrix_ = glGetUniformLocation(line_shader_program_, "mvp_matrix");
    line_uniforms_.in_color_   = glGetUniformLocation(line_shader_program_, "in_color");
}
