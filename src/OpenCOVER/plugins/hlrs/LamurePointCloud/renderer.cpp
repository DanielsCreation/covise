#include "renderer.h"
#include "Lamure.h"
#include "util.h"

#include <cover/coVRConfig.h>
#include <cover/VRViewer.h>
#include <cover/VRSceneGraph.h>
#include <cover/coVRFileManager.h>

#include <osg/Version>
#include <osg/Geometry>
#include <osg/Vec3>
#include <osg/Vec3ui>
#include <osg/BufferObject>
#include <osg/Point>
#include <osg/PointSprite>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/LineStipple>
#include <osg/BufferTemplate>
#include <osg/State>

#include <scm/gl_util/primitives/quad.h>
#include <scm/gl_util/font/font_face.h>
#include <scm/gl_util/font/text.h>
#include <scm/gl_util/font/text_renderer.h>

#include <lamure/ren/camera.h>
#include <lamure/lmr_camera.h>
#include <lamure/ren/trackball.h>

#include <lamure/ren/model_database.h>
#include <lamure/ren/cut_database.h>
#include <lamure/ren/controller.h>
#include <lamure/pvs/pvs_database.h>

#include <iostream>
#include <gl_state.h>
#include <config/CoviseConfig.h>

std::string shader_root_path = LAMURE_SHADERS_DIR;
std::string font_root_path = LAMURE_FONTS_DIR;

LamureRenderer::LamureRenderer(Lamure *plugin) : 
    m_plugin(plugin)
{
    m_renderer = this;

    m_group = new osg::Group();
    m_group->setName("LamureRendererGroup");
}

LamureRenderer::~LamureRenderer()
{
}

struct InitDrawCallback : public osg::Drawable::DrawCallback {
    InitDrawCallback(osg::ref_ptr<osg::StateSet> stateset, Lamure* plugin) : _stateset(stateset), _plugin(plugin), _initialized(false) 
    {
        if (_plugin->getUI()->getNotifyButton()->state()) { std::cout << "[Notify] InitDrawCallback()" << std::endl; }
        _renderer = _plugin->getRenderer();
    }

    virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const override {
        scm::math::mat4d modelview_matrix = LamureUtil::matConv4D(osg::Matrixd(renderInfo.getState()->getModelViewMatrix()));
        scm::math::mat4d projection_matrix = LamureUtil::matConv4D(osg::Matrixd(renderInfo.getState()->getProjectionMatrix()));

        _renderer->setModelviewMatrix(modelview_matrix);
        _renderer->setProjectionMatrix(projection_matrix);

        if (_plugin->getUI()->getSyncButton()->state() == 1) {
            _renderer->getScmCamera()->set_view_matrix(modelview_matrix);
            _renderer->getScmCamera()->set_projection_matrix(projection_matrix);
        }
        if (!_initialized) {
            GLState before = GLState::capture();
            std::cout << "test" << std::endl;
            _renderer->initSchismObjects();
            _renderer->initFramebuffer();
            _renderer->initLamureShader();
            _renderer->initUniforms();
            _renderer->initFrustumResources();
            _renderer->initBoxResources();
            _renderer->initCoordResources();
            _renderer->getPointcloudGeode()->setNodeMask(_plugin->getUI()->getPointcloudButton()->state() ? 0xFFFFFFFF : 0);
            _renderer->getBoundingboxGeode()->setNodeMask(_plugin->getUI()->getBoundingboxButton()->state() ? 0xFFFFFFFF : 0);
            _renderer->getFrustumGeode()->setNodeMask(_plugin->getUI()->getFrustumButton()->state() ? 0xFFFFFFFF : 0);
            _renderer->getCoordGeode()->setNodeMask(_plugin->getUI()->getCoordButton()->state() ? 0xFFFFFFFF : 0);
            _renderer->getTextGeode()->setNodeMask(_plugin->getUI()->getTextButton()->state() ? 0xFFFFFFFF : 0);
            _initialized = true;
            before.restore();
        }
    }
    osg::ref_ptr<osg::StateSet> _stateset;
    Lamure* _plugin;
    LamureRenderer* _renderer;
    mutable bool _initialized;
};

struct InitGeometry : public osg::Geometry {
    InitGeometry(osg::ref_ptr<osg::StateSet> stateset, Lamure* plugin) : _plugin(plugin) {
        if (_plugin->getUI()->getNotifyButton()->state()) { std::cout << "[Notify] InitGeometry()" << std::endl; }
        setUseDisplayList(false);
        setUseVertexBufferObjects(true);
        setUseVertexArrayObject(false);
        setDrawCallback(new InitDrawCallback(stateset, plugin));
    }
    Lamure* _plugin;
};

void updateFrustumTransform(osg::ref_ptr<osg::MatrixTransform> matrixTransform, const osg::Vec3& translation) {
    osg::Matrix transMatrix = osg::Matrix::translate(translation);
    matrixTransform->setMatrix(transMatrix);
};

struct TextCullCallback : public osg::Drawable::CullCallback
{
    TextCullCallback(Lamure* plugin, osgText::Text* values, Lamure::RenderInfo* render_info)
        : _plugin(plugin),
        _values(values),
        _render_info(render_info)
    {
        _lastUpdateTime = std::chrono::steady_clock::now();
        _minInterval = std::chrono::milliseconds(100);
        _renderer = _plugin->getRenderer();
    }

    virtual bool cull(osg::NodeVisitor* nv, osg::Drawable* drawable, osg::RenderInfo* renderInfo) const override
    {
        auto now = std::chrono::steady_clock::now();
        if (now - _lastUpdateTime >= _minInterval)
        {
            scm::math::vec3d camPos = _renderer->getScmCamera()->get_cam_pos();

            osg::Matrix baseMatrix = opencover::VRSceneGraph::instance()->getScaleTransform()->getMatrix();
            osg::Matrix transformMatrix = opencover::VRSceneGraph::instance()->getTransform()->getMatrix();
            baseMatrix.postMult(transformMatrix);

            scm::math::mat4d osg_base = LamureUtil::matConv4D(baseMatrix);
            scm::math::mat4d osg_view = LamureUtil::matConv4D(_renderer->getOsgCamera()->getViewMatrix());
            scm::math::mat4d osg_projection = LamureUtil::matConv4D(_renderer->getOsgCamera()->getProjectionMatrix());

            std::stringstream osg_base_ss;
            std::stringstream osg_projection_ss;
            std::stringstream osg_mvp_ss;
            std::stringstream gl_modelview_ss;
            std::stringstream gl_projection_ss;
            std::stringstream gl_mvp_ss;
            std::stringstream scm_modelview_ss;
            std::stringstream scm_projection_ss;
            std::stringstream scm_mvp_ss;
            std::stringstream value_ss;

            osg_base_ss << osg_view * osg_base;
            osg_projection_ss << osg_projection;
            osg_mvp_ss << osg_projection * osg_view * osg_base;
            gl_modelview_ss << _renderer->getModelviewMatrix();
            gl_projection_ss << _renderer->getProjextionMatrix();
            gl_mvp_ss << _renderer->getProjextionMatrix() * _renderer->getModelviewMatrix();
            scm_modelview_ss << _renderer->getScmCamera()->get_view_matrix();
            scm_projection_ss << _renderer->getScmCamera()->get_projection_matrix();
            scm_mvp_ss << _renderer->getScmCamera()->get_projection_matrix() * _renderer->getScmCamera()->get_view_matrix();
            value_ss << "\n"
                << std::fixed << std::setprecision(2)
                << 1.0f / opencover::cover->frameDuration() << "\n"
                << _render_info->rendered_nodes_ << "\n"
                << _render_info->rendered_splats_ << "\n"
                << _render_info->rendered_bounding_boxes_ << "\n\n\n"
                << camPos.x << "\n"
                << camPos.y << "\n"
                << camPos.z << "\n\n\n\n"
                << osg_base_ss.str() << "\n\n\n"
                << osg_projection_ss.str() << "\n\n\n"
                << osg_mvp_ss.str() << "\n\n\n"
                << gl_modelview_ss.str() << "\n\n\n"
                << gl_projection_ss.str() << "\n\n\n"
                << gl_mvp_ss.str() << "\n\n\n"
                << scm_modelview_ss.str() << "\n\n\n"
                << scm_projection_ss.str() << "\n\n\n"
                << scm_mvp_ss.str() << "\n";
            _values->setText(value_ss.str(), osgText::String::ENCODING_UTF8);
            _lastUpdateTime = now;
        }
        return false;
    }
    Lamure* _plugin;
    LamureRenderer* _renderer;
    osg::ref_ptr<osgText::Text> _values;
    Lamure::RenderInfo* _render_info;
    mutable std::chrono::steady_clock::time_point _lastUpdateTime;
    std::chrono::milliseconds _minInterval;
};

struct TextGeode : public osg::Geode
{
    TextGeode(Lamure* plugin)
    {
        if (plugin->getUI()->getNotifyButton()->state()) { std::cout << "[Notify] TextGeode()" << std::endl; }
        osg::Quat rotation(osg::DegreesToRadians(90.0f), osg::Vec3(1.0f, 0.0f, 0.0f));
        osg::Vec4 color(1.0f, 1.0f, 1.0f, 1.0f);
        std::string font = opencover::coVRFileManager::instance()->getFontFile(NULL);
        float characterSize = 20.0f;
        const osg::GraphicsContext::Traits* traits = opencover::coVRConfig::instance()->windows[0].context->getTraits();
        osg::Vec3 pos_label(+traits->width * 0.5f, 0.0f, traits->height * 0.7f);
        osg::Vec3 pos_value = pos_label + osg::Vec3(100.0f, 0.0f, 0.0f);
        osg::ref_ptr<osgText::Text> label = new osgText::Text();
        label->setRotation(rotation);
        label->setColor(color);
        label->setFont(font);
        label->setCharacterSize(characterSize);
        label->setPosition(pos_label);
        std::stringstream label_ss;
        label_ss << "Rendering" << "\n"
            << "FPS:" << "\n"
            << "Nodes:" << "\n"
            << "Splats:" << "\n"
            << "Boxes:" << "\n\n"
            << "Frustum Position" << "\n"
            << "X:" << "\n"
            << "Y:" << "\n"
            << "Z:" << "\n\n\n"
            << "OSG BASE:" << "\n\n\n\n\n\n"
            << "OSG Projection:" << "\n\n\n\n\n\n"
            << "OSG MVP:" << "\n\n\n\n\n\n"
            << "GL ModelView:" << "\n\n\n\n\n\n"
            << "GL Projection:" << "\n\n\n\n\n\n"
            << "GL MVP:" << "\n\n\n\n\n\n"
            << "SCM ModelView:" << "\n\n\n\n\n\n"
            << "SCM Projection:" << "\n\n\n\n\n\n"
            << "SCM MVP:" << "\n";
        label->setText(label_ss.str(), osgText::String::ENCODING_UTF8);

        osg::ref_ptr<osgText::Text> value = new osgText::Text();
        value->setRotation(rotation);
        value->setColor(color);
        value->setFont(font);
        value->setCharacterSize(characterSize);
        value->setPosition(pos_value);
        std::stringstream value_ss;
        value_ss << "\n"
            << "0.00:" << "\n"
            << "0.00" << "\n"
            << "0.00" << "\n"
            << "0.00:" << "\n\n\n"
            << "0.00" << "\n"
            << "0.00" << "\n"
            << "0.00" << "\n\n\n\n\n"
            << "0.00" << "\n\n\n\n"
            << "0.00" << "\n\n\n\n"
            << "0.00" << "\n\n\n\n"
            << "0.00" << "\n\n\n\n"
            << "0.00" << "\n\n\n\n"
            << "0.00" << "\n\n\n\n"
            << "0.00" << "\n";
        value->setText(value_ss.str(), osgText::String::ENCODING_UTF8);
        this->addDrawable(label.get());
        this->addDrawable(value.get());
        value->setCullCallback(new TextCullCallback(plugin, value.get(), &plugin->getRenderInfo()));
    }
};

struct CoordDrawCallback : public osg::Drawable::DrawCallback
{
    CoordDrawCallback(osg::ref_ptr<osg::StateSet> stateset, Lamure* plugin)
        : _stateset(stateset),
        _plugin(plugin),
        _initialized(false) 
    {
        _renderer = _plugin->getRenderer();
    }

    virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const override
    {
        GLState before = GLState::capture();
        glPushAttrib(GL_ALL_ATTRIB_BITS);

        glBindVertexArray(_renderer->getCoordResource().vao_);
        glBindBuffer(GL_ARRAY_BUFFER, _renderer->getCoordResource().vbo_);
        glUseProgram(_renderer->getLineShader().program);

        scm::math::mat4 view_matrix_ = LamureUtil::matConv4F(osg::Matrix(renderInfo.getState()->getModelViewMatrix()));
        scm::math::mat4 projection_matrix_ = LamureUtil::matConv4F(osg::Matrix(renderInfo.getState()->getProjectionMatrix()));
        scm::math::mat4 mvp_matrix = projection_matrix_ * view_matrix_;

        glUniformMatrix4fv(_renderer->getLineShader().mvp_matrix_location, 1, GL_FALSE, mvp_matrix.data_array);
        glUniform4f(_renderer->getLineShader().in_color_location, _plugin->getSettings().frustum_color_[0], _plugin->getSettings().frustum_color_[1], _plugin->getSettings().frustum_color_[2], _plugin->getSettings().frustum_color_[3]);
        glDrawElements(GL_LINES, _renderer->getCoordResource().idx_.size(), GL_UNSIGNED_SHORT, nullptr);

        glPopAttrib();
        before.restore();
    }
    osg::ref_ptr<osg::StateSet> _stateset;
    Lamure* _plugin;
    LamureRenderer* _renderer;
    mutable bool _initialized;
};

struct CoordGeometry : public osg::Geometry
{
    CoordGeometry(osg::ref_ptr<osg::StateSet> stateset, Lamure* plugin)
        : _stateset(stateset), _plugin(plugin)
    {
        if (plugin->getUI()->getNotifyButton()->state()) { std::cout << "[Notify] CoordGeometry()" << std::endl; }
        setUseDisplayList(false);
        setUseVertexBufferObjects(true);
        setUseVertexArrayObject(false);
        setDrawCallback(new CoordDrawCallback(stateset, plugin));
    }
    osg::ref_ptr<osg::StateSet> _stateset;
    Lamure* _plugin;
};

struct FrustumDrawCallback : public osg::Drawable::DrawCallback
{
    FrustumDrawCallback(osg::ref_ptr<osg::StateSet> stateset, Lamure* plugin)
        : _stateset(stateset),
        _plugin(plugin),
        _initialized(false) 
    {
        _renderer = _plugin->getRenderer();
    }

    virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const override
    {
        GLState before = GLState::capture();

        if (!_initialized)
        {
            _renderer->initFrustumResources();
            _initialized = true;
        }

        std::vector<scm::math::vec3d> corner_values = _renderer->getScmCamera()->get_frustum_corners();
        for (size_t i = 0; i < corner_values.size(); ++i) {
            auto vv = scm::math::vec3f(corner_values[i]);
            _renderer->getFrustumResource().vertices_[i * 3 + 0] = vv.x;
            _renderer->getFrustumResource().vertices_[i * 3 + 1] = vv.y;
            _renderer->getFrustumResource().vertices_[i * 3 + 2] = vv.z;
        }

        glBindVertexArray(_renderer->getFrustumResource().vao_);
        glBindBuffer(GL_ARRAY_BUFFER, _renderer->getFrustumResource().vbo_);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * _renderer->getFrustumResource().vertices_.size(), _renderer->getFrustumResource().vertices_.data(), GL_STATIC_DRAW);
        glUseProgram(_renderer->getLineShader().program);

        scm::math::mat4 view_matrix_ = LamureUtil::matConv4F(osg::Matrix(renderInfo.getState()->getModelViewMatrix()));
        scm::math::mat4 projection_matrix_ = LamureUtil::matConv4F(osg::Matrix(renderInfo.getState()->getProjectionMatrix()));
        scm::math::mat4 mvp_matrix = projection_matrix_ * view_matrix_;

        glUniformMatrix4fv(_renderer->getLineShader().mvp_matrix_location, 1, GL_FALSE, mvp_matrix.data_array);
        glUniform4f(_renderer->getLineShader().in_color_location, _plugin->getSettings().frustum_color_[0], _plugin->getSettings().frustum_color_[1], _plugin->getSettings().frustum_color_[2], _plugin->getSettings().frustum_color_[3]);
        glDrawElements(GL_LINES, _renderer->getFrustumResource().idx_.size(), GL_UNSIGNED_SHORT, nullptr);

        before.restore();
    }
    osg::ref_ptr<osg::StateSet> _stateset;
    Lamure* _plugin;
    LamureRenderer* _renderer;
    mutable bool _initialized;
};

struct FrustumGeometry : public osg::Geometry
{
    FrustumGeometry(osg::ref_ptr<osg::StateSet> stateset, Lamure* _plugin)
    {
        if (_plugin->getUI()->getNotifyButton()->state()) { std::cout << "[Notify] FrustumGeometryGL()" << std::endl; }
        setUseDisplayList(false);
        setUseVertexBufferObjects(true);
        setUseVertexArrayObject(false);
        setDrawCallback(new FrustumDrawCallback(stateset, _plugin));
    }
};

struct BoundingBoxDrawCallback : public virtual osg::Drawable::DrawCallback
{
    BoundingBoxDrawCallback(osg::ref_ptr<osg::StateSet> stateset, Lamure* plugin)
        : _stateset(stateset),
        _plugin(plugin)
    {
        if (_plugin->getUI()->getNotifyButton()->state()) { std::cout << "[Notify] BoundingBoxDrawCallback()" << std::endl; }
        _renderer = _plugin->getRenderer();
    }

    virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const {

        GLState before = GLState::capture();
        glPushAttrib(GL_ALL_ATTRIB_BITS);

        osg::State* state = renderInfo.getState();
        scm::math::mat4 view_matrix_ = LamureUtil::matConv4F(state->getModelViewMatrix());
        scm::math::mat4 projection_matrix_ = LamureUtil::matConv4F(state->getProjectionMatrix());
        scm::math::mat4 osg_scale = LamureUtil::matConv4F(opencover::cover->getObjectsScale()->getMatrix());

        lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
        lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
        lamure::ren::controller* controller = lamure::ren::controller::get_instance();
        lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();
        if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) { controller->reset_system(_plugin->getDataProvenance()); }
        else { controller->reset_system(); }

        lamure::context_t context_id = controller->deduce_context_id(_renderer->getOsgCamera()->getGraphicsContext()->getState()->getContextID());
        for (lamure::model_t model_id = 0; model_id < _plugin->getSettings().num_models; ++model_id) {
            lamure::model_t m_id = controller->deduce_model_id(std::to_string(model_id));
            cuts->send_transform(context_id, m_id, osg_scale * scm::math::mat4(_plugin->getModelInfo().model_transformations_[m_id]));
            cuts->send_threshold(context_id, m_id, _plugin->getSettings().lod_error_);
            cuts->send_rendered(context_id, m_id);
            database->get_model(m_id)->set_transform(osg_scale * scm::math::mat4(_plugin->getModelInfo().model_transformations_[m_id]));
        }

        lamure::view_t view_id = controller->deduce_view_id(context_id, _renderer->getScmCamera()->view_id());
        cuts->send_camera(context_id, view_id, *_renderer->getScmCamera());
        std::vector<scm::math::vec3d> corner_values = _renderer->getScmCamera()->get_frustum_corners();
        double top_minus_bottom = scm::math::length((corner_values[2]) - (corner_values[0]));
        float height_divided_by_top_minus_bottom_ = opencover::coVRConfig::instance()->windows[0].context->getTraits()->height / top_minus_bottom;
        cuts->send_height_divided_by_top_minus_bottom(context_id, view_id, height_divided_by_top_minus_bottom_);

        if (_plugin->getSettings().use_pvs_) {
            scm::math::vec3d cam_pos = _renderer->getScmCamera()->get_cam_pos();
            pvs->set_viewer_position(cam_pos);
        }

        if (_plugin->getSettings().lod_update_) {
            if (lamure::ren::policy::get_instance()->size_of_provenance() > 0)
            { controller->dispatch(context_id, _renderer->getDevice(), _plugin->getDataProvenance()); }
            else { controller->dispatch(context_id, _renderer->getDevice()); }
        }

        glBindVertexArray(_renderer->getBoxResource().vao_);
        glUseProgram(_renderer->getLineShader().program);
        glUniform4f(_renderer->getLineShader().in_color_location, _plugin->getSettings().bvh_color_[0], _plugin->getSettings().bvh_color_[1], _plugin->getSettings().bvh_color_[2], _plugin->getSettings().bvh_color_[3]);

        uint64_t rendered_bounding_boxes = 0;
        for (uint16_t model_id = 0; model_id < _plugin->getSettings().num_models; ++model_id) {
            if (!_plugin->getUI()->getModelVisibility()[model_id]) { continue; }
            lamure::ren::cut& cut = cuts->get_cut(context_id, _renderer->getOsgCamera()->getGraphicsContext()->getState()->getContextID(), model_id);
            std::vector<lamure::ren::cut::node_slot_aggregate> renderable = cut.complete_set();
            const lamure::ren::bvh* bvh = database->get_model(model_id)->get_bvh();
            std::vector<scm::gl::boxf>const& bbv = bvh->get_bounding_boxes();
            scm::math::mat4 model_matrix_ = scm::math::mat4(_plugin->getModelInfo().model_transformations_[model_id]);
            scm::math::mat4 mvp_matrix_ = projection_matrix_ * view_matrix_ * model_matrix_;
            scm::gl::frustum frustum_ = _renderer->getScmCamera()->get_frustum_by_model(model_matrix_);
            glUniformMatrix4fv(_renderer->getLineShader().mvp_matrix_location, 1, GL_FALSE, mvp_matrix_.data_array);
            for (auto const& node_slot_aggregate : renderable) {
                uint32_t node_culling_result = _renderer->getScmCamera()->cull_against_frustum(frustum_, bbv[node_slot_aggregate.node_id_]);
                if (node_culling_result != 1) {
                    std::map<uint32_t, LamureRenderer::Resource> bvh_res = _renderer->getBvhResource();
                    const std::vector<float>& corners_ = bvh_res[model_id].corners_[node_slot_aggregate.node_id_];
                    glBindBuffer(GL_ARRAY_BUFFER, _renderer->getBoxResource().vbo_);
                    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * corners_.size(), corners_.data());
                    glDrawElements(GL_LINES, 24, GL_UNSIGNED_SHORT, nullptr);
                    rendered_bounding_boxes++;
                }
            }
        }
        _plugin->getRenderInfo().rendered_bounding_boxes_ = rendered_bounding_boxes;
        glPopAttrib();
        before.restore();
    };
    osg::ref_ptr<osg::StateSet> _stateset;
    Lamure* _plugin;
    LamureRenderer* _renderer;
};

struct BoundingBoxGeometry : public osg::Geometry
{
    BoundingBoxGeometry(osg::ref_ptr<osg::StateSet> stateset, Lamure* plugin)
    {
        if (plugin->getUI()->getNotifyButton()->state()) { std::cout << "[Notify] BoundingBoxGeometry()" << std::endl; }
        setUseDisplayList(false);
        setUseVertexBufferObjects(true);
        setUseVertexArrayObject(false);
        setDrawCallback(new BoundingBoxDrawCallback(stateset, plugin));
    }
};

struct PointsDrawCallback : public virtual osg::Drawable::DrawCallback
{
    PointsDrawCallback(osg::ref_ptr<osg::StateSet> pointcloud_stateset, Lamure* plugin)
        : _stateset(pointcloud_stateset),
        _plugin(plugin),
        _initialized(false)
    { 
        if (_plugin->getUI()->getNotifyButton()->state()) { std::cout << "[Notify] PointsDrawCallback()" << std::endl; } 
        _renderer = _plugin->getRenderer();
    }

    virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const
    {
        if (_plugin->rendering_) { return; }
        _plugin->rendering_ = true;

        GLState before = GLState::capture();
        glDisable(GL_CULL_FACE);

        osg::State* state = renderInfo.getState();
        state->setCheckForGLErrors(osg::State::CheckForGLErrors::ONCE_PER_ATTRIBUTE);

        scm::math::mat4 view_matrix_ = LamureUtil::matConv4F(osg::Matrix(renderInfo.getState()->getModelViewMatrix()));
        scm::math::mat4 projection_matrix_ = LamureUtil::matConv4F(osg::Matrix(renderInfo.getState()->getProjectionMatrix()));
        scm::math::mat4 osg_scale = LamureUtil::matConv4F(opencover::cover->getObjectsScale()->getMatrix());

        lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
        lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
        lamure::ren::controller* controller = lamure::ren::controller::get_instance();
        lamure::pvs::pvs_database* pvs = lamure::pvs::pvs_database::get_instance();

        if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) { controller->reset_system(_plugin->getDataProvenance()); }
        else { controller->reset_system(); }

        lamure::context_t context_id = controller->deduce_context_id(_renderer->getOsgCamera()->getGraphicsContext()->getState()->getContextID());
        lamure::view_t    view_id = controller->deduce_view_id(context_id, _renderer->getScmCamera()->view_id());
        size_t surfels_per_node = database->get_primitives_per_node();

        for (lamure::model_t model_id = 0; model_id < _plugin->getSettings().num_models; ++model_id) {
            lamure::model_t m_id = controller->deduce_model_id(std::to_string(model_id));
            cuts->send_transform(context_id, m_id, scm::math::mat4(_plugin->getModelInfo().model_transformations_[m_id]));
            cuts->send_threshold(context_id, m_id, _plugin->getSettings().lod_error_);
            cuts->send_rendered(context_id, m_id);
            database->get_model(m_id)->set_transform(scm::math::mat4(_plugin->getModelInfo().model_transformations_[m_id]));
        }
        cuts->send_camera(context_id, view_id, *(_renderer->getScmCamera()));
        std::vector<scm::math::vec3d> corner_values = _renderer->getScmCamera()->get_frustum_corners();
        double top_minus_bottom = scm::math::length((corner_values[2]) - (corner_values[0]));
        float height_divided_by_top_minus_bottom_ = opencover::coVRConfig::instance()->windows[0].context->getTraits()->height / top_minus_bottom;
        cuts->send_height_divided_by_top_minus_bottom(context_id, view_id, height_divided_by_top_minus_bottom_);

        if (_plugin->getSettings().use_pvs_) {
            scm::math::vec3d cam_pos = _renderer->getScmCamera()->get_cam_pos();
            pvs->set_viewer_position(cam_pos);
        }
        if (_plugin->getSettings().lod_update_) {
            if (lamure::ren::policy::get_instance()->size_of_provenance() > 0)
            { controller->dispatch(context_id, _renderer->getDevice(), _plugin->getDataProvenance()); }
            else { controller->dispatch(context_id, _renderer->getDevice()); }
        }

        if (_initialized) { glBindVertexArray(_renderer->getPclResource().vao_); }
        _renderer->getContext()->apply_vertex_input();
        if (lamure::ren::policy::get_instance()->size_of_provenance() > 0) {
            _renderer->getContext()->bind_vertex_array(controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, _renderer->getDevice(), _plugin->getDataProvenance()));
        }
        else { _renderer->getContext()->bind_vertex_array(controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::POINTCLOUD, _renderer->getDevice())); }

        const scm::math::mat4d viewport_scale = scm::math::make_scale(opencover::coVRConfig::instance()->windows[0].context->getTraits()->width * 0.5, opencover::coVRConfig::instance()->windows[0].context->getTraits()->height * 0.5, 0.5);
        const scm::math::mat4d viewport_translate = scm::math::make_translation(1.0, 1.0, 1.0);
        const scm::math::mat4 model_to_screen_matrix_ = scm::math::mat4(viewport_scale * viewport_translate);
        scm::math::vec3 eye_ = scm::math::vec3f(_renderer->getScmCamera()->get_cam_pos());
        scm::math::vec2 viewport_ = scm::math::vec2f(opencover::coVRConfig::instance()->windows[0].context->getTraits()->width, opencover::coVRConfig::instance()->windows[0].context->getTraits()->height);

        if (!_plugin->getSettings().surfel_shader_ && !_plugin->getSettings().provenance_) {
            glUseProgram(_renderer->getPointShader().program);
            glEnable(GL_POINT_SMOOTH);
            glEnable(GL_PROGRAM_POINT_SIZE);
            glUniform1f(_renderer->getPointShader().max_radius_loc, _plugin->getSettings().max_radius_);
            glUniform1f(_renderer->getPointShader().scale_radius_loc, _plugin->getSettings().scale_radius_);
            glUniform1f(_renderer->getPointShader().point_size_factor_loc,  _plugin->getSettings().point_size_factor_ * opencover::cover->getScale());
            glUniform1f(_renderer->getPointShader().proj_scale_loc, viewport_.y * 0.5f * projection_matrix_.data_array[5]);
        }
        else if (_plugin->getSettings().surfel_shader_ && !_plugin->getSettings().provenance_) {
            glUseProgram(_renderer->getSurfelShader().program);
            glUniform1f(_renderer->getSurfelShader().max_radius_loc, _plugin->getSettings().max_radius_);
            glUniform1f(_renderer->getSurfelShader().scale_radius_loc,_plugin->getSettings().scale_radius_);
            glUniform1f(_renderer->getSurfelShader().surfel_size_factor_loc,_plugin->getSettings().surfel_size_factor_);
            glUniform1f(_renderer->getSurfelShader().proj_scale_loc, viewport_.y * 0.5f * projection_matrix_.data_array[5]);
            glUniform2f(_renderer->getSurfelShader().viewport_loc, viewport_.x, viewport_.y);
        }

        uint64_t rendered_splats_ = 0;
        uint64_t rendered_nodes_ = 0;

        for (uint16_t model_id = 0; model_id < _plugin->getSettings().num_models; ++model_id) {
            if (!_plugin->getUI()->getModelVisibility()[model_id]) { continue; }
            lamure::ren::cut& cut = cuts->get_cut(context_id, _renderer->getOsgCamera()->getGraphicsContext()->getState()->getContextID(), model_id);
            std::vector<lamure::ren::cut::node_slot_aggregate> renderable = cut.complete_set();
            const lamure::ren::bvh* bvh = database->get_model(model_id)->get_bvh();
            std::vector<scm::gl::boxf>const& bounding_box_vector = bvh->get_bounding_boxes();

            scm::math::mat4 model_matrix_ = scm::math::mat4(_plugin->getModelInfo().model_transformations_[model_id]);
            scm::math::mat4 model_view_matrix_ = view_matrix_ * model_matrix_;
            scm::math::mat4 mvp_matrix_ = projection_matrix_ * model_view_matrix_;
            scm::gl::frustum frustum_ = _renderer->getScmCamera()->get_frustum_by_model(model_matrix_);

            if (!_plugin->getSettings().surfel_shader_ && !_plugin->getSettings().provenance_) {
                glUniformMatrix4fv(_renderer->getPointShader().mvp_matrix_loc, 1, GL_FALSE, mvp_matrix_.data_array);
            }
            else if (_plugin->getSettings().surfel_shader_ && !_plugin->getSettings().provenance_) {
                glUniformMatrix4fv(_renderer->getSurfelShader().model_view_matrix_loc,     1, GL_FALSE, model_view_matrix_.data_array);
                glUniformMatrix4fv(_renderer->getSurfelShader().mvp_matrix_loc, 1, GL_FALSE, mvp_matrix_.data_array);
            }

            for (auto const& node_slot_aggregate : renderable) {
                if (_renderer->getScmCamera()->cull_against_frustum(frustum_, bounding_box_vector[node_slot_aggregate.node_id_]) != 1) {
                    glDrawArrays(scm::gl::PRIMITIVE_POINT_LIST, (node_slot_aggregate.slot_id_) * (GLsizei)surfels_per_node, surfels_per_node);
                    rendered_splats_ += surfels_per_node;
                    ++rendered_nodes_;
                }
            }
        }

        _plugin->getRenderInfo().rendered_splats_ = rendered_splats_;
        _plugin->getRenderInfo().rendered_nodes_ = rendered_nodes_;
        _plugin->rendering_ = false;

        if (!_initialized) {
            GLState after = GLState::capture();
            if (after.getVertexArrayBinding()
                != before.getVertexArrayBinding())
            {
                _renderer->getPclResource().vao_ = after.getVertexArrayBinding();
                _initialized = true;
            }
        }

        before.restore();
        if (_plugin->getUI()->getNotifyButton()->state()) {
            GLState after = GLState::capture();
            GLState::compare(before, after, "[Notify] PointsDrawCallback::drawImplementation()");
        }
    }
    osg::ref_ptr<osg::StateSet> _stateset;
    Lamure* _plugin;
    LamureRenderer* _renderer;
    mutable bool _initialized;
};

struct PointsGeometry : public osg::Geometry
{
    PointsGeometry(osg::ref_ptr<osg::StateSet> stateset, Lamure* plugin) :
        _plugin(plugin)
    {
        if (_plugin->getUI()->getNotifyButton()->state()) { std::cout << "[Notify] PointsGeometry()" << std::endl; }
        setUseDisplayList(false);
        setUseVertexBufferObjects(true);
        setUseVertexArrayObject(false);
        setDrawCallback(new PointsDrawCallback(stateset, _plugin));

        osg::Vec3 minPt = LamureUtil::vecConv3F(_plugin->getModelInfo().models_min);
        osg::Vec3 maxPt = LamureUtil::vecConv3F(_plugin->getModelInfo().models_max);
        osg::Vec3 halfExtents(std::max(fabs(minPt.x()), fabs(maxPt.x())),
            std::max(fabs(minPt.y()), fabs(maxPt.y())),
            std::max(fabs(minPt.z()), fabs(maxPt.z())));
        _bbox = osg::BoundingBox(-halfExtents, halfExtents);
        _bsphere = osg::BoundingSphere(_bbox.center(), _bbox.radius());

        setInitialBound(_bbox);
    }
    Lamure* _plugin;
    osg::BoundingSphere _bsphere;
    osg::BoundingBox _bbox;
};

void LamureRenderer::init()
{
    std::cout << "LamureRenderer::init()" << std::endl;
    initCamera();
    std::cout << "LamureRenderer::initCamera() end" << std::endl;

    m_init_geode         = new osg::Geode();
    m_pointcloud_geode   = new osg::Geode();
    m_boundingbox_geode  = new osg::Geode();
    m_frustum_geode      = new osg::Geode();
    m_coord_geode        = new osg::Geode();
    m_text_geode         = new TextGeode(m_plugin);

    m_init_stateset = new osg::StateSet();
    m_pointcloud_stateset = new osg::StateSet();
    m_boundingbox_stateset = new osg::StateSet();
    m_frustum_stateset = new osg::StateSet();
    m_coord_stateset = new osg::StateSet();
    m_text_stateset = new osg::StateSet();

    m_text_stateset->setRenderBinDetails(10, "RenderBin");
    auto ui = m_plugin->getUI();

    ui->getPointcloudButton()->setState(   m_plugin->getSettings().pointcloud_state );
    ui->getBoundingboxButton()->setState(  m_plugin->getSettings().boundingbox_state );
    ui->getFrustumButton()->setState(      m_plugin->getSettings().frustum_state );
    ui->getCoordButton()->setState(        m_plugin->getSettings().coord_state );
    ui->getTextButton()->setState(         m_plugin->getSettings().text_state );
    ui->getSyncButton()->setState(         m_plugin->getSettings().sync_state );
    ui->getNotifyButton()->setState(       m_plugin->getSettings().notify_state );

    m_pointcloud_geode->setNodeMask(0);
    m_boundingbox_geode->setNodeMask(0);
    m_frustum_geode->setNodeMask(0);
    m_coord_geode->setNodeMask(0);
    m_text_geode->setNodeMask(0);

    ui->getPointcloudButton()->setCallback([this](bool state) { m_pointcloud_geode->setNodeMask(state ? 0xFFFFFFFF : 0x0); });
    ui->getBoundingboxButton()->setCallback([this](bool state) { m_boundingbox_geode->setNodeMask(state ? 0xFFFFFFFF : 0x0); });
    ui->getFrustumButton()->setCallback([this](bool state) { m_frustum_geode->setNodeMask(state ? 0xFFFFFFFF : 0x0); });
    ui->getCoordButton()->setCallback([this](bool state) { m_coord_geode->setNodeMask(state ? 0xFFFFFFFF : 0x0); });
    ui->getTextButton()->setCallback([this](bool state) { m_text_geode->setNodeMask(state ? 0xFFFFFFFF : 0x0); });
    ui->getDumpButton()->setCallback([this](bool state) {  });

    m_init_geode->setStateSet(m_init_stateset.get());
    m_pointcloud_geode->setStateSet(m_pointcloud_stateset.get());
    m_boundingbox_geode->setStateSet(m_boundingbox_stateset.get());
    m_frustum_geode->setStateSet(m_frustum_stateset.get());
    m_coord_geode->setStateSet(m_coord_stateset.get());
    m_text_geode->setStateSet(m_text_stateset.get());

    osg::ref_ptr<osg::MatrixTransform> frustumTransform = new osg::MatrixTransform;
    updateFrustumTransform(frustumTransform, osg::Vec3(m_scm_camera->get_cam_pos()[0], m_scm_camera->get_cam_pos()[1], m_scm_camera->get_cam_pos()[2]));
    m_group->addChild(frustumTransform);
    m_group->addChild(m_frustum_geode);
    m_group->addChild(m_coord_geode);
    m_group->addChild(m_boundingbox_geode);
    m_group->addChild(m_pointcloud_geode);
    m_group->addChild(m_init_geode);
    m_hud_camera->addChild(m_text_geode.get());

    m_init_geometry = new InitGeometry(m_init_stateset, m_plugin);
    m_pointcloud_geometry = new PointsGeometry(m_pointcloud_stateset, m_plugin);
    m_boundingbox_geometry = new BoundingBoxGeometry(m_boundingbox_stateset, m_plugin);
    m_frustum_geometry = new FrustumGeometry(m_frustum_stateset, m_plugin);
    m_coord_geometry = new CoordGeometry(m_coord_stateset, m_plugin);

    m_init_geode->addDrawable(m_init_geometry);
    m_pointcloud_geode->addDrawable(m_pointcloud_geometry);
    m_boundingbox_geode->addDrawable(m_boundingbox_geometry);
    m_frustum_geode->addDrawable(m_frustum_geometry);
    m_coord_geode->addDrawable(m_coord_geometry);

    if (covise::coCoviseConfig::isOn("COVER.showRotationPoint", false)) {
        opencover::coVRNavigationManager::instance()->setRotationPointVisible(true);
        m_plugin->getTrackball().initial_pos_ = opencover::coVRNavigationManager::instance()->getRotationPoint();
        m_plugin->getTrackball().pos_ = opencover::coVRNavigationManager::instance()->getRotationPoint();
        m_plugin->getTrackball().size_ = covise::coCoviseConfig::getFloat("COVER.rotationPointSize", 1.0f);
        m_plugin->getTrackball().dist_ = (m_plugin->getTrackball().initial_pos_ - m_osg_camera->getViewMatrix().getTrans()).length();
    }

}

void LamureRenderer::initSchismObjects()
{
    if (!m_device)
    {
        m_device.reset(new scm::gl::render_device());
        if (!m_device)
        {
            std::cout << "error creating device" << std::endl;
        }
    }
    if (!m_context)
    {
        m_context = m_device->main_context();
        if (!m_context)
        {
            std::cout << "error creating context" << std::endl;
        }
    }
}

void LamureRenderer::initCamera()
{
    std::cout << "LamureRenderer::initCamera()" << std::endl;
    m_osg_camera = opencover::VRViewer::instance()->getCamera();
    lamure::context_t lmr_ctx = m_osg_camera->getGraphicsContext()->getState()->getContextID();

    double look_dist = 1.0;
    double left, right, bottom, top, zNear, zFar;
    osg::Vec3d eye, center, up;
    m_osg_camera->getProjectionMatrixAsFrustum(left, right, bottom, top, zNear, zFar);
    m_osg_camera->getViewMatrixAsLookAt(eye, center, up, look_dist);

    osg::Matrixd view = m_osg_camera->getViewMatrix();
    osg::Matrixd proj = m_osg_camera->getProjectionMatrix();

    m_scm_camera = new lamure::ren::camera((lamure::view_t)lmr_ctx, zNear, zFar, LamureUtil::matConv4D(view), LamureUtil::matConv4D(proj));

    osgViewer::Viewer::Windows windows;
    opencover::VRViewer::instance()->getWindows(windows);
    osgViewer::GraphicsWindow *window = windows.front();
    m_hud_camera = new osg::Camera();
    m_hud_camera->setName("hud_camera");
    m_hud_camera->setGraphicsContext(window);
    m_hud_camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    m_hud_camera->setProjectionResizePolicy(osg::Camera::FIXED);
    m_hud_camera->setViewMatrix(m_osg_camera->getViewMatrix());
    m_hud_camera->setProjectionMatrix(m_osg_camera->getProjectionMatrix());
    m_hud_camera->setViewport(0, 0, window->getTraits()->width, window->getTraits()->height);
    m_hud_camera->setRenderOrder(osg::Camera::POST_RENDER, 2);
    m_hud_camera->setClearMask(0);
    m_hud_camera->setRenderer(new osgViewer::Renderer(m_hud_camera.get()));
    m_osg_camera->addChild(m_hud_camera.get());
    
    scm::math::vec3f temp_center = scm::math::vec3f::zero();
    scm::math::vec3f root_min_temp = scm::math::vec3f::zero();
    scm::math::vec3f root_max_temp = scm::math::vec3f::zero();

    for (lamure::model_t model_id = 0; model_id < m_plugin->getSettings().num_models; ++model_id)
    {
        lamure::model_t m_id = lamure::ren::controller::get_instance()->deduce_model_id(std::to_string(model_id));

        auto root_bb = lamure::ren::model_database::get_instance()->get_model(model_id)->get_bvh()->get_bounding_boxes()[0];

        m_plugin->getModelInfo().root_bb_min.push_back(scm::math::mat4f(m_plugin->getModelInfo().model_transformations_[model_id]) * scm::math::vec4f(root_bb.min_vertex()[0], root_bb.min_vertex()[1], root_bb.min_vertex()[2], 1));
        m_plugin->getModelInfo().root_bb_max.push_back(scm::math::mat4f(m_plugin->getModelInfo().model_transformations_[model_id]) * scm::math::vec4f(root_bb.max_vertex()[0], root_bb.max_vertex()[1], root_bb.max_vertex()[2], 1));
        m_plugin->getModelInfo().root_center.push_back(scm::math::mat4f(m_plugin->getModelInfo().model_transformations_[model_id]) * scm::math::vec4f(root_bb.center()[0], root_bb.center()[1], root_bb.center()[2], 1));

        temp_center += m_plugin->getModelInfo().root_center.back();
        if (m_plugin->getModelInfo().root_bb_min[model_id][0] < root_min_temp[0])
        {
            root_min_temp[0] = m_plugin->getModelInfo().root_bb_min[model_id][0];
        }
        if (m_plugin->getModelInfo().root_bb_min[model_id][1] < root_min_temp[1])
        {
            root_min_temp[1] = m_plugin->getModelInfo().root_bb_min[model_id][1];
        }
        if (m_plugin->getModelInfo().root_bb_min[model_id][2] < root_min_temp[2])
        {
            root_min_temp[2] = m_plugin->getModelInfo().root_bb_min[model_id][2];
        }
        if (m_plugin->getModelInfo().root_bb_max[model_id][0] > root_max_temp[0])
        {
            root_max_temp[0] = m_plugin->getModelInfo().root_bb_max[model_id][0];
        }
        if (m_plugin->getModelInfo().root_bb_max[model_id][1] > root_max_temp[1])
        {
            root_max_temp[1] = m_plugin->getModelInfo().root_bb_max[model_id][1];
        }
        if (m_plugin->getModelInfo().root_bb_max[model_id][2] > root_max_temp[2])
        {
            root_max_temp[2] = m_plugin->getModelInfo().root_bb_max[model_id][2];
        }
    }
    m_plugin->getModelInfo().models_center = temp_center / m_plugin->getSettings().num_models;
    m_plugin->getModelInfo().models_min = root_min_temp;
    m_plugin->getModelInfo().models_max = root_max_temp;
}

void LamureRenderer::initFramebuffer()
{
    if (m_plugin->getUI()->getNotifyButton()->state()) { std::cout << "[Notify] initFramebuffer() " << std::endl; }
    fbo_.reset();
    fbo_color_buffer_.reset();
    fbo_depth_buffer_.reset();
    auto traits = opencover::coVRConfig::instance()->windows[0].context->getTraits();

    fbo_ = m_device->create_frame_buffer();
    fbo_color_buffer_ = m_device->create_texture_2d(scm::math::vec2ui(traits->width, traits->height), scm::gl::FORMAT_RGBA_32F, 1, 1, 1);
    fbo_depth_buffer_ = m_device->create_texture_2d(scm::math::vec2ui(traits->width, traits->height), scm::gl::FORMAT_D24, 1, 1, 1);
    fbo_->attach_color_buffer(0, fbo_color_buffer_);
    fbo_->attach_depth_stencil_buffer(fbo_depth_buffer_);
}


void LamureRenderer::initUniforms() {
    cout << "[Notify] initUniforms()" << endl;

    glUseProgram(m_point_shader.program);
    m_point_shader.mvp_matrix_loc         = glGetUniformLocation(m_point_shader.program, "mvp_matrix");
    m_point_shader.max_radius_loc         = glGetUniformLocation(m_point_shader.program, "max_radius");
    m_point_shader.scale_radius_loc		 = glGetUniformLocation(m_point_shader.program, "scale_radius");
    m_point_shader.point_size_factor_loc  = glGetUniformLocation(m_point_shader.program, "point_size_factor");
    m_point_shader.proj_scale_loc         = glGetUniformLocation(m_point_shader.program, "proj_scale");

    glUseProgram(m_surfel_shader.program);
    m_surfel_shader.mvp_matrix_loc           = glGetUniformLocation(m_surfel_shader.program, "mvp_matrix");
    m_surfel_shader.max_radius_loc           = glGetUniformLocation(m_surfel_shader.program, "max_radius");
    m_surfel_shader.scale_radius_loc			= glGetUniformLocation(m_surfel_shader.program, "scale_radius");
    m_surfel_shader.surfel_size_factor_loc   = glGetUniformLocation(m_surfel_shader.program, "surfel_size_factor");
    m_surfel_shader.proj_scale_loc           = glGetUniformLocation(m_surfel_shader.program, "proj_scale");
    m_surfel_shader.model_view_matrix_loc    = glGetUniformLocation(m_surfel_shader.program, "model_view_matrix");
    m_surfel_shader.viewport_loc             = glGetUniformLocation(m_surfel_shader.program, "viewport");

    glUseProgram(m_line_shader.program);
    m_line_shader.mvp_matrix_location = glGetUniformLocation(m_line_shader.program, "mvp_matrix");
    m_line_shader.in_color_location = glGetUniformLocation(m_line_shader.program, "in_color");

    glUseProgram(0);
}


bool LamureRenderer::readShader(std::string const &path_string, std::string &shader_string, bool keep_optional_shader_code = false)
{
    if (!boost::filesystem::exists(path_string))
    {
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
    while (std::getline(shader_source, line_buffer))
    {
        line_buffer = LamureUtil::stripWhitespace(line_buffer);
        if (LamureUtil::parsePrefix(line_buffer, include_prefix))
        {
            if (!disregard_code || keep_optional_shader_code)
            {
                std::string filename_string = line_buffer;
                readShader(base_path + filename_string, shader_string);
            }
        }
        else if (LamureUtil::parsePrefix(line_buffer, optional_begin_prefix))
        {
            disregard_code = true;
        }
        else if (LamureUtil::parsePrefix(line_buffer, optional_end_prefix))
        {
            disregard_code = false;
        }
        else
        {
            if ((!disregard_code) || keep_optional_shader_code)
            {
                shader_string += line_buffer + "\n";
            }
        }
    }
    return true;
}

void LamureRenderer::initLamureShader()
{
    try
    {
        if (!readShader(shader_root_path + "/vis/vis_point.glslv", vis_point_vs_source) ||
            !readShader(shader_root_path + "/vis/vis_point.glslf", vis_point_fs_source) ||
            !readShader(shader_root_path + "/vis/vis_point_prov.glslv", vis_point_prov_vs_source) ||
            !readShader(shader_root_path + "/vis/vis_point_prov.glslf", vis_point_prov_fs_source) ||
            !readShader(shader_root_path + "/vis/vis_surfel.glslv", vis_surfel_vs_source) ||
            !readShader(shader_root_path + "/vis/vis_surfel.glslg", vis_surfel_gs_source) ||
            !readShader(shader_root_path + "/vis/vis_surfel.glslf", vis_surfel_fs_source) || 
            !readShader(shader_root_path + "/vis/vis_surfel_prov.glslv", vis_surfel_prov_vs_source) || 
            !readShader(shader_root_path + "/vis/vis_surfel_prov.glslf", vis_surfel_prov_fs_source) || 
            !readShader(shader_root_path + "/vis/vis_line_bb.glslv", vis_line_bb_vs_source) || 
            !readShader(shader_root_path + "/vis/vis_line_bb.glslf", vis_line_bb_fs_source) || 
            !readShader(shader_root_path + "/vis/vis_quad.glslv", vis_quad_vs_source) || 
            !readShader(shader_root_path + "/vis/vis_quad.glslf", vis_quad_fs_source) || 
            !readShader(shader_root_path + "/vis/vis_line.glslv", vis_line_vs_source) || 
            !readShader(shader_root_path + "/vis/vis_line.glslf", vis_line_fs_source) || 
            !readShader(shader_root_path + "/vis/vis_triangle.glslv", vis_triangle_vs_source) || 
            !readShader(shader_root_path + "/vis/vis_triangle.glslf", vis_triangle_fs_source) || 
            !readShader(shader_root_path + "/vis/vis_plane.glslv", vis_plane_vs_source) || 
            !readShader(shader_root_path + "/vis/vis_plane.glslf", vis_plane_fs_source) ||
            !readShader(shader_root_path + "/vis/vis_text.glslv", vis_text_vs_source) || 
            !readShader(shader_root_path + "/vis/vis_text.glslf", vis_text_fs_source) || 
            !readShader(shader_root_path + "/vis/vis_box.glslv", vis_box_vs_source) || 
            !readShader(shader_root_path + "/vis/vis_box.glslg", vis_box_gs_source) || 
            !readShader(shader_root_path + "/vis/vis_box.glslf", vis_box_fs_source))
        {
            std::cout << "error reading shader files" << std::endl;
            exit(1);
        }

        m_point_shader.program = m_renderer->compileAndLinkShaders(vis_point_vs_source, vis_point_fs_source);
        m_surfel_shader.program = m_renderer->compileAndLinkShaders(vis_surfel_vs_source, vis_surfel_gs_source, vis_surfel_fs_source);
        m_line_shader.program = m_renderer->compileAndLinkShaders(vis_line_bb_vs_source, vis_line_bb_fs_source);
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
}
unsigned int LamureRenderer::createShader(const std::string& vertexShader, const std::string& fragmentShader, uint8_t ctx_id)
{
    osg::GLExtensions* gl_api = new osg::GLExtensions(ctx_id);
    unsigned int program = gl_api->glCreateProgram();
    unsigned int vs = compileShader(GL_VERTEX_SHADER, vertexShader, ctx_id);
    unsigned int fs = compileShader(GL_FRAGMENT_SHADER, fragmentShader, ctx_id);
    gl_api->glAttachShader(program, vs);
    gl_api->glAttachShader(program, fs);
    gl_api->glLinkProgram(program);
    gl_api->glValidateProgram(program);
    gl_api->glDeleteProgram(vs);
    gl_api->glDeleteProgram(fs);
    return 1;
}

unsigned int LamureRenderer::compileShader(unsigned int type, const std::string& source, uint8_t ctx_id)
{
    osg::GLExtensions* gl_api = new osg::GLExtensions(ctx_id);
    unsigned int id = gl_api->glCreateShader(type);
    const char* src = source.c_str();
    gl_api->glShaderSource(id, 1, &src, nullptr);
    gl_api->glCompileShader(id);
    int result;
    gl_api->glGetShaderiv(id, GL_COMPILE_STATUS, &result);
    if (result == false)
    {
        int length;
        gl_api->glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
        char* message = (char*)alloca(length * sizeof(char));
        gl_api->glGetShaderInfoLog(id, length, &length, message);
        std::cout << "Failed to compile " << (type == GL_VERTEX_SHADER ? "vertex" : "fragment") << " shader!" << std::endl;
        std::cout << message << std::endl;
        gl_api->glDeleteShader(id);
        return 0;
    };
    return id;
}

GLuint LamureRenderer::compileAndLinkShaders(std::string vs_source, std::string fs_source)
{
    GLuint program = glCreateProgram();
    GLuint vs = compileShader(GL_VERTEX_SHADER, vs_source, 0);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fs_source, 0);
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glValidateProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    return program;
}

GLuint LamureRenderer::compileAndLinkShaders(std::string vs_source, std::string gs_source, std::string fs_source)
{
    GLuint program = glCreateProgram();
    GLuint vs = compileShader(GL_VERTEX_SHADER, vs_source, 0);
    GLuint gs = compileShader(GL_GEOMETRY_SHADER, gs_source, 0);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fs_source, 0);
    glAttachShader(program, vs);
    glAttachShader(program, gs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glValidateProgram(program);
    glDeleteShader(vs);
    glDeleteShader(gs);
    glDeleteShader(fs);
    return program;
}


void LamureRenderer::initFrustumResources() {
    if (m_plugin->getUI()->getNotifyButton()->state()) {
        std::cout << "[Notify] create_frustum_resources() " << std::endl;
    }
    std::vector<scm::math::vec3d> corner_values = m_renderer->getScmCamera()->get_frustum_corners();
    for (size_t i = 0; i < corner_values.size(); ++i) {
        auto vv = scm::math::vec3f(corner_values[i]);
        m_frustum_resource.vertices_[i * 3 + 0] = vv.x;
        m_frustum_resource.vertices_[i * 3 + 1] = vv.y;
        m_frustum_resource.vertices_[i * 3 + 2] = vv.z;
    }
    GLuint vao_;
    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);
    GLuint ibo_;
    glGenBuffers(1, &ibo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_frustum_resource.idx_.size() * sizeof(unsigned short), m_frustum_resource.idx_.data(), GL_STATIC_DRAW);
    GLuint vbo_;
    glGenBuffers(1, &vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * m_frustum_resource.vertices_.size(), m_frustum_resource.vertices_.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    m_frustum_resource.vao_ = vao_;
    m_frustum_resource.vbo_ = vbo_;
    m_frustum_resource.ibo_ = ibo_;
    glBindVertexArray(0);
}

void LamureRenderer::initCoordResources() {
    if (m_plugin->getUI()->getNotifyButton()->state()) { 
        std::cout << "[Notify] init_coord_resources() " << std::endl;
    }
    GLuint vao_;
    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);
    GLuint ibo_;
    glGenBuffers(1, &ibo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_coord_resource.idx_.size() * sizeof(unsigned short), m_coord_resource.idx_.data(), GL_STATIC_DRAW);
    GLuint vbo_;
    glGenBuffers(1, &vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * m_coord_resource.vertices_.size(), m_coord_resource.vertices_.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    m_coord_resource.vao_ = vao_;
    m_coord_resource.vbo_ = vbo_;
    m_coord_resource.ibo_ = ibo_;
    glBindVertexArray(0);
}

void LamureRenderer::initBoxResources() {
    if (m_plugin->getUI()->getNotifyButton()->state()) { std::cout << "[Notify] init_box_resources() " << std::endl; }

    for (uint32_t model_id = 0; model_id < m_plugin->getSettings().num_models; ++model_id) {
        std::vector<vector<float>> corners_;
        const auto& bvh_ = lamure::ren::model_database::get_instance()->get_model(model_id)->get_bvh();
        const auto& bounding_boxes = bvh_->get_bounding_boxes();
        for (uint64_t node_id = 0; node_id < bounding_boxes.size(); ++node_id) {
            corners_.push_back(LamureUtil::getBoxCorners(bounding_boxes[node_id]));
        }
        m_bvh_resource[model_id].corners_ = corners_;
    }
    vector<float> vertices_ = LamureUtil::getBoxCorners(lamure::ren::model_database::get_instance()->get_model(0)->get_bvh()->get_bounding_boxes()[0]);
    GLuint vao_;
    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);
    GLuint ibo_;
    glGenBuffers(1, &ibo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_box_resource.idx_.size() * sizeof(unsigned short), m_box_resource.idx_.data(), GL_STATIC_DRAW);
    GLuint vbo_;
    glGenBuffers(1, &vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices_.size(), vertices_.data(), GL_STREAM_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    m_box_resource.vao_ = vao_;
    m_box_resource.vbo_ = vbo_;
    m_box_resource.ibo_ = ibo_;
    glBindVertexArray(0);
}
