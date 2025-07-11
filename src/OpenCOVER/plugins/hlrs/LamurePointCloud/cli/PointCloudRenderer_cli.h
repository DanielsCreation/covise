#ifndef POINT_CLOUD_RENDERER_CLI_H
#define POINT_CLOUD_RENDERER_CLI_H

#include <osg/Drawable>
#include <osg/RenderInfo>
#include <osg/StateSet>
#include <string>
#include <vector>
#include <GL/glew.h>
#include <scm/time.h>
#include <osgText/Text>
#include <osg/NodeCallback>

class LamurePointCloud_cli;

// Forward-declare geometry classes that use this renderer as callback
class PointsGeometry;
class BoundingBoxGeometry;
class FrustumGeometry;
class CoordGeometry;

class PointCloudRenderer
{
public:
    PointCloudRenderer(LamurePointCloud_cli* plugin);
    ~PointCloudRenderer();

    void initShaders();
    void initResources();

    // Draw callback implementations
    void drawPointCloud(osg::RenderInfo& renderInfo) const;
    void drawBoundingBoxes(osg::RenderInfo& renderInfo) const;
    void drawFrustum(osg::RenderInfo& renderInfo) const;
    void drawCoords(osg::RenderInfo& renderInfo) const;

private:
    // Helper class to save and restore OpenGL state
    class GLState {
    public:
        GLState() {
            glGetIntegerv(GL_CURRENT_PROGRAM, &program_);
            glGetIntegerv(GL_ACTIVE_TEXTURE, &activeTexture_);
            glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &arrayBufferBinding_);
            glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, &elementArrayBufferBinding_);
            glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &vertexArrayBinding_);
            glGetBooleanv(GL_CULL_FACE, &cullFaceEnabled_);
            glGetIntegerv(GL_CULL_FACE_MODE, &cullFaceMode_);
        }

        void restore() const {
            glUseProgram(program_);
            glActiveTexture(activeTexture_);
            glBindBuffer(GL_ARRAY_BUFFER, arrayBufferBinding_);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementArrayBufferBinding_);
            glBindVertexArray(vertexArrayBinding_);
            if (cullFaceEnabled_) {
                glEnable(GL_CULL_FACE);
            } else {
                glDisable(GL_CULL_FACE);
            }
            glCullFace(cullFaceMode_);
        }

        GLuint getVertexArrayBinding() const { return vertexArrayBinding_; }

    private:
        GLint program_;
        GLint activeTexture_;
        GLint arrayBufferBinding_;
        GLint elementArrayBufferBinding_;
        GLint vertexArrayBinding_;
        GLboolean cullFaceEnabled_;
        GLint cullFaceMode_;
    };

    bool readShader(const std::string& path, std::string& source);
    GLuint compileShader(GLenum type, const std::string& source) const;
    GLuint linkProgram(const std::vector<GLuint>& shaders) const;

    void initPointShader();
    void initSurfelShader();
    void initLineShader();

    // Member variables (snake_case with trailing underscore)
    LamurePointCloud_cli* plugin_ = nullptr;

    // Shader programs
    GLuint point_shader_program_ = 0;
    GLuint surfel_shader_program_ = 0;
    GLuint line_shader_program_ = 0;

    // Uniform locations
    struct PointShaderUniforms {
        GLint mvp_matrix_;
        GLint max_radius_;
        GLint scale_radius_;
        GLint point_size_factor_;
        GLint proj_scale_;
    } point_uniforms_;

    struct SurfelShaderUniforms {
        GLint mvp_matrix_;
        GLint model_view_matrix_;
        GLint max_radius_;
        GLint scale_radius_;
        GLint surfel_size_factor_;
        GLint proj_scale_;
        GLint viewport_;
    } surfel_uniforms_;

    struct LineShaderUniforms {
        GLint mvp_matrix_;
        GLint in_color_;
    } line_uniforms_;

    // GL resources
    GLuint pointcloud_vao_ = 0;
    GLuint boundingbox_vao_ = 0;
    GLuint boundingbox_vbo_ = 0;
    GLuint boundingbox_ibo_ = 0;
    GLuint frustum_vao_ = 0;
    GLuint frustum_vbo_ = 0;
    GLuint frustum_ibo_ = 0;
    GLuint coord_vao_ = 0;
    GLuint coord_vbo_ = 0;
    GLuint coord_ibo_ = 0;

    // Rendering context and settings
    lamure::context_t lmr_ctx_;
    uint32_t render_width_;
    uint32_t render_height_;
    lamure::ren::Data_Provenance data_provenance_;
    bool prov_valid_;
    float height_divided_by_top_minus_bottom_;
    uint32_t num_models_;

    scm::gl::render_device_ptr device_;
    scm::gl::render_context_ptr context_;
    scm::gl::quad_geometry_ptr screen_quad_;
    scm::gl::text_renderer_ptr text_renderer_;
    scm::gl::text_ptr renderable_text_;

    lamure::ren::camera* scm_camera_;
    osg::ref_ptr<osg::Camera> osg_camera_;
    osg::ref_ptr<osg::Camera> rtt_camera_;

    std::string shader_root_path_;

    // Resource structs
    struct Resource {
        GLuint vao_ = 0;
        GLuint vbo_ = 0;
        GLuint ibo_ = 0;
        std::vector<float> vertices_;
        std::vector<unsigned short> idx_;
    };
    Resource box_resource_;
    Resource frustum_resource_;
    Resource coord_resource_;

    // Model info
    struct ModelInfo {
        scm::math::vec3f models_min;
        scm::math::vec3f models_max;
    } model_info_;

    // Render info
    struct RenderInfo {
        uint64_t rendered_splats_ = 0;
        uint64_t rendered_nodes_ = 0;
        uint64_t rendered_bounding_boxes_ = 0;
    } render_info_;

    // BVH resources
    std::map<uint32_t, struct BvhResStruct { std::vector<float> corners_; }> bvh_res_;

    // Helper conversions
    osg::Vec3f vecConv3F(const scm::math::vec3f& v) const;
    scm::math::mat4f matConv4F(const osg::Matrixd& m) const;
    scm::math::mat4d matConv4D(const osg::Matrixd& m) const;

    const osg::GraphicsContext::Traits* traits_;

    // Nested class for text culling
    class TextCullCallback : public osg::NodeCallback
    {
    public:
        TextCullCallback(PointCloudRenderer* renderer) : renderer_(renderer) {}

        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            osg::Geode* geode = dynamic_cast<osg::Geode*>(node);
            if (geode)
            {
                for (unsigned int i = 0; i < geode->getNumDrawables(); ++i)
                {
                    osgText::Text* text = dynamic_cast<osgText::Text*>(geode->getDrawable(i));
                    if (text)
                    {
                        std::string text_string;
                        text_string  = "FPS: " + std::to_string(opencover::cover->getFPS()) + "\n";
                        text_string += "Rendered Splats: " + std::to_string(renderer_->render_info_.rendered_splats_) + "\n";
                        text_string += "Rendered Nodes: " + std::to_string(renderer_->render_info_.rendered_nodes_) + "\n";
                        text_string += "Rendered Bounding Boxes: " + std::to_string(renderer_->render_info_.rendered_bounding_boxes_) + "\n";
                        text->setText(text_string);
                    }
                }
            }
            traverse(node, nv);
        }
    private:
        PointCloudRenderer* renderer_;
    };

    // Nested class for text geode
    class TextGeode : public osg::Geode
    {
    public:
        TextGeode(PointCloudRenderer* renderer)
        {
            osgText::Text* text = new osgText::Text();
            text->setFont("arial.ttf");
            text->setFontResolution(100, 100);
            text->setColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
            text->setCharacterSize(20.0f);
            text->setAxisAlignment(osgText::Text::SCREEN);
            text->setPosition(osg::Vec3(10.0f, 10.0f, 0.0f));
            addDrawable(text);
            setCullCallback(new TextCullCallback(renderer));
        }
    };
};

#endif // POINT_CLOUD_RENDERER_CLI_H
