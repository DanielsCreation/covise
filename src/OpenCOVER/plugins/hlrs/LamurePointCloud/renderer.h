#ifndef _LAMURE_RENDERER_H
#define _LAMURE_RENDERER_H

//gl
#ifndef __gl_h_
#include <GL/glew.h>
#endif

#include <scm/core/math.h>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/Geode>
#include <osg/Camera>
#include <osg/State>
#include <osgViewer/Viewer>
#include <osgViewer/Renderer>
#include <osgText/Text>
#include <lamure/ren/camera.h>

#include <scm/core/pointer_types.h>

class Lamure;
class LamureRenderer {

public:

    std::map<uint32_t, std::vector<uint32_t>> m_bvh_node_vertex_offsets;



private:
    Lamure* m_plugin;
    LamureRenderer* m_renderer;

    osg::ref_ptr<osg::Group> m_group;

    bool m_rendering;

    // Private methods
    bool readShader(const std::string& pathString, std::string& shaderString, bool keepOptionalShaderCode);
    void initCamera();
    void printSettings() const;

    GLuint compileAndLinkShaders(std::string vsSource, std::string fsSource);
    GLuint compileAndLinkShaders(std::string vsSource, std::string gsSource, std::string fsSource);
    unsigned int createShader(const std::string& vertexShader, const std::string& fragmentShader, uint8_t ctxId);
    unsigned int compileShader(unsigned int type, const std::string& source, uint8_t ctxId);

    void setPointUniforms();
    void setSurfelUniforms();
    void setLamureUniforms(scm::gl::program_ptr shader);

    // Resource structs
    struct pcl_resource;
    struct box_resource;
    struct plane_resource;
    struct sphere_resource;
    struct coord_recourse;
    struct frustum_resource;
    struct text_resource;


    struct PointShader {
        GLuint program;
        GLint mvp_matrix_loc;
        GLint max_radius_loc;
        GLint scale_radius_loc;
        GLint point_size_factor_loc;
        GLint proj_scale_loc;
    };
    PointShader m_point_shader;

    struct SurfelShader {
        GLuint program;
        GLint max_radius_loc;
        GLint scale_radius_loc;
        GLint surfel_size_factor_loc;
        GLint mvp_matrix_loc;
        GLint model_view_matrix_loc;  // Neue Uniform Location
        GLint proj_scale_loc;
        GLint viewport_loc; 
    };
    SurfelShader m_surfel_shader;

    struct PointProvShader {
        GLuint program;
        // Core-Matrizen
        GLint mvp_matrix_loc;
        GLint view_matrix_loc;
        GLint projection_matrix_loc;
        GLint model_matrix_loc;
        GLint model_view_matrix_loc;
        GLint inverse_mv_matrix_loc;
        GLint model_to_screen_matrix_loc;
        GLint viewport_loc;
        GLint height_divided_by_top_minus_bottom_loc;
        // Point-Cloud-Parameter
        GLint scale_radius_loc;
        GLint max_radius_loc;
        GLint window_size_loc;
        GLint near_plane_loc;
        GLint far_plane_loc;
        GLint point_size_factor_loc;
        // Anzeige-Flags
        GLint show_normals_loc;
        GLint show_accuracy_loc;
        GLint show_output_sensitivity_loc;
        GLint channel_loc;
        GLint heatmap_enabled_loc;
        // Heatmap-Bereiche
        GLint heatmap_min_loc;
        GLint heatmap_max_loc;
        GLint heatmap_min_color_loc;
        GLint heatmap_max_color_loc;
        // Beleuchtung
        GLint use_material_color_loc;
        GLint material_diffuse_loc;
        GLint material_specular_loc;
        GLint ambient_light_color_loc;
        GLint point_light_color_loc;
    };
    PointProvShader m_point_prov_shader;

    struct LineShader {
        GLuint program;
        GLint in_color_location;
        GLint mvp_matrix_location;
    };
    LineShader m_line_shader;

    struct PclResource {
        GLuint program;
        GLuint vao;
    };


    struct BoxResource {
        GLuint vbo = 0;
        GLuint ibo = 0;
        GLuint vao = 0;
        GLuint program = 0;
        std::array<unsigned short, 24> idx = { 
            0, 1, 2, 3, 4, 5, 6, 7,
            0, 2, 1, 3, 4, 6, 5, 7,
            0, 4, 1, 5, 2, 6, 3, 7,
        };
    };

    struct PlaneResource {
        GLuint vbo{ 0 };
        GLuint ibo{ 0 };
        GLuint vao{ 0 };
        GLuint program{ 0 };
        std::array<unsigned short, 6> idx = {
            1,3,7,5,1,7
        };
    };

    struct CoordRecourse {
        GLuint vao = 0;
        GLuint vbo = 0;
        GLuint ibo = 0;
        GLuint program = 0;
        std::array<float, 12> vertices = {
            0.f,   0.f,   0.f,
            50.f,   0.f,   0.f,  // X-Achse
            0.f,  50.f,   0.f,  // Y-Achse
            0.f,   0.f,   50.f  // Z-Achse
        };
        std::array<unsigned short, 6> idx = {
            0, 1,
            0, 2,
            0, 3,
        };
    };

    struct FrustumResource {
        GLuint vao = 0;
        GLuint vbo = 0;
        GLuint ibo = 0;
        GLuint program = 0;
        std::array<float, 24> vertices;
        std::array<unsigned short, 24> idx = {
            0, 1, 2, 3, 4, 5, 6, 7,
            0, 2, 1, 3, 4, 6, 5, 7,
            0, 4, 1, 5, 2, 6, 3, 7,
        };
    };

    struct TextResource {
        GLuint vao{ 0 };
        GLuint vbo{ 0 };
        GLuint program{ 0 };
        GLuint atlas_texture{ 0 };
        std::string text;
        size_t num_vertices{ 0 };
    };

    // Resources
    PclResource m_pcl_resource;
    BoxResource m_box_resource;
    PlaneResource m_plane_resource;
    CoordRecourse m_coord_resource;
    FrustumResource m_frustum_resource;
    TextResource m_text_resource;

    

    // Matrizen
    scm::math::mat4d m_modelview_matrix;
    scm::math::mat4d m_projection_matrix;

    // Schism objects
    scm::gl::render_device_ptr      m_device;
    scm::gl::render_context_ptr     m_context;

    // Cameras
    lamure::ren::camera* m_scm_camera;
    osg::ref_ptr<osg::Camera>   m_osg_camera;
    osg::ref_ptr<osg::Camera>   m_hud_camera;

    // Geodes
    osg::ref_ptr<osg::Geode> m_init_geode;
    osg::ref_ptr<osg::Geode> m_pointcloud_geode;
    osg::ref_ptr<osg::Geode> m_boundingbox_geode;
    osg::ref_ptr<osg::Geode> m_frustum_geode;
    osg::ref_ptr<osg::Geode> m_coord_geode;
    osg::ref_ptr<osg::Geode> m_text_geode;

    // Stateset
    osg::ref_ptr<osg::StateSet> m_init_stateset;
    osg::ref_ptr<osg::StateSet> m_pointcloud_stateset;
    osg::ref_ptr<osg::StateSet> m_boundingbox_stateset;
    osg::ref_ptr<osg::StateSet> m_frustum_stateset;
    osg::ref_ptr<osg::StateSet> m_coord_stateset;
    osg::ref_ptr<osg::StateSet> m_text_stateset;

    // Geometry
    osg::ref_ptr<osg::Geometry> m_init_geometry;
    osg::ref_ptr<osg::Geometry> m_pointcloud_geometry;
    osg::ref_ptr<osg::Geometry> m_boundingbox_geometry;
    osg::ref_ptr<osg::Geometry> m_frustum_geometry;
    osg::ref_ptr<osg::Geometry> m_coord_geometry;

    // Framebuffers
    scm::gl::frame_buffer_ptr fbo;
    scm::gl::texture_2d_ptr fbo_color_buffer;
    scm::gl::texture_2d_ptr fbo_depth_buffer;
    scm::gl::frame_buffer_ptr pass1_fbo;
    scm::gl::frame_buffer_ptr pass2_fbo;
    scm::gl::frame_buffer_ptr pass3_fbo;
    scm::gl::texture_2d_ptr pass1_depth_buffer;
    scm::gl::texture_2d_ptr pass2_color_buffer;
    scm::gl::texture_2d_ptr pass2_normal_buffer;
    scm::gl::texture_2d_ptr pass2_view_space_pos_buffer;
    scm::gl::texture_2d_ptr pass2_depth_buffer;

    // Render states
    scm::gl::depth_stencil_state_ptr depth_state_disable;
    scm::gl::depth_stencil_state_ptr depth_state_less;
    scm::gl::depth_stencil_state_ptr depth_state_without_writing;
    scm::gl::rasterizer_state_ptr no_backface_culling_rasterizer_state;
    scm::gl::blend_state_ptr color_blending_state;
    scm::gl::blend_state_ptr color_no_blending_state;
    scm::gl::sampler_state_ptr filter_linear;
    scm::gl::sampler_state_ptr filter_nearest;
    scm::gl::sampler_state_ptr vt_filter_linear;
    scm::gl::sampler_state_ptr vt_filter_nearest;
    scm::gl::texture_2d_ptr bg_texture;

    // Shader sources
    std::string vis_point_vs_source;
    std::string vis_point_fs_source;
    std::string vis_point_prov_vs_source;
    std::string vis_point_prov_fs_source;
    std::string vis_surfel_vs_source;
    std::string vis_surfel_gs_source;
    std::string vis_surfel_fs_source;
    std::string vis_surfel_prov_vs_source;
    std::string vis_surfel_prov_fs_source;

    std::string vis_line_bb_vs_source;
    std::string vis_line_bb_fs_source;
    std::string vis_quad_vs_source;
    std::string vis_quad_fs_source;
    std::string vis_line_vs_source;
    std::string vis_line_fs_source;
    std::string vis_triangle_vs_source;
    std::string vis_triangle_fs_source;
    std::string vis_plane_vs_source;
    std::string vis_plane_fs_source;
    std::string vis_text_vs_source;
    std::string vis_text_fs_source;
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
    std::string vis_box_vs_source;
    std::string vis_box_gs_source;
    std::string vis_box_fs_source;

    std::string vis_xyz_vs_lighting_source;
    std::string vis_xyz_gs_lighting_source;
    std::string vis_xyz_fs_lighting_source;
    std::string vis_xyz_pass2_vs_lighting_source;
    std::string vis_xyz_pass2_gs_lighting_source;
    std::string vis_xyz_pass2_fs_lighting_source;
    std::string vis_xyz_pass3_vs_lighting_source;
    std::string vis_xyz_pass3_fs_lighting_source;

public:
    LamureRenderer(Lamure* lamure_plugin);
    ~LamureRenderer();

    void init();

    osg::ref_ptr<osg::Group> getGroup() { return m_group; }

    void initBoxResources();
    void initCoordResources();
    void initFrustumResources();
    void initLamureShader();
    void initSchismObjects();
    void initUniforms();
    void initFramebuffer();

    bool getRendering() { return m_rendering; };
    void setRendering(bool rendering) { m_rendering = rendering; };

    // Getters for private members
    lamure::ren::camera* getScmCamera() { return m_scm_camera; }
    osg::ref_ptr<osg::Camera> getOsgCamera() { return m_osg_camera; }

    scm::math::mat4d getModelviewMatrix() { return m_modelview_matrix; }
    scm::math::mat4d getProjextionMatrix() { return m_projection_matrix; }
    void setModelviewMatrix(scm::math::mat4d model_view_matrix) { m_modelview_matrix = model_view_matrix; }
    void setProjectionMatrix(scm::math::mat4d projection_matrix) { m_projection_matrix = projection_matrix; }

    osg::ref_ptr<osg::Geode> getPointcloudGeode() { return m_pointcloud_geode; }
    osg::ref_ptr<osg::Geode> getBoundingboxGeode() { return m_boundingbox_geode; }
    osg::ref_ptr<osg::Geode> getFrustumGeode() { return m_frustum_geode; }
    osg::ref_ptr<osg::Geode> getCoordGeode() { return m_coord_geode; }
    osg::ref_ptr<osg::Geode> getTextGeode() { return m_text_geode; }

    scm::gl::render_device_ptr getDevice() { return m_device; }
    scm::gl::render_context_ptr getContext() { return m_context; }

    const PointShader&      getPointShader()      const { return m_point_shader; }
    const SurfelShader&     getSurfelShader()     const { return m_surfel_shader; }
    const PointProvShader&  getPointProvShader()  const { return m_point_prov_shader; }
    const LineShader&       getLineShader()       const { return m_line_shader; }

    PclResource&      getPclResource()      { return m_pcl_resource; }
    BoxResource&      getBoxResource()      { return m_box_resource; }
    PlaneResource&    getPlaneResource()    { return m_plane_resource; }
    CoordRecourse&    getCoordResource()    { return m_coord_resource; }
    FrustumResource&  getFrustumResource()  { return m_frustum_resource; }
    TextResource&     getTextResource()     { return m_text_resource; }

};

#endif // _LAMURE_RENDERER_H