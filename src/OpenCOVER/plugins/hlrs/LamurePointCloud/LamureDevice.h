
// Copyright (c) 2012 Christopher Lux <christopherlux@gmail.com>
// Distributed under the Modified BSD License, see license.txt.

#ifndef LAMURE_DEVICE_H_INCLUDED
#define LAMURE_DEVICE_H_INCLUDED

#include <scm/gl_core/render_device/device.h>

#include <iosfwd>
#include <limits>
#include <list>
#include <set>
#include <utility>
#include <vector>

#include <boost/noncopyable.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>


#include <scm/config.h>
#include <scm/core/math.h>
#include <scm/core/memory.h>

#include <scm/gl_core/gl_core_fwd.h>
#include <scm/gl_core/data_formats.h>
#include <scm/gl_core/buffer_objects/buffer.h>
#include <scm/gl_core/shader_objects/shader_objects_fwd.h>
#include <scm/gl_core/shader_objects/shader_macro.h>
#include <scm/gl_core/state_objects/blend_state.h>
#include <scm/gl_core/state_objects/depth_stencil_state.h>
#include <scm/gl_core/state_objects/rasterizer_state.h>

#include <scm/core/platform/platform.h>
#include <scm/core/utilities/platform_warning_disable.h>

#include "LamureContext.h"

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <scm/core/numeric_types.h>
#include <scm/gl_core/render_device/render_device_fwd.h>
#include <scm/gl_core/shader_objects/shader_objects_fwd.h>
#include <scm/core/numeric_types.h>
#include "scm/gl_core/gl_core_fwd.h"



namespace gl {
    namespace opengl {
        class gl_core;
    }
}


class LamureDevice
{
public:

    struct device_capabilities {
        int             _max_vertex_attributes;
        int             _max_draw_buffers;
        int             _max_dual_source_draw_buffers;
        int             _max_texture_size;
        int             _max_texture_3d_size;
        int             _max_samples;
        int             _max_array_texture_layers;
        int             _max_depth_texture_samples;
        int             _max_color_texture_samples;
        int             _max_integer_samples;
        int             _max_texture_image_units;
        int             _max_texture_buffer_size;
        int             _max_frame_buffer_color_attachments;
        int             _max_vertex_uniform_blocks;
        int             _max_geometry_uniform_blocks;
        int             _max_fragment_uniform_blocks;
        int             _max_combined_uniform_blocks;
        int             _max_combined_vertex_uniform_components;
        int             _max_combined_geometry_uniform_components;
        int             _max_combined_fragment_uniform_components;
        int             _max_uniform_buffer_bindings;
        int             _max_uniform_block_size;
        int             _uniform_buffer_offset_alignment;
        int             _max_viewports;
        int             _max_transform_feedback_separate_attribs;
        int             _max_transform_feedback_buffers;
        int             _max_vertex_streams;
        int             _max_image_units;
        int             _max_vertex_atomic_counters;
        int             _max_geometry_atomic_counters;
        int             _max_fragment_atomic_counters;
        int             _max_combined_atomic_counters;
        int             _max_atomic_counter_buffer_bindings;
        int             _min_map_buffer_alignment;

        int             _num_program_binary_formats;
        boost::shared_array<int>   _program_binary_formats;

        int             _max_shader_storage_block_bindings;
        int             _max_shader_storage_block_size;
        scm::int64           _shader_storage_buffer_offset_alignment;
    }; 

protected:
    typedef boost::unordered_set<scm::gl::render_device_resource*>      resource_ptr_set;
    typedef boost::unordered_map<std::string, scm::gl::shader_macro>    shader_macro_map;

    typedef std::set<std::string>                                       string_set;
    typedef std::list<scm::gl::shader_ptr>                              shader_list;
    typedef std::vector<scm::gl::buffer_ptr>                            buffer_array;


////// methods ////////////////////////////////////////////////////////////////////////////////////
public:
    LamureDevice();
    virtual ~LamureDevice();

    const scm::gl::opengl::gl_core& opengl_api() const;

    scm::gl::render_context_ptr              main_context() const;
    //render_context_ptr              create_context();
    const device_capabilities&      capabilities() const;

    virtual void                    print_device_informations(std::ostream& os) const;
    const std::string               device_vendor() const;
    const std::string               device_renderer() const;
    const std::string               device_shader_compiler() const;
    const std::string               device_context_version() const;


    // buffer api /////////////////////////////////////////////////////////////////////////////////
public:
    scm::gl::buffer_ptr                     create_buffer(const scm::gl::buffer_desc& in_buffer_desc, const void* in_initial_data = 0);
    scm::gl::buffer_ptr                     create_buffer(scm::gl::buffer_binding in_binding, scm::gl::buffer_usage   in_usage, scm::size_t    in_size, const void* in_initial_data = 0);
    bool                                    resize_buffer(const scm::gl::buffer_ptr& in_buffer, scm::size_t in_size);

    scm::gl::vertex_array_ptr               create_vertex_array(const scm::gl::vertex_format& in_vert_fmt, const buffer_array& in_attrib_buffers, const scm::gl::program_ptr& in_program = scm::gl::program_ptr());

    scm::gl::transform_feedback_ptr         create_transform_feedback(const scm::gl::stream_output_setup& in_setup);

////// attributes /////////////////////////////////////////////////////////////////////////////////
protected:
    // device /////////////////////////////////////////////////////////////////////////////////////
    struct mutex_impl;
    boost::shared_ptr<mutex_impl>          _mutex_impl;

    // device /////////////////////////////////////////////////////////////////////////////////////
    boost::shared_ptr<scm::gl::opengl::gl_core>     _opengl_api_core;
    scm::gl::render_context_ptr              _main_context;

    // shader api /////////////////////////////////////////////////////////////////////////////////
    shader_macro_map                _default_macro_defines;
    string_set                      _default_include_paths;

    device_capabilities             _capabilities;
    resource_ptr_set                _registered_resources;


protected:
    typedef boost::unordered_set<scm::gl::render_device_resource*>   resource_ptr_set;

    typedef boost::unordered_map<std::string, scm::gl::shader_macro> shader_macro_map;
    typedef std::set<std::string>                           string_set;

    typedef std::list<scm::gl::shader_ptr>                           shader_list;

    typedef std::vector<scm::gl::buffer_ptr>                         buffer_array;


protected:
    void                            init_capabilities();

    void                            register_resource(scm::gl::render_device_resource* res_ptr);
    void                            release_resource(scm::gl::render_device_resource* res_ptr);

    // buffer api /////////////////////////////////////////////////////////////////////////////////
public:
    scm::gl::buffer_ptr             create_buffer(const scm::gl::buffer_desc& in_buffer_desc, const void* in_initial_data = 0);
    scm::gl::buffer_ptr             create_buffer(scm::gl::buffer_binding in_binding, scm::gl::buffer_usage   in_usage, scm::size_t    in_size, const void* in_initial_data = 0);
    bool                            resize_buffer(const scm::gl::buffer_ptr& in_buffer, scm::size_t in_size);

    scm::gl::vertex_array_ptr                create_vertex_array(const scm::gl::vertex_format& in_vert_fmt, const buffer_array& in_attrib_buffers, const scm::gl::program_ptr& in_program = scm::gl::program_ptr());
    scm::gl::transform_feedback_ptr          create_transform_feedback(const scm::gl::stream_output_setup& in_setup);

    // shader api /////////////////////////////////////////////////////////////////////////////////
public:
    bool                            add_include_files(const std::string& in_path,
        const std::string& in_glsl_root_path = std::string("/"),
        const std::string& in_file_extensions = std::string(".glsl .glslh"),
        bool               in_scan_subdirectories = true);
    bool                            add_include_string(const std::string& in_path, const std::string& in_source_string);

    void                            add_macro_define(const std::string& in_name, const std::string& in_value);
    void                            add_macro_define(const scm::gl::shader_macro& in_macro);
    void                            add_macro_defines(const scm::gl::shader_macro_array& in_macros);

    scm::gl::shader_ptr create_shader(
        scm::gl::shader_stage in_stage,
        const std::string& in_source,
        const std::string& in_source_name = "");

    scm::gl::shader_ptr create_shader(
        scm::gl::shader_stage in_stage,
        const std::string& in_source,
        const scm::gl::shader_macro_array& in_macros,
        const std::string& in_source_name = "");

    scm::gl::shader_ptr create_shader(
        scm::gl::shader_stage in_stage,
        const std::string& in_source,
        const scm::gl::shader_include_path_list& in_inc_paths,
        const std::string& in_source_name = "");

    scm::gl::shader_ptr create_shader(
        scm::gl::shader_stage in_stage,
        const std::string& in_source,
        const scm::gl::shader_macro_array& in_macros,
        const scm::gl::shader_include_path_list& in_inc_paths,
        const std::string& in_source_name = "");

    scm::gl::shader_ptr create_shader_from_file(
        scm::gl::shader_stage in_stage,
        const std::string& in_file_name);

    scm::gl::shader_ptr create_shader_from_file(
        scm::gl::shader_stage in_stage,
        const std::string& in_source,
        const scm::gl::shader_macro_array& in_macros);

    scm::gl::shader_ptr create_shader_from_file(
        scm::gl::shader_stage in_stage,
        const std::string& in_source,
        const scm::gl::shader_include_path_list& in_inc_paths);

    scm::gl::shader_ptr create_shader_from_file(
        scm::gl::shader_stage in_stage,
        const std::string& in_source,
        const scm::gl::shader_macro_array& in_macros,
        const scm::gl::shader_include_path_list& in_inc_paths);

    scm::gl::program_ptr create_program(
        const shader_list& in_shaders,
        const std::string& in_program_name = "");

    scm::gl::program_ptr create_program(
        const shader_list& in_shaders,
        const scm::gl::stream_capture_array& in_capture,
        bool in_rasterization_discard = false,
        const std::string& in_program_name = "");

protected:
    bool add_include_string_internal(const std::string& in_path, const std::string& in_source_string, bool lock_thread);

    // texture api ////////////////////////////////////////////////////////////////////////////////
public:
    scm::gl::texture_1d_ptr                  create_texture_1d(const scm::gl::texture_1d_desc& in_desc);
    scm::gl::texture_1d_ptr                  create_texture_1d(const scm::gl::texture_1d_desc& in_desc,
        const scm::gl::data_format         in_initial_data_format,
        const std::vector<void*>& in_initial_mip_level_data);
    scm::gl::texture_1d_ptr                  create_texture_1d(const unsigned      in_size,
        const scm::gl::data_format   in_format,
        const unsigned      in_mip_levels = 1,
        const unsigned      in_array_layers = 1);
    scm::gl::texture_1d_ptr                  create_texture_1d(const unsigned            in_size,
        const scm::gl::data_format         in_format,
        const unsigned            in_mip_levels,
        const unsigned            in_array_layers,
        const scm::gl::data_format         in_initial_data_format,
        const std::vector<void*>& in_initial_mip_level_data);
    scm::gl::texture_1d_ptr                  create_texture_1d(const scm::gl::texture_1d_ptr& in_orig_texture,
        const scm::gl::data_format         in_format,
        const scm::math::vec2ui& in_mip_range,
        const scm::math::vec2ui& in_layer_range);

    scm::gl::texture_2d_ptr                  create_texture_2d(const scm::gl::texture_2d_desc& in_desc);
    scm::gl::texture_2d_ptr                  create_texture_2d(const scm::gl::texture_2d_desc& in_desc,
        const scm::gl::data_format         in_initial_data_format,
        const std::vector<void*>& in_initial_mip_level_data);
    scm::gl::texture_2d_ptr                  create_texture_2d(const scm::math::vec2ui& in_size,
        const scm::gl::data_format   in_format,
        const unsigned      in_mip_levels = 1,
        const unsigned      in_array_layers = 1,
        const unsigned      in_samples = 1);
    scm::gl::texture_2d_ptr                  create_texture_2d(const scm::math::vec2ui& in_size,
        const scm::gl::data_format         in_format,
        const unsigned            in_mip_levels,
        const unsigned            in_array_layers,
        const unsigned            in_samples,
        const scm::gl::data_format         in_initial_data_format,
        const std::vector<void*>& in_initial_mip_level_data);
    scm::gl::texture_2d_ptr                  create_texture_2d(const scm::gl::texture_2d_ptr& in_orig_texture,
        const scm::gl::data_format         in_format,
        const scm::math::vec2ui& in_mip_range,
        const scm::math::vec2ui& in_layer_range);

    scm::gl::texture_3d_ptr                  create_texture_3d(const scm::gl::texture_3d_desc& in_desc);
    scm::gl::texture_3d_ptr                  create_texture_3d(const scm::gl::texture_3d_desc& in_desc,
        const scm::gl::data_format         in_initial_data_format,
        const std::vector<void*>& in_initial_mip_level_data);
    scm::gl::texture_3d_ptr                  create_texture_3d(const scm::math::vec3ui& in_size,
        const scm::gl::data_format   in_format,
        const unsigned      in_mip_levels = 1);
    scm::gl::texture_3d_ptr                  create_texture_3d(const scm::math::vec3ui& in_size,
        const scm::gl::data_format         in_format,
        const unsigned            in_mip_levels,
        const scm::gl::data_format         in_initial_data_format,
        const std::vector<void*>& in_initial_mip_level_data);
    scm::gl::texture_3d_ptr                  create_texture_3d(const scm::gl::texture_3d_ptr& in_orig_texture,
        const scm::gl::data_format         in_format,
        const scm::math::vec2ui& in_mip_range);

    scm::gl::texture_cube_ptr                create_texture_cube(const scm::gl::texture_cube_desc& in_desc);
    scm::gl::texture_cube_ptr                create_texture_cube(const scm::gl::texture_cube_desc& in_desc,
        const scm::gl::data_format          in_initial_data_format,
        const std::vector<void*>& in_initial_mip_level_data_px,
        const std::vector<void*>& in_initial_mip_level_data_nx,
        const std::vector<void*>& in_initial_mip_level_data_py,
        const std::vector<void*>& in_initial_mip_level_data_ny,
        const std::vector<void*>& in_initial_mip_level_data_pz,
        const std::vector<void*>& in_initial_mip_level_data_nz);
    scm::gl::texture_cube_ptr                create_texture_cube(const scm::math::vec2ui& in_size,
        const scm::gl::data_format   in_format,
        const unsigned      in_mip_levels = 1);
    scm::gl::texture_cube_ptr                create_texture_cube(const scm::math::vec2ui& in_size,
        const scm::gl::data_format          in_format,
        const unsigned                      in_mip_levels,
        const scm::gl::data_format          in_initial_data_format,
        const std::vector<void*>& in_initial_mip_level_data_px,
        const std::vector<void*>& in_initial_mip_level_data_nx,
        const std::vector<void*>& in_initial_mip_level_data_py,
        const std::vector<void*>& in_initial_mip_level_data_ny,
        const std::vector<void*>& in_initial_mip_level_data_pz,
        const std::vector<void*>& in_initial_mip_level_data_nz);

    scm::gl::texture_buffer_ptr              create_texture_buffer(const scm::gl::texture_buffer_desc& in_desc);
    scm::gl::texture_buffer_ptr              create_texture_buffer(const scm::gl::data_format   in_format,
        const scm::gl::buffer_ptr& in_buffer);
    scm::gl::texture_buffer_ptr              create_texture_buffer(const scm::gl::data_format   in_format,
        scm::gl::buffer_usage        in_buffer_usage,
        scm::size_t         in_buffer_size,
        const void* in_buffer_initial_data = 0);

    scm::gl::texture_handle_ptr              create_resident_handle(const scm::gl::texture_ptr& in_texture,
        const scm::gl::sampler_state_ptr& in_sampler);

    scm::gl::sampler_state_ptr               create_sampler_state(const scm::gl::sampler_state_desc& in_desc);
    scm::gl::sampler_state_ptr               create_sampler_state(scm::gl::texture_filter_mode  in_filter1,
        scm::gl::texture_wrap_mode    in_wrap,
        unsigned             in_max_anisotropy = 1,
        float                in_min_lod = -(std::numeric_limits<float>::max)(),
        float                in_max_lod = (std::numeric_limits<float>::max)(),
        float                in_lod_bias = 0.0f,
        scm::gl::compare_func         in_compare_func = scm::gl::COMPARISON_LESS_EQUAL,
        scm::gl::texture_compare_mode in_compare_mode = scm::gl::TEXCOMPARE_NONE);
    scm::gl::sampler_state_ptr               create_sampler_state(scm::gl::texture_filter_mode  in_filter,
        scm::gl::texture_wrap_mode    in_wrap_s,
        scm::gl::texture_wrap_mode    in_wrap_t,
        scm::gl::texture_wrap_mode    in_wrap_r,
        unsigned             in_max_anisotropy = 1,
        float                in_min_lod = -(std::numeric_limits<float>::max)(),
        float                in_max_lod = (std::numeric_limits<float>::max)(),
        float                in_lod_bias = 0.0f,
        scm::gl::compare_func         in_compare_func = scm::gl::COMPARISON_LESS_EQUAL,
        scm::gl::texture_compare_mode in_compare_mode = scm::gl::TEXCOMPARE_NONE);

    // frame buffer api ///////////////////////////////////////////////////////////////////////////
    scm::gl::render_buffer_ptr               create_render_buffer(const scm::gl::render_buffer_desc& in_desc);
    scm::gl::render_buffer_ptr               create_render_buffer(const scm::math::vec2ui& in_size,
        const scm::gl::data_format   in_format,
        const unsigned      in_samples = 1);
    scm::gl::frame_buffer_ptr                create_frame_buffer();


    // state api //////////////////////////////////////////////////////////////////////////////////
public:
    scm::gl::depth_stencil_state_ptr         create_depth_stencil_state(const scm::gl::depth_stencil_state_desc& in_desc);
    scm::gl::depth_stencil_state_ptr         create_depth_stencil_state(bool in_depth_test, bool in_depth_mask = true, scm::gl::compare_func in_depth_func = scm::gl::COMPARISON_LESS,
        bool in_stencil_test = false, unsigned in_stencil_rmask = ~0u, unsigned in_stencil_wmask = ~0u,
        scm::gl::stencil_ops in_stencil_ops = scm::gl::stencil_ops());
    scm::gl::depth_stencil_state_ptr         create_depth_stencil_state(bool in_depth_test, bool in_depth_mask, scm::gl::compare_func in_depth_func,
        bool in_stencil_test, unsigned in_stencil_rmask, unsigned in_stencil_wmask,
        scm::gl::stencil_ops in_stencil_front_ops, scm::gl::stencil_ops in_stencil_back_ops);

    scm::gl::rasterizer_state_ptr            create_rasterizer_state(const scm::gl::rasterizer_state_desc& in_desc);
    scm::gl::rasterizer_state_ptr            create_rasterizer_state(scm::gl::fill_mode in_fmode, scm::gl::cull_mode in_cmode = scm::gl::CULL_BACK, scm::gl::polygon_orientation in_fface = scm::gl::ORIENT_CCW,
        bool in_msample = false, bool in_sshading = false, float in_min_sshading = 0.0f,
        bool in_sctest = false, bool in_smlines = false,
        const scm::gl::point_raster_state& in_point_state = scm::gl::point_raster_state());

    scm::gl::blend_state_ptr                 create_blend_state(const scm::gl::blend_state_desc& in_desc);
    scm::gl::blend_state_ptr                 create_blend_state(bool in_enabled,
        scm::gl::blend_func in_src_rgb_func, scm::gl::blend_func in_dst_rgb_func,
        scm::gl::blend_func in_src_alpha_func, scm::gl::blend_func in_dst_alpha_func,
        scm::gl::blend_equation  in_rgb_equation = scm::gl::EQ_FUNC_ADD, scm::gl::blend_equation in_alpha_equation = scm::gl::EQ_FUNC_ADD,
        unsigned in_write_mask = scm::gl::COLOR_ALL, bool in_alpha_to_coverage = false);
    scm::gl::blend_state_ptr                 create_blend_state(const scm::gl::blend_ops_array& in_blend_ops, bool in_alpha_to_coverage = false);

    // query api //////////////////////////////////////////////////////////////////////////////////
public:
    scm::gl::timer_query_ptr                 create_timer_query();
    scm::gl::occlusion_query_ptr             create_occlusion_query(const scm::gl::occlusion_query_mode in_oq_mode);
    scm::gl::transform_feedback_statistics_query_ptr create_transform_feedback_statistics_query(int stream = 0);




protected:
    bool                            add_include_string_internal(const std::string& in_path,
        const std::string& in_source_string,
        bool         lock_thread);


}; // class LamureDevice


#endif // CO_SCM_GL_CORE_DEVICE_H_INCLUDED
