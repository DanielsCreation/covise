
// Copyright (c) 2012 Christopher Lux <christopherlux@gmail.com>
// Distributed under the Modified BSD License, see license.txt.

#include "LamureDevice.h"

#include <algorithm>
#include <exception>
#include <stdexcept>
#include <sstream>

#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>

#include <scm/core/io/tools.h>
#include <scm/core/io/iomanip.h>
#include <scm/core/log/logger_state.h>
#include <scm/core/utilities/foreach.h>

#include <scm/gl_core/config.h>
#include <scm/gl_core/log.h>
#include <scm/gl_core/buffer_objects.h>
#include <scm/gl_core/frame_buffer_objects.h>
#include <scm/gl_core/query_objects.h>
#include <scm/gl_core/render_device/context.h>
#include <scm/gl_core/render_device/opengl/gl_core.h>
#include <scm/gl_core/render_device/opengl/util/assert.h>
#include <scm/gl_core/render_device/opengl/util/error_helper.h>
#include <scm/gl_core/shader_objects/program.h>
#include <scm/gl_core/shader_objects/shader.h>
#include <scm/gl_core/shader_objects/stream_capture.h>
#include <scm/gl_core/state_objects/depth_stencil_state.h>
#include <scm/gl_core/state_objects/rasterizer_state.h>
#include <scm/gl_core/state_objects/sampler_state.h>
#include <scm/gl_core/texture_objects.h>


#define SCM_GL_CORE_OPENGL_CORE_VERSION 200
#define SCM_GL_CORE_OPENGL_CORE_VERSION_410 410

using namespace scm;
using namespace scm::gl;


struct LamureDevice::mutex_impl
{
    boost::mutex    _mutex;
};

LamureDevice::LamureDevice() : _mutex_impl(new mutex_impl)
{
    _opengl_api_core.reset(new scm::gl::opengl::gl_core());

    if (!_opengl_api_core->initialize()) {
        std::ostringstream s;
        s << "render_device::render_device(): error initializing gl core.";
        scm::gl::glerr() << scm::log::fatal << s.str() << scm::log::end;
        throw std::runtime_error(s.str());
    }
    unsigned req_version_major = SCM_GL_CORE_OPENGL_CORE_VERSION / 100;
    unsigned req_version_minor = (SCM_GL_CORE_OPENGL_CORE_VERSION - req_version_major * 100) / 10;


    init_capabilities();
    try {
        _main_context.reset(new LamureContext(*this));
        _main_context->apply();
    }
        catch (const std::exception& e) {
        std::ostringstream s;
        s << "render_device::render_device(): error creating main_context (evoking error: "
            << e.what()
            << ").";
        throw std::runtime_error(s.str());
    }
}

const scm::gl::opengl::gl_core&
LamureDevice::opengl_api() const
{
    return *_opengl_api_core;
}


void LamureDevice::init_capabilities() {


    const scm::gl::opengl::gl_core& glcore = opengl_api();

    glcore.glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, &_capabilities._max_vertex_attributes);
    glcore.glGetIntegerv(GL_MAX_DRAW_BUFFERS, &_capabilities._max_draw_buffers);
    glcore.glGetIntegerv(GL_MAX_DUAL_SOURCE_DRAW_BUFFERS, &_capabilities._max_dual_source_draw_buffers);

    assert(_capabilities._max_vertex_attributes > 0);
    assert(_capabilities._max_draw_buffers > 0);
    assert(_capabilities._max_dual_source_draw_buffers > 0);

    glcore.glGetIntegerv(GL_MAX_TEXTURE_SIZE, &_capabilities._max_texture_size);
    glcore.glGetIntegerv(GL_MAX_3D_TEXTURE_SIZE, &_capabilities._max_texture_3d_size);
    glcore.glGetIntegerv(GL_MAX_ARRAY_TEXTURE_LAYERS, &_capabilities._max_array_texture_layers);
    glcore.glGetIntegerv(GL_MAX_SAMPLES, &_capabilities._max_samples);
    glcore.glGetIntegerv(GL_MAX_DEPTH_TEXTURE_SAMPLES, &_capabilities._max_depth_texture_samples);
    glcore.glGetIntegerv(GL_MAX_COLOR_TEXTURE_SAMPLES, &_capabilities._max_color_texture_samples);
    glcore.glGetIntegerv(GL_MAX_INTEGER_SAMPLES, &_capabilities._max_integer_samples);
    glcore.glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &_capabilities._max_texture_image_units);
    glcore.glGetIntegerv(GL_MAX_TEXTURE_BUFFER_SIZE, &_capabilities._max_texture_buffer_size);
    glcore.glGetIntegerv(GL_MAX_COLOR_ATTACHMENTS, &_capabilities._max_frame_buffer_color_attachments);

    assert(_capabilities._max_texture_size > 0);
    assert(_capabilities._max_texture_3d_size > 0);
    assert(_capabilities._max_array_texture_layers > 0);
    assert(_capabilities._max_samples > 0);
    assert(_capabilities._max_depth_texture_samples > 0);
    assert(_capabilities._max_color_texture_samples > 0);
    assert(_capabilities._max_integer_samples > 0);
    assert(_capabilities._max_texture_image_units > 0);
    assert(_capabilities._max_texture_buffer_size > 0);
    assert(_capabilities._max_frame_buffer_color_attachments > 0);

    glcore.glGetIntegerv(GL_MAX_VERTEX_UNIFORM_BLOCKS, &_capabilities._max_vertex_uniform_blocks);
    glcore.glGetIntegerv(GL_MAX_GEOMETRY_UNIFORM_BLOCKS, &_capabilities._max_geometry_uniform_blocks);
    glcore.glGetIntegerv(GL_MAX_FRAGMENT_UNIFORM_BLOCKS, &_capabilities._max_fragment_uniform_blocks);
    glcore.glGetIntegerv(GL_MAX_COMBINED_UNIFORM_BLOCKS, &_capabilities._max_combined_uniform_blocks);
    glcore.glGetIntegerv(GL_MAX_COMBINED_VERTEX_UNIFORM_COMPONENTS, &_capabilities._max_combined_vertex_uniform_components);
    glcore.glGetIntegerv(GL_MAX_COMBINED_GEOMETRY_UNIFORM_COMPONENTS, &_capabilities._max_combined_geometry_uniform_components);
    glcore.glGetIntegerv(GL_MAX_COMBINED_FRAGMENT_UNIFORM_COMPONENTS, &_capabilities._max_combined_fragment_uniform_components);
    glcore.glGetIntegerv(GL_MAX_UNIFORM_BUFFER_BINDINGS, &_capabilities._max_uniform_buffer_bindings);
    glcore.glGetIntegerv(GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT, &_capabilities._uniform_buffer_offset_alignment);
    glcore.glGetIntegerv(GL_MAX_UNIFORM_BLOCK_SIZE, &_capabilities._max_uniform_block_size);

    assert(_capabilities._max_vertex_uniform_blocks > 0);
    assert(_capabilities._max_geometry_uniform_blocks > 0);
    assert(_capabilities._max_fragment_uniform_blocks > 0);
    assert(_capabilities._max_combined_uniform_blocks > 0);
    assert(_capabilities._max_combined_vertex_uniform_components > 0);
    assert(_capabilities._max_combined_geometry_uniform_components > 0);
    assert(_capabilities._max_combined_fragment_uniform_components > 0);
    assert(_capabilities._max_uniform_buffer_bindings > 0);
    assert(_capabilities._uniform_buffer_offset_alignment > 0);
    assert(_capabilities._max_uniform_block_size > 0);

    if (SCM_GL_CORE_OPENGL_CORE_VERSION >= SCM_GL_CORE_OPENGL_CORE_VERSION_410) {
        glcore.glGetIntegerv(GL_MAX_VIEWPORTS, &_capabilities._max_viewports);
    }
    else {
        _capabilities._max_viewports = 1;
    }

    glcore.glGetIntegerv(GL_MAX_TRANSFORM_FEEDBACK_SEPARATE_ATTRIBS, &_capabilities._max_transform_feedback_separate_attribs);
    //if (SCM_GL_CORE_OPENGL_CORE_VERSION >= SCM_GL_CORE_OPENGL_CORE_VERSION_400) {
    //    glcore.glGetIntegerv(GL_MAX_TRANSFORM_FEEDBACK_BUFFERS, &_capabilities._max_transform_feedback_buffers);
    //    glcore.glGetIntegerv(GL_MAX_VERTEX_STREAMS, &_capabilities._max_vertex_streams);
    //}
    //else {
    //    _capabilities._max_transform_feedback_buffers = _capabilities._max_transform_feedback_separate_attribs;
    //    _capabilities._max_vertex_streams = 1;
    //}
    _capabilities._max_transform_feedback_buffers = _capabilities._max_transform_feedback_separate_attribs;
    _capabilities._max_vertex_streams = 1;
    assert(_capabilities._max_transform_feedback_separate_attribs > 0);
    assert(_capabilities._max_transform_feedback_buffers > 0);
    assert(_capabilities._max_vertex_streams > 0);

    /*if (SCM_GL_CORE_OPENGL_CORE_VERSION >= SCM_GL_CORE_OPENGL_CORE_VERSION_420) {
        glcore.glGetIntegerv(GL_MAX_IMAGE_UNITS, &_capabilities._max_image_units);
    }
    else if (glcore.extension_EXT_shader_image_load_store) {
        glcore.glGetIntegerv(GL_MAX_IMAGE_UNITS_EXT, &_capabilities._max_image_units);
    }
    else {
        _capabilities._max_image_units = 0;
    }*/
    _capabilities._max_image_units = 0;

    //if (SCM_GL_CORE_OPENGL_CORE_VERSION >= SCM_GL_CORE_OPENGL_CORE_VERSION_420) {
    //    glcore.glGetIntegerv(GL_MAX_VERTEX_ATOMIC_COUNTERS, &_capabilities._max_vertex_atomic_counters);
    //    glcore.glGetIntegerv(GL_MAX_FRAGMENT_ATOMIC_COUNTERS, &_capabilities._max_geometry_atomic_counters);
    //    glcore.glGetIntegerv(GL_MAX_GEOMETRY_ATOMIC_COUNTERS, &_capabilities._max_fragment_atomic_counters);
    //    glcore.glGetIntegerv(GL_MAX_COMBINED_ATOMIC_COUNTERS, &_capabilities._max_combined_atomic_counters);
    //    glcore.glGetIntegerv(GL_MAX_ATOMIC_COUNTER_BUFFER_BINDINGS, &_capabilities._max_atomic_counter_buffer_bindings);

    //    assert(_capabilities._max_vertex_atomic_counters >= 0);
    //    assert(_capabilities._max_geometry_atomic_counters >= 0);
    //    assert(_capabilities._max_fragment_atomic_counters > 0);
    //    assert(_capabilities._max_combined_atomic_counters > 0);
    //    assert(_capabilities._max_atomic_counter_buffer_bindings > 0);
    //}
    //else {
    //    _capabilities._max_vertex_atomic_counters = 0;
    //    _capabilities._max_geometry_atomic_counters = 0;
    //    _capabilities._max_fragment_atomic_counters = 0;
    //    _capabilities._max_combined_atomic_counters = 0;
    //    _capabilities._max_atomic_counter_buffer_bindings = 0;
    //}
    _capabilities._max_vertex_atomic_counters = 0;
    _capabilities._max_geometry_atomic_counters = 0;
    _capabilities._max_fragment_atomic_counters = 0;
    _capabilities._max_combined_atomic_counters = 0;
    _capabilities._max_atomic_counter_buffer_bindings = 0;

    //if (SCM_GL_CORE_OPENGL_CORE_VERSION >= SCM_GL_CORE_OPENGL_CORE_VERSION_420
    //    || glcore.extension_ARB_map_buffer_alignment) {
    //    glcore.glGetIntegerv(GL_MIN_MAP_BUFFER_ALIGNMENT, &_capabilities._min_map_buffer_alignment);
    //}
    //else {
    //    _capabilities._min_map_buffer_alignment = 1;
    //}
    _capabilities._min_map_buffer_alignment = 1;

    //if (SCM_GL_CORE_OPENGL_CORE_VERSION >= SCM_GL_CORE_OPENGL_CORE_VERSION_410) {
    //    glcore.glGetIntegerv(GL_NUM_PROGRAM_BINARY_FORMATS, &_capabilities._num_program_binary_formats);
    //    if (_capabilities._num_program_binary_formats > 0) {
    //        _capabilities._program_binary_formats.reset(new int[_capabilities._num_program_binary_formats]);
    //        glcore.glGetIntegerv(GL_PROGRAM_BINARY_FORMATS, _capabilities._program_binary_formats.get());
    //    }
    //}
    //else {
    //    _capabilities._num_program_binary_formats = 0;
    //}
    _capabilities._num_program_binary_formats = 0;

    //if (SCM_GL_CORE_OPENGL_CORE_VERSION >= SCM_GL_CORE_OPENGL_CORE_VERSION_430) {
    //    glcore.glGetIntegerv(GL_MAX_SHADER_STORAGE_BUFFER_BINDINGS, &_capabilities._max_shader_storage_block_bindings);
    //    glcore.glGetIntegerv(GL_MAX_SHADER_STORAGE_BLOCK_SIZE, &_capabilities._max_shader_storage_block_size);
    //    glcore.glGetInteger64v(GL_SHADER_STORAGE_BUFFER_OFFSET_ALIGNMENT, &_capabilities._shader_storage_buffer_offset_alignment);
    //}
    //else {
    //    _capabilities._max_shader_storage_block_bindings = 0;
    //    _capabilities._max_shader_storage_block_size = 0;
    //    _capabilities._shader_storage_buffer_offset_alignment = 1;
    //}
    _capabilities._max_shader_storage_block_bindings = 0;
    _capabilities._max_shader_storage_block_size = 0;
    _capabilities._shader_storage_buffer_offset_alignment = 1;

    scm::log::logger_format_saver ofs(scm::gl::glout().associated_logger());
    scm::gl::glout() << "render_device::init_capabilities(): OpenGL capabilities"
        << scm::log::indent;

    scm::gl::glout() << "general: " << scm::log::nline
        << scm::log::indent
        << "MAX_VERTEX_ATTRIBS                          " << _capabilities._max_vertex_attributes << scm::log::nline
        << "MAX_DRAW_BUFFERS                            " << _capabilities._max_draw_buffers << scm::log::nline
        << "MAX_DUAL_SOURCE_DRAW_BUFFERS                " << _capabilities._max_dual_source_draw_buffers << scm::log::nline
        << "MAX_TEXTURE_SIZE                            " << _capabilities._max_texture_size << scm::log::nline
        << "MAX_3D_TEXTURE_SIZE                         " << _capabilities._max_texture_3d_size << scm::log::nline
        << "MAX_ARRAY_TEXTURE_LAYERS                    " << _capabilities._max_array_texture_layers << scm::log::nline
        << "MAX_SAMPLES                                 " << _capabilities._max_samples << scm::log::nline
        << "MAX_DEPTH_TEXTURE_SAMPLES                   " << _capabilities._max_depth_texture_samples << scm::log::nline
        << "MAX_COLOR_TEXTURE_SAMPLES                   " << _capabilities._max_color_texture_samples << scm::log::nline
        << "MAX_INTEGER_SAMPLES                         " << _capabilities._max_integer_samples << scm::log::nline
        << "MAX_TEXTURE_IMAGE_UNITS                     " << _capabilities._max_texture_image_units << scm::log::nline
        << "MAX_TEXTURE_BUFFER_SIZE                     " << _capabilities._max_texture_buffer_size << scm::log::nline
        << "MAX_COLOR_ATTACHMENTS                       " << _capabilities._max_frame_buffer_color_attachments << scm::log::nline
        << "MAX_VIEWPORTS                               " << _capabilities._max_viewports
        << scm::log::outdent;

    scm::gl::glout() << "uniform blocks: " << scm::log::nline
        << scm::log::indent
        << "MAX_UNIFORM_BLOCK_SIZE                      " << scm::io::data_size(_capabilities._max_uniform_block_size) << scm::log::nline
        << "MAX_VERTEX_UNIFORM_BLOCKS                   " << _capabilities._max_vertex_uniform_blocks << scm::log::nline
        << "MAX_GEOMETRY_UNIFORM_BLOCKS                 " << _capabilities._max_geometry_uniform_blocks << scm::log::nline
        << "MAX_FRAGMENT_UNIFORM_BLOCKS                 " << _capabilities._max_fragment_uniform_blocks << scm::log::nline
        << "MAX_COMBINED_UNIFORM_BLOCKS                 " << _capabilities._max_combined_uniform_blocks << scm::log::nline
        << "MAX_COMBINED_VERTEX_UNIFORM_COMPONENTS      " << _capabilities._max_combined_vertex_uniform_components << scm::log::nline
        << "MAX_COMBINED_GEOMETRY_UNIFORM_COMPONENTS    " << _capabilities._max_combined_geometry_uniform_components << scm::log::nline
        << "MAX_COMBINED_FRAGMENT_UNIFORM_COMPONENTS    " << _capabilities._max_combined_fragment_uniform_components << scm::log::nline
        << "MAX_UNIFORM_BUFFER_BINDINGS                 " << _capabilities._max_uniform_buffer_bindings << scm::log::nline
        << "UNIFORM_BUFFER_OFFSET_ALIGNMENT             " << _capabilities._uniform_buffer_offset_alignment
        << scm::log::outdent;

    scm::gl::glout() << "transform feedback: " << scm::log::nline
        << scm::log::indent
        << "MAX_TRANSFORM_FEEDBACK_SEPARATE_ATTRIBS     " << _capabilities._max_transform_feedback_separate_attribs << scm::log::nline
        << "MAX_TRANSFORM_FEEDBACK_BUFFERS              " << _capabilities._max_transform_feedback_buffers << scm::log::nline
        << "MAX_VERTEX_STREAMS                          " << _capabilities._max_vertex_streams
        << scm::log::outdent;

    scm::gl::glout() << "image load/store: " << scm::log::nline
        << scm::log::indent
        << "MAX_IMAGE_UNITS                             " << _capabilities._max_image_units
        << scm::log::outdent;

    scm::gl::glout() << "atomic counters: " << scm::log::nline
        << scm::log::indent
        << "MAX_VERTEX_ATOMIC_COUNTERS                  " << _capabilities._max_vertex_atomic_counters << scm::log::nline
        << "MAX_FRAGMENT_ATOMIC_COUNTERS                " << _capabilities._max_geometry_atomic_counters << scm::log::nline
        << "MAX_GEOMETRY_ATOMIC_COUNTERS                " << _capabilities._max_fragment_atomic_counters << scm::log::nline
        << "MAX_COMBINED_ATOMIC_COUNTERS                " << _capabilities._max_combined_atomic_counters << scm::log::nline
        << "MAX_ATOMIC_COUNTER_BUFFER_BINDINGS          " << _capabilities._max_atomic_counter_buffer_bindings
        << scm::log::outdent;

    scm::gl::glout() << "map buffer alignment: " << scm::log::nline
        << scm::log::indent
        << "MIN_MAP_BUFFER_ALIGNMENT                    " << _capabilities._min_map_buffer_alignment
        << scm::log::outdent;

    scm::gl::glout() << "shader storage buffers: " << scm::log::nline
        << scm::log::indent
        << "MAX_SHADER_STORAGE_BUFFER_BINDINGS          " << _capabilities._max_shader_storage_block_bindings << scm::log::nline
        << "MAX_SHADER_STORAGE_BLOCK_SIZE               " << scm::io::data_size(_capabilities._max_shader_storage_block_size) << scm::log::nline
        << "SHADER_STORAGE_BUFFER_OFFSET_ALIGNMENT      " << _capabilities._shader_storage_buffer_offset_alignment
        << scm::log::outdent;

    std::stringstream pbf;
    pbf << "(";
    if (0 < _capabilities._num_program_binary_formats) {
        for (int f = 0; f < _capabilities._num_program_binary_formats; ++f) {
            pbf << std::hex << "0x" << _capabilities._program_binary_formats[f];
            if (f < _capabilities._num_program_binary_formats - 1) {
                pbf << ", ";
            }
        }
    }
    else {
        pbf << "N/A";
    }
    pbf << ")";

    scm::gl::glout() << "program binary formats: " << scm::log::nline
        << scm::log::indent
        << "GL_NUM_PROGRAM_BINARY_FORMATS               " << _capabilities._num_program_binary_formats << scm::log::nline
        << "GL_PROGRAM_BINARY_FORMATS                   " << pbf.str() << scm::log::nline
        << scm::log::outdent;

    //std::cout << "GL_MAX_IMAGE_UNITS_EXT " << _capabilities._max_image_units << std::endl;
}



// texture api ////////////////////////////////////////////////////////////////////////////////////
texture_1d_ptr
LamureDevice::create_texture_1d(const texture_1d_desc& in_desc)
{
    texture_1d_ptr  new_tex(new texture_1d(*this, in_desc));
    if (new_tex->fail()) {
        if (new_tex->bad()) {
            glerr() << log::error << "render_device::create_texture_1d(): unable to create texture object ("
                << new_tex->state().state_string() << ")." << log::end;
        }
        else {
            glerr() << log::error << "render_device::create_texture_1d(): unable to allocate texture image data ("
                << new_tex->state().state_string() << ")." << log::end;
        }
        return texture_1d_ptr();
    }
    else {
        return new_tex;
    }
}

texture_1d_ptr
LamureDevice::create_texture_1d(const texture_1d_desc& in_desc,
    const data_format         in_initial_data_format,
    const std::vector<void*>& in_initial_mip_level_data)
{
    texture_1d_ptr  new_tex(new texture_1d(*this, in_desc, in_initial_data_format, in_initial_mip_level_data));
    if (new_tex->fail()) {
        if (new_tex->bad()) {
            glerr() << log::error << "render_device::create_texture_1d(): unable to create texture object ("
                << new_tex->state().state_string() << ")." << log::end;
        }
        else {
            glerr() << log::error << "render_device::create_texture_1d(): unable to allocate texture image data ("
                << new_tex->state().state_string() << ")." << log::end;
        }
        return texture_1d_ptr();
    }
    else {
        return new_tex;
    }
}

texture_1d_ptr
LamureDevice::create_texture_1d(const unsigned      in_size,
    const data_format   in_format,
    const unsigned      in_mip_levels,
    const unsigned      in_array_layers)
{
    return create_texture_1d(texture_1d_desc(in_size, in_format, in_mip_levels, in_array_layers));
}

texture_1d_ptr
LamureDevice::create_texture_1d(const unsigned            in_size,
    const data_format         in_format,
    const unsigned            in_mip_levels,
    const unsigned            in_array_layers,
    const data_format         in_initial_data_format,
    const std::vector<void*>& in_initial_mip_level_data)
{
    return create_texture_1d(texture_1d_desc(in_size, in_format, in_mip_levels, in_array_layers),
        in_initial_data_format,
        in_initial_mip_level_data);
}

texture_1d_ptr
LamureDevice::create_texture_1d(const texture_1d_ptr& in_orig_texture,
    const data_format         in_format,
    const math::vec2ui& in_mip_range,
    const math::vec2ui& in_layer_range)
{
    texture_1d_ptr  new_tex(new texture_1d(*this, *in_orig_texture, in_format, in_mip_range, in_layer_range));
    if (new_tex->fail()) {
        glerr() << log::error << "render_device::create_texture_1d(): unable to create texture view object ("
            << new_tex->state().state_string() << ")." << log::end;
        return texture_1d_ptr();
    }
    else {
        return new_tex;
    }
}

texture_2d_ptr
LamureDevice::create_texture_2d(const texture_2d_desc& in_desc)
{
    texture_2d_ptr  new_tex(new texture_2d(*this, in_desc));
    if (new_tex->fail()) {
        if (new_tex->bad()) {
            glerr() << log::error << "render_device::create_texture_2d(): unable to create texture object ("
                << new_tex->state().state_string() << ")." << log::end;
        }
        else {
            glerr() << log::error << "render_device::create_texture_2d(): unable to allocate texture image data ("
                << new_tex->state().state_string() << ")." << log::end;
        }
        return texture_2d_ptr();
    }
    else {
        return new_tex;
    }
}

texture_2d_ptr
LamureDevice::create_texture_2d(const texture_2d_desc& in_desc,
    const data_format         in_initial_data_format,
    const std::vector<void*>& in_initial_mip_level_data)
{
    texture_2d_ptr  new_tex(new texture_2d(*this, in_desc, in_initial_data_format, in_initial_mip_level_data));
    if (new_tex->fail()) {
        if (new_tex->bad()) {
            glerr() << log::error << "render_device::create_texture_2d(): unable to create texture object ("
                << new_tex->state().state_string() << ")." << log::end;
        }
        else {
            glerr() << log::error << "render_device::create_texture_2d(): unable to allocate texture image data ("
                << new_tex->state().state_string() << ")." << log::end;
        }
        return texture_2d_ptr();
    }
    else {
        return new_tex;
    }
}

texture_2d_ptr
LamureDevice::create_texture_2d(const math::vec2ui& in_size,
    const data_format   in_format,
    const unsigned      in_mip_levels,
    const unsigned      in_array_layers,
    const unsigned      in_samples)
{
    return create_texture_2d(texture_2d_desc(in_size, in_format, in_mip_levels, in_array_layers, in_samples));
}

texture_2d_ptr
LamureDevice::create_texture_2d(const math::vec2ui& in_size,
    const data_format         in_format,
    const unsigned            in_mip_levels,
    const unsigned            in_array_layers,
    const unsigned            in_samples,
    const data_format         in_initial_data_format,
    const std::vector<void*>& in_initial_mip_level_data)
{
    return create_texture_2d(texture_2d_desc(in_size, in_format, in_mip_levels, in_array_layers, in_samples),
        in_initial_data_format,
        in_initial_mip_level_data);
}

texture_2d_ptr
LamureDevice::create_texture_2d(const texture_2d_ptr& in_orig_texture,
    const data_format         in_format,
    const math::vec2ui& in_mip_range,
    const math::vec2ui& in_layer_range)
{
    texture_2d_ptr  new_tex(new texture_2d(*this, *in_orig_texture, in_format, in_mip_range, in_layer_range));
    if (new_tex->fail()) {
        glerr() << log::error << "render_device::create_texture_2d(): unable to create texture view object ("
            << new_tex->state().state_string() << ")." << log::end;
        return texture_2d_ptr();
    }
    else {
        return new_tex;
    }
}

texture_3d_ptr
LamureDevice::create_texture_3d(const texture_3d_desc& in_desc)
{
    texture_3d_ptr  new_tex(new texture_3d(*this, in_desc));
    if (new_tex->fail()) {
        if (new_tex->bad()) {
            glerr() << log::error << "render_device::create_texture_3d(): unable to create texture object ("
                << new_tex->state().state_string() << ")." << log::end;
        }
        else {
            glerr() << log::error << "render_device::create_texture_3d(): unable to allocate texture image data ("
                << new_tex->state().state_string() << ")." << log::end;
        }
        return texture_3d_ptr();
    }
    else {
        return new_tex;
    }
}

texture_3d_ptr
LamureDevice::create_texture_3d(const texture_3d_desc& in_desc,
    const data_format         in_initial_data_format,
    const std::vector<void*>& in_initial_mip_level_data)
{
    texture_3d_ptr  new_tex(new texture_3d(*this, in_desc, in_initial_data_format, in_initial_mip_level_data));
    if (new_tex->fail()) {
        if (new_tex->bad()) {
            glerr() << log::error << "render_device::create_texture_3d(): unable to create texture object ("
                << new_tex->state().state_string() << ")." << log::end;
        }
        else {
            glerr() << log::error << "render_device::create_texture_3d(): unable to allocate texture image data ("
                << new_tex->state().state_string() << ")." << log::end;
        }
        return texture_3d_ptr();
    }
    else {
        return new_tex;
    }
}

texture_3d_ptr
LamureDevice::create_texture_3d(const math::vec3ui& in_size,
    const data_format   in_format,
    const unsigned      in_mip_levels)
{
    return create_texture_3d(texture_3d_desc(in_size, in_format, in_mip_levels));
}

texture_3d_ptr
LamureDevice::create_texture_3d(const math::vec3ui& in_size,
    const data_format         in_format,
    const unsigned            in_mip_levels,
    const data_format         in_initial_data_format,
    const std::vector<void*>& in_initial_mip_level_data)
{
    return create_texture_3d(texture_3d_desc(in_size, in_format, in_mip_levels),
        in_initial_data_format,
        in_initial_mip_level_data);
}

texture_3d_ptr
LamureDevice::create_texture_3d(const texture_3d_ptr& in_orig_texture,
    const data_format         in_format,
    const math::vec2ui& in_mip_range)
{
    texture_3d_ptr  new_tex(new texture_3d(*this, *in_orig_texture, in_format, in_mip_range));
    if (new_tex->fail()) {
        glerr() << log::error << "render_device::create_texture_3d(): unable to create texture view object ("
            << new_tex->state().state_string() << ")." << log::end;
        return texture_3d_ptr();
    }
    else {
        return new_tex;
    }
}

texture_cube_ptr
LamureDevice::create_texture_cube(const texture_cube_desc& in_desc)
{
    texture_cube_ptr  new_tex(new texture_cube(*this, in_desc));
    if (new_tex->fail()) {
        if (new_tex->bad()) {
            glerr() << log::error << "render_device::create_texture_cube(): unable to create texture object ("
                << new_tex->state().state_string() << ")." << log::end;
        }
        else {
            glerr() << log::error << "render_device::create_texture_cube(): unable to allocate texture image data ("
                << new_tex->state().state_string() << ")." << log::end;
        }
        return texture_cube_ptr();
    }
    else {
        return new_tex;
    }
}

texture_cube_ptr
LamureDevice::create_texture_cube(const texture_cube_desc& in_desc,
    const data_format         in_initial_data_format,
    const std::vector<void*>& in_initial_mip_level_data_px,
    const std::vector<void*>& in_initial_mip_level_data_nx,
    const std::vector<void*>& in_initial_mip_level_data_py,
    const std::vector<void*>& in_initial_mip_level_data_ny,
    const std::vector<void*>& in_initial_mip_level_data_pz,
    const std::vector<void*>& in_initial_mip_level_data_nz)
{
    texture_cube_ptr  new_tex(new texture_cube(*this, in_desc, in_initial_data_format,
        in_initial_mip_level_data_px,
        in_initial_mip_level_data_nx,
        in_initial_mip_level_data_py,
        in_initial_mip_level_data_ny,
        in_initial_mip_level_data_pz,
        in_initial_mip_level_data_nz));
    if (new_tex->fail()) {
        if (new_tex->bad()) {
            glerr() << log::error << "render_device::create_texture_cube(): unable to create texture object ("
                << new_tex->state().state_string() << ")." << log::end;
        }
        else {
            glerr() << log::error << "render_device::create_texture_cube(): unable to allocate texture image data ("
                << new_tex->state().state_string() << ")." << log::end;
        }
        return texture_cube_ptr();
    }
    else {
        return new_tex;
    }
}

texture_cube_ptr
LamureDevice::create_texture_cube(const math::vec2ui& in_size,
    const data_format   in_format,
    const unsigned      in_mip_levels)
{
    return create_texture_cube(texture_cube_desc(in_size, in_format, in_mip_levels));
}

texture_cube_ptr
LamureDevice::create_texture_cube(const math::vec2ui& in_size,
    const data_format         in_format,
    const unsigned            in_mip_levels,
    const data_format         in_initial_data_format,
    const std::vector<void*>& in_initial_mip_level_data_px,
    const std::vector<void*>& in_initial_mip_level_data_nx,
    const std::vector<void*>& in_initial_mip_level_data_py,
    const std::vector<void*>& in_initial_mip_level_data_ny,
    const std::vector<void*>& in_initial_mip_level_data_pz,
    const std::vector<void*>& in_initial_mip_level_data_nz)
{
    return create_texture_cube(texture_cube_desc(in_size, in_format, in_mip_levels),
        in_initial_data_format,
        in_initial_mip_level_data_px,
        in_initial_mip_level_data_nx,
        in_initial_mip_level_data_py,
        in_initial_mip_level_data_ny,
        in_initial_mip_level_data_pz,
        in_initial_mip_level_data_nz);
}

texture_buffer_ptr
LamureDevice::create_texture_buffer(const texture_buffer_desc& in_desc)
{
    texture_buffer_ptr  new_tex(new texture_buffer(*this, in_desc));
    if (new_tex->fail()) {
        if (new_tex->bad()) {
            glerr() << log::error << "render_device::create_texture_buffer(): unable to create texture buffer object ("
                << new_tex->state().state_string() << ")." << log::end;
        }
        else {
            glerr() << log::error << "render_device::create_texture_buffer(): unable to allocate or attach texture buffer data ("
                << new_tex->state().state_string() << ")." << log::end;
        }
        return texture_buffer_ptr();
    }
    else {
        return new_tex;
    }
}

texture_buffer_ptr
LamureDevice::create_texture_buffer(const data_format   in_format,
    const buffer_ptr& in_buffer)
{
    return create_texture_buffer(texture_buffer_desc(in_format, in_buffer));
}

texture_buffer_ptr
LamureDevice::create_texture_buffer(const data_format   in_format,
    buffer_usage        in_buffer_usage,
    scm::size_t         in_buffer_size,
    const void* in_buffer_initial_data)
{
    buffer_ptr  tex_buffer = create_buffer(BIND_TEXTURE_BUFFER, in_buffer_usage, in_buffer_size, in_buffer_initial_data);
    if (!tex_buffer) {
        glerr() << log::error << "render_device::create_texture_buffer(): unable to create texture buffer data buffer." << log::end;
        return texture_buffer_ptr();
    }
    return create_texture_buffer(in_format, tex_buffer);
}

texture_handle_ptr
LamureDevice::create_resident_handle(const texture_ptr& in_texture,
    const sampler_state_ptr& in_sampler)
{
    assert(in_texture);
    assert(in_sampler);

    texture_handle_ptr new_tex_handle(new texture_handle(*this, *in_texture, *in_sampler));
    if (new_tex_handle->fail()) {
        glerr() << log::error << "render_device::create_resident_handle(): unable to create texture handle ("
            << new_tex_handle->state().state_string() << ")." << log::end;
        return texture_handle_ptr();
    }
    else {
        return new_tex_handle;
    }
}



LamureDevice::~LamureDevice()
{
    _main_context.reset();

    assert(0 == _registered_resources.size());
}


