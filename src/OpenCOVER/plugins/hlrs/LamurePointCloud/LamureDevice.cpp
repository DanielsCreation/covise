
// Copyright (c) 2012 Christopher Lux <christopherlux@gmail.com>
// Distributed under the Modified BSD License, see license.txt.

#include "LamureDevice.h"
#include "LamureContext.h"


#include <lamure/types.h>

#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>

#include <scm/core/io/tools.h>
#include <scm/core/io/iomanip.h>
#include <scm/core/log/logger_state.h>
#include <scm/core/utilities/foreach.h>


#define SCM_GL_CORE_OPENGL_CORE_VERSION 200
#define SCM_GL_CORE_OPENGL_CORE_VERSION_410 410

class LamureContext;

struct LamureDevice::mutex_impl
{
    boost::mutex    _mutex;
};

LamureDevice::LamureDevice() : _mutex_impl(new mutex_impl)
{
    //_device = this;
    unsigned req_version_major = SCM_GL_CORE_OPENGL_CORE_VERSION / 100;
    unsigned req_version_minor = (SCM_GL_CORE_OPENGL_CORE_VERSION - req_version_major * 100) / 10;

    init_capabilities();

    _main_context.reset(new LamureContext(*this));

}

scm::gl::render_context_ptr LamureDevice::main_context() const
{
    return _main_context;
}

const LamureDevice::device_capabilities&
LamureDevice::capabilities() const
{
    return _capabilities;
}

LamureDevice::~LamureDevice()
{
}



void LamureDevice::init_capabilities() {

    glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, &_capabilities._max_vertex_attributes);
    glGetIntegerv(GL_MAX_DRAW_BUFFERS, &_capabilities._max_draw_buffers);
    glGetIntegerv(GL_MAX_DUAL_SOURCE_DRAW_BUFFERS, &_capabilities._max_dual_source_draw_buffers);

    assert(_capabilities._max_vertex_attributes > 0);
    assert(_capabilities._max_draw_buffers > 0);
    assert(_capabilities._max_dual_source_draw_buffers > 0);

    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &_capabilities._max_texture_size);
    glGetIntegerv(GL_MAX_3D_TEXTURE_SIZE, &_capabilities._max_texture_3d_size);
    glGetIntegerv(GL_MAX_ARRAY_TEXTURE_LAYERS, &_capabilities._max_array_texture_layers);
    glGetIntegerv(GL_MAX_SAMPLES, &_capabilities._max_samples);
    glGetIntegerv(GL_MAX_DEPTH_TEXTURE_SAMPLES, &_capabilities._max_depth_texture_samples);
    glGetIntegerv(GL_MAX_COLOR_TEXTURE_SAMPLES, &_capabilities._max_color_texture_samples);
    glGetIntegerv(GL_MAX_INTEGER_SAMPLES, &_capabilities._max_integer_samples);
    glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &_capabilities._max_texture_image_units);
    glGetIntegerv(GL_MAX_TEXTURE_BUFFER_SIZE, &_capabilities._max_texture_buffer_size);
    glGetIntegerv(GL_MAX_COLOR_ATTACHMENTS, &_capabilities._max_frame_buffer_color_attachments);

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

    glGetIntegerv(GL_MAX_VERTEX_UNIFORM_BLOCKS, &_capabilities._max_vertex_uniform_blocks);
    glGetIntegerv(GL_MAX_GEOMETRY_UNIFORM_BLOCKS, &_capabilities._max_geometry_uniform_blocks);
    glGetIntegerv(GL_MAX_FRAGMENT_UNIFORM_BLOCKS, &_capabilities._max_fragment_uniform_blocks);
    glGetIntegerv(GL_MAX_COMBINED_UNIFORM_BLOCKS, &_capabilities._max_combined_uniform_blocks);
    glGetIntegerv(GL_MAX_COMBINED_VERTEX_UNIFORM_COMPONENTS, &_capabilities._max_combined_vertex_uniform_components);
    glGetIntegerv(GL_MAX_COMBINED_GEOMETRY_UNIFORM_COMPONENTS, &_capabilities._max_combined_geometry_uniform_components);
    glGetIntegerv(GL_MAX_COMBINED_FRAGMENT_UNIFORM_COMPONENTS, &_capabilities._max_combined_fragment_uniform_components);
    glGetIntegerv(GL_MAX_UNIFORM_BUFFER_BINDINGS, &_capabilities._max_uniform_buffer_bindings);
    glGetIntegerv(GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT, &_capabilities._uniform_buffer_offset_alignment);
    glGetIntegerv(GL_MAX_UNIFORM_BLOCK_SIZE, &_capabilities._max_uniform_block_size);

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
        glGetIntegerv(GL_MAX_VIEWPORTS, &_capabilities._max_viewports);
    }
    else {
        _capabilities._max_viewports = 1;
    }

    glGetIntegerv(GL_MAX_TRANSFORM_FEEDBACK_SEPARATE_ATTRIBS, &_capabilities._max_transform_feedback_separate_attribs);
    if (SCM_GL_CORE_OPENGL_CORE_VERSION >= SCM_GL_CORE_OPENGL_CORE_VERSION_400) {
        glGetIntegerv(GL_MAX_TRANSFORM_FEEDBACK_BUFFERS, &_capabilities._max_transform_feedback_buffers);
        glGetIntegerv(GL_MAX_VERTEX_STREAMS, &_capabilities._max_vertex_streams);
    }
    else {
        _capabilities._max_transform_feedback_buffers = _capabilities._max_transform_feedback_separate_attribs;
        _capabilities._max_vertex_streams = 1;
    }
    _capabilities._max_transform_feedback_buffers = _capabilities._max_transform_feedback_separate_attribs;
    _capabilities._max_vertex_streams = 1;
    assert(_capabilities._max_transform_feedback_separate_attribs > 0);
    assert(_capabilities._max_transform_feedback_buffers > 0);
    assert(_capabilities._max_vertex_streams > 0);

    if (SCM_GL_CORE_OPENGL_CORE_VERSION >= SCM_GL_CORE_OPENGL_CORE_VERSION_420) {
        glGetIntegerv(GL_MAX_IMAGE_UNITS, &_capabilities._max_image_units);
    }
    //else if (glcore.extension_EXT_shader_image_load_store) {
    //    glGetIntegerv(GL_MAX_IMAGE_UNITS_EXT, &_capabilities._max_image_units);
    //}
    else {
        _capabilities._max_image_units = 0;
    }

    if (SCM_GL_CORE_OPENGL_CORE_VERSION >= SCM_GL_CORE_OPENGL_CORE_VERSION_420) {
        glGetIntegerv(GL_MAX_VERTEX_ATOMIC_COUNTERS, &_capabilities._max_vertex_atomic_counters);
        glGetIntegerv(GL_MAX_FRAGMENT_ATOMIC_COUNTERS, &_capabilities._max_geometry_atomic_counters);
        glGetIntegerv(GL_MAX_GEOMETRY_ATOMIC_COUNTERS, &_capabilities._max_fragment_atomic_counters);
        glGetIntegerv(GL_MAX_COMBINED_ATOMIC_COUNTERS, &_capabilities._max_combined_atomic_counters);
        glGetIntegerv(GL_MAX_ATOMIC_COUNTER_BUFFER_BINDINGS, &_capabilities._max_atomic_counter_buffer_bindings);

        assert(_capabilities._max_vertex_atomic_counters >= 0);
        assert(_capabilities._max_geometry_atomic_counters >= 0);
        assert(_capabilities._max_fragment_atomic_counters > 0);
        assert(_capabilities._max_combined_atomic_counters > 0);
        assert(_capabilities._max_atomic_counter_buffer_bindings > 0);
    }
    else {
        _capabilities._max_vertex_atomic_counters = 0;
        _capabilities._max_geometry_atomic_counters = 0;
        _capabilities._max_fragment_atomic_counters = 0;
        _capabilities._max_combined_atomic_counters = 0;
        _capabilities._max_atomic_counter_buffer_bindings = 0;
    }
    _capabilities._max_vertex_atomic_counters = 0;
    _capabilities._max_geometry_atomic_counters = 0;
    _capabilities._max_fragment_atomic_counters = 0;
    _capabilities._max_combined_atomic_counters = 0;
    _capabilities._max_atomic_counter_buffer_bindings = 0;

    //if (SCM_GL_CORE_OPENGL_CORE_VERSION >= SCM_GL_CORE_OPENGL_CORE_VERSION_420
    //    || glcore.extension_ARB_map_buffer_alignment) {
    //    glGetIntegerv(GL_MIN_MAP_BUFFER_ALIGNMENT, &_capabilities._min_map_buffer_alignment);
    //}
    //else {
    //    _capabilities._min_map_buffer_alignment = 1;
    //}
    _capabilities._min_map_buffer_alignment = 1;

    if (SCM_GL_CORE_OPENGL_CORE_VERSION >= SCM_GL_CORE_OPENGL_CORE_VERSION_410) {
        glGetIntegerv(GL_NUM_PROGRAM_BINARY_FORMATS, &_capabilities._num_program_binary_formats);
        if (_capabilities._num_program_binary_formats > 0) {
            _capabilities._program_binary_formats.reset(new int[_capabilities._num_program_binary_formats]);
            glGetIntegerv(GL_PROGRAM_BINARY_FORMATS, _capabilities._program_binary_formats.get());
        }
    }
    else {
        _capabilities._num_program_binary_formats = 0;
    }
    _capabilities._num_program_binary_formats = 0;

    if (SCM_GL_CORE_OPENGL_CORE_VERSION >= SCM_GL_CORE_OPENGL_CORE_VERSION_430) {
        glGetIntegerv(GL_MAX_SHADER_STORAGE_BUFFER_BINDINGS, &_capabilities._max_shader_storage_block_bindings);
        glGetIntegerv(GL_MAX_SHADER_STORAGE_BLOCK_SIZE, &_capabilities._max_shader_storage_block_size);
        glGetInteger64v(GL_SHADER_STORAGE_BUFFER_OFFSET_ALIGNMENT, &_capabilities._shader_storage_buffer_offset_alignment);
    }
    else {
        _capabilities._max_shader_storage_block_bindings = 0;
        _capabilities._max_shader_storage_block_size = 0;
        _capabilities._shader_storage_buffer_offset_alignment = 1;
    }
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

    std::cout << "GL_MAX_IMAGE_UNITS_EXT " << _capabilities._max_image_units << std::endl;
}

// buffer api /////////////////////////////////////////////////////////////////////////////////////
scm::gl::buffer_ptr
LamureDevice::create_buffer(const scm::gl::buffer_desc& in_buffer_desc,
    const void* in_initial_data)
{
    scm::gl::buffer_ptr new_buffer = NULL;
    //scm::gl::buffer_ptr new_buffer(new scm::gl::buffer(*this, in_buffer_desc, in_initial_data), boost::bind(&LamureDevice::release_resource, this, _1));
    if (new_buffer->fail()) {
        /*if (new_buffer->bad()) {
            scm::gl::glerr() << scm::log::error << "render_device::create_buffer(): unable to create buffer object ("
                << new_buffer->state().state_string() << ")." << scm::log::end;
        }
        else {
            scm::gl::glerr() << scm::log::error << "render_device::create_buffer(): unable to allocate buffer ("
                << new_buffer->state().state_string() << ")." << scm::log::end;
        }*/
        return scm::gl::buffer_ptr();
    }
    else {
        //register_resource(new_buffer.get());
        return new_buffer;
    }
}

scm::gl::buffer_ptr
LamureDevice::create_buffer(scm::gl::buffer_binding in_binding,
    scm::gl::buffer_usage   in_usage,
    scm::size_t    in_size,
    const void* in_initial_data)
{
    return create_buffer(scm::gl::buffer_desc(in_binding, in_usage, in_size), in_initial_data);
}

bool
LamureDevice::resize_buffer(const scm::gl::buffer_ptr& in_buffer, scm::size_t in_size)
{
    scm::gl::buffer_desc desc = in_buffer->descriptor();
    desc._size = in_size;
    /*if (!in_buffer->buffer_data(*this, desc, 0)) {
        scm::gl::glerr() << scm::log::error << "render_device::resize_buffer(): unable to reallocate buffer ("
            << in_buffer->state().state_string() << ")." << scm::log::end;
        return false;
    }
    else {
        return true;
    }*/return true;
}

scm::gl::vertex_array_ptr
LamureDevice::create_vertex_array(const scm::gl::vertex_format& in_vert_fmt,
    const buffer_array& in_attrib_buffers,
    const scm::gl::program_ptr& in_program)
{
    scm::gl::vertex_array_ptr new_array = NULL;
    /*scm::gl::vertex_array_ptr new_array(new scm::gl::vertex_array(*this, in_vert_fmt, in_attrib_buffers, in_program));
    if (new_array->fail()) {
        if (new_array->bad()) {
            scm::gl::glerr() << scm::log::error << "render_device::create_vertex_array(): unable to create vertex array object ("
                << new_array->state().state_string() << ")." << scm::log::end;
        }
        else {
            scm::gl::glerr() << scm::log::error << "render_device::create_vertex_array(): unable to initialize vertex array object ("
                << new_array->state().state_string() << ")." << scm::log::end;
        }
        return scm::gl::vertex_array_ptr();
    }*/
    return new_array;
}

scm::gl::transform_feedback_ptr
LamureDevice::create_transform_feedback(const scm::gl::stream_output_setup& in_setup)
{
    /*scm::gl::transform_feedback_ptr new_feedback(new scm::gl::transform_feedback(*this, in_setup));
    if (new_feedback->fail()) {
        if (new_feedback->bad()) {
            scm::gl::glerr() << scm::log::error << "render_device::create_transform_feedback(): unable to create transform feedback object ("
                << new_feedback->state().state_string() << ")." << scm::log::end;
        }
        else {
            scm::gl::glerr() << scm::log::error << "render_device::create_transform_feedback(): unable to initialize transform feedback object ("
                << new_feedback->state().state_string() << ")." << scm::log::end;
        }
        return scm::gl::transform_feedback_ptr();
    }
    return new_feedback;*/return scm::gl::transform_feedback_ptr();
}

// shader api /////////////////////////////////////////////////////////////////////////////////////
bool
LamureDevice::add_include_files(const std::string& in_path,
    const std::string& in_glsl_root_path,
    const std::string& in_file_extensions,
    bool               in_scan_subdirectories)
{
    { // protect this function from multiple thread access
        boost::mutex::scoped_lock lock(_mutex_impl->_mutex);

        typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
        boost::char_separator<char> space_separator(" ");
        tokenizer                   file_extensions(in_file_extensions, space_separator);

        namespace bfs = boost::filesystem;

        std::string         output_root_path
            = boost::trim_left_copy_if(
                boost::trim_right_copy_if(
                    in_glsl_root_path,
                    boost::is_any_of("/")),
                boost::is_any_of("/"));

        if (!output_root_path.empty()) {
            output_root_path = std::string("/") + output_root_path + std::string("/");
        }
        else {
            output_root_path = std::string("/");
        }

        bfs::path           input_path = bfs::path(in_path);
        bfs::path           input_root;

        //if (input_path.is_relative()) {
        //    input_path = bfs::absolute(input_path);
        //}

        if (!bfs::exists(input_path)) {
            scm::gl::glerr() << scm::log::error << "render_device::add_include_files(): "
                << "<error> input path does not exist (" << input_path << ")." << scm::log::end;
            return false;
        }

        if (bfs::is_directory(input_path)) {
            input_root = input_path;
        }
        else {
            scm::gl::glerr() << scm::log::error << "render_device::add_include_files(): "
                << "<error> input path is a file (" << input_path << ")." << scm::log::end;
            return false;
        }

        if (in_scan_subdirectories) {
            bfs::recursive_directory_iterator  file_iter(input_path);
            bfs::recursive_directory_iterator  e = bfs::recursive_directory_iterator();
            for (; file_iter != e; ++file_iter) {
                bfs::path current_file = file_iter->path();
                if (!bfs::is_directory(current_file)) {
                    if (std::find(file_extensions.begin(),
                        file_extensions.end(),
                        current_file.extension().string())
                        != file_extensions.end())
                    {
                        std::string     source_string;
                        if (scm::io::read_text_file(current_file.string(), source_string)) {
                            // me not likey... but does the trick in a portable manner
                            bfs::path::const_iterator first_mis
                                = std::mismatch(input_root.begin(), input_root.end(),
                                    current_file.begin()).second;
                            bfs::path input_rel_path;
                            for (; first_mis != current_file.end(); ++first_mis) input_rel_path /= *first_mis;

                            assert(input_path / input_rel_path == current_file);
                            add_include_string_internal(output_root_path + input_rel_path.generic_string(), source_string, false);
                        }
                        else {
                            scm::gl::glout() << scm::log::warning << "render_device::add_include_files(): error reading shader file " << current_file << scm::log::end;
                        }
                    }
                }
            }
        }
        else {
            bfs::directory_iterator  file_iter(input_path);
            bfs::directory_iterator  e = bfs::directory_iterator();
            for (; file_iter != e; ++file_iter) {
                bfs::path current_file = file_iter->path();
                if (!bfs::is_directory(current_file)) {
                    if (std::find(file_extensions.begin(),
                        file_extensions.end(),
                        current_file.extension().string())
                        != file_extensions.end())
                    {
                        std::string     source_string;
                        if (scm::io::read_text_file(current_file.string(), source_string)) {
                            // me not likey... but does the trick in a portable manner
                            bfs::path::const_iterator first_mis
                                = std::mismatch(input_root.begin(), input_root.end(),
                                    current_file.begin()).second;
                            bfs::path input_rel_path;
                            for (; first_mis != current_file.end(); ++first_mis) input_rel_path /= *first_mis;

                            assert(input_path / input_rel_path == current_file);
                            add_include_string_internal(output_root_path + input_rel_path.generic_string(), source_string, false);
                        }
                        else {
                            scm::gl::glout() << scm::log::warning << "render_device::add_include_files(): error reading shader file " << current_file << scm::log::end;
                        }
                    }
                }
            }
        }
    }

    return true;
}

bool
LamureDevice::add_include_string(const std::string& in_path,
    const std::string& in_source_string)
{
    return add_include_string_internal(in_path, in_source_string, true);
}

bool
LamureDevice::add_include_string_internal(const std::string& in_path,
    const std::string& in_source_string,
    bool         lock_thread)
{
    { // protect this function from multiple thread access
        /*scm::scoped_ptr<boost::mutex::scoped_lock> lock;
        if (lock_thread) {
            lock.reset(new boost::mutex::scoped_lock(_mutex_impl->_mutex));
        }

        const opengl::gl_core& glcore = opengl_api();
        scm::gl::util::gl_error          glerror(glcore);
        if (!glcore.extension_ARB_shading_language_include) {
            scm::gl::glout() << scm::log::warning << "render_device::add_include_string(): "
                << "shader includes not supported (GL_ARB_shading_language_include unsupported), ignoring include string." << scm::log::end;
            return false;
        }
        if (in_path[0] != '/') {
            scm::gl::glerr() << scm::log::error << "render_device::add_include_string(): "
                << "<error> path not starting with '/'." << scm::log::end;
            return false;
        }
        glcore.glNamedStringARB(GL_SHADER_INCLUDE_ARB,
            static_cast<int>(in_path.length()), in_path.c_str(),
            static_cast<int>(in_source_string.length()), in_source_string.c_str());
        if (glerror) {
            switch (glerror.to_object_state()) {
            case scm::gl::object_state::OS_ERROR_INVALID_VALUE:
                scm::gl::glerr() << scm::log::error << "render_device::add_include_string(): "
                    << "error creating named include string (path or source string empty or path not starting with '/'." << scm::log::end;
                return false;
                break;
            default:
                scm::gl::glerr() << scm::log::error << "render_device::add_include_string(): "
                    << "error creating named include string (an unknown error occured)" << scm::log::end;
                return false;
            }
        }
        size_t      parent_path_end = in_path.find_last_of('/');
        std::string parent_path = in_path.substr(0, parent_path_end);

        //if (!parent_path.empty()) {
        //    _default_include_paths.insert(parent_path);
        //}

        gl_assert(glcore, leaving render_device::add_include_string());*/
    }

    return true;
}

void
LamureDevice::add_macro_define(const std::string& in_name,
    const std::string& in_value)
{
    { // protect this function from multiple thread access
        boost::mutex::scoped_lock lock(_mutex_impl->_mutex);

        _default_macro_defines[in_name] = scm::gl::shader_macro(in_name, in_value);
    }
}

void
LamureDevice::add_macro_define(const scm::gl::shader_macro& in_macro)
{
    { // protect this function from multiple thread access
        boost::mutex::scoped_lock lock(_mutex_impl->_mutex);

        _default_macro_defines[in_macro._name] = in_macro;
    }
}

void
LamureDevice::add_macro_defines(const scm::gl::shader_macro_array& in_macros)
{
    { // protect this function from multiple thread access
        /*boost::mutex::scoped_lock lock(_mutex_impl->_mutex);

        foreach(const shader_macro & m, in_macros.macros()) {
            _default_macro_defines[m._name] = m;
        }*/
    }
}


scm::gl::shader_ptr
LamureDevice::create_shader(scm::gl::shader_stage       in_stage,
    const std::string& in_source,
    const std::string& in_source_name)
{
    return create_shader(in_stage, in_source, scm::gl::shader_macro_array(), scm::gl::shader_include_path_list(), in_source_name);
}


scm::gl::shader_ptr
LamureDevice::create_shader(scm::gl::shader_stage              in_stage,
    const std::string& in_source,
    const scm::gl::shader_macro_array& in_macros,
    const std::string& in_source_name)
{
    return create_shader(in_stage, in_source, in_macros, scm::gl::shader_include_path_list(), in_source_name);
}

scm::gl::shader_ptr
LamureDevice::create_shader(scm::gl::shader_stage                    in_stage,
    const std::string& in_source,
    const scm::gl::shader_include_path_list& in_inc_paths,
    const std::string& in_source_name)
{
    return create_shader(in_stage, in_source, scm::gl::shader_macro_array(), in_inc_paths, in_source_name);
}


scm::gl::shader_ptr
LamureDevice::create_shader(scm::gl::shader_stage                    in_stage,
    const std::string& in_source,
    const scm::gl::shader_macro_array& in_macros,
    const scm::gl::shader_include_path_list& in_inc_paths,
    const std::string& in_source_name)
{
    // combine macro definitions
    scm::gl::shader_macro_array  macro_array(in_macros);

    { // protect this function from multiple thread access
        boost::mutex::scoped_lock lock(_mutex_impl->_mutex);

        shader_macro_map::const_iterator mb = _default_macro_defines.begin();
        shader_macro_map::const_iterator me = _default_macro_defines.end();

        for (; mb != me; ++mb) {
            macro_array(mb->second._name, mb->second._value);
        }
    }

    // combine shader include paths
    scm::gl::shader_include_path_list   include_paths(in_inc_paths);

    { // protect this function from multiple thread access
        boost::mutex::scoped_lock lock(_mutex_impl->_mutex);

        string_set::const_iterator ipb = _default_include_paths.begin();
        string_set::const_iterator ipe = _default_include_paths.end();

        for (; ipb != ipe; ++ipb) {
            include_paths.push_back(*ipb);
        }
    }

    /*scm::gl::shader_ptr new_shader(new scm::gl::shader(*this,
        in_stage,
        in_source,
        in_source_name,
        macro_array,
        include_paths));
    if (new_shader->fail()) {
        if (new_shader->bad()) {
            scm::gl::glerr() << "render_device::create_shader(): unable to create shader object ("
                << "name: " << in_source_name << ", "
                << "stage: " << shader_stage_string(in_stage) << ", "
                << new_shader->state().state_string() << ")." << scm::log::end;
        }
        else {
            scm::gl::glerr() << "render_device::create_shader(): unable to compile shader ("
                << "name: " << in_source_name << ", "
                << "stage: " << shader_stage_string(in_stage) << ", "
                << new_shader->state().state_string() << "):" << scm::log::nline
                << new_shader->info_log() << scm::log::end;
        }
        return scm::gl::shader_ptr();
    }
    else {
        if (!new_shader->info_log().empty()) {
            scm::gl::glout() << scm::log::info << "render_device::create_shader(): compiler info ("
                << "name: " << in_source_name << ", "
                << "stage: " << shader_stage_string(in_stage)
                << ")" << scm::log::nline
                << new_shader->info_log() << scm::log::end;
        }
        return new_shader;
    }*/return scm::gl::shader_ptr();
}

scm::gl::shader_ptr
LamureDevice::create_shader_from_file(scm::gl::shader_stage       in_stage,
    const std::string& in_file_name)
{
    return create_shader_from_file(in_stage, in_file_name, scm::gl::shader_macro_array(), scm::gl::shader_include_path_list());
}

scm::gl::shader_ptr
LamureDevice::create_shader_from_file(scm::gl::shader_stage              in_stage,
    const std::string& in_file_name,
    const scm::gl::shader_macro_array& in_macros)
{
    return create_shader_from_file(in_stage, in_file_name, in_macros, scm::gl::shader_include_path_list());
}

scm::gl::shader_ptr
LamureDevice::create_shader_from_file(scm::gl::shader_stage                    in_stage,
    const std::string& in_file_name,
    const scm::gl::shader_include_path_list& in_inc_paths)
{
    return create_shader_from_file(in_stage, in_file_name, scm::gl::shader_macro_array(), in_inc_paths);
}

scm::gl::shader_ptr
LamureDevice::create_shader_from_file(scm::gl::shader_stage                    in_stage,
    const std::string& in_file_name,
    const scm::gl::shader_macro_array& in_macros,
    const scm::gl::shader_include_path_list& in_inc_paths)
{
    namespace bfs = boost::filesystem;
    bfs::path       file_path(in_file_name);
    std::string     source_string;

    if (!bfs::exists(file_path)) {
        scm::gl::glerr() << "render_device::create_shader_from_file(): unable to find shader file " << in_file_name << scm::log::end;
        return (scm::gl::shader_ptr());
    }
    if (!scm::io::read_text_file(in_file_name, source_string)) {
        scm::gl::glerr() << "render_device::create_shader_from_file(): error reading shader file " << in_file_name << scm::log::end;
        return (scm::gl::shader_ptr());
    }

    return create_shader(in_stage, source_string, in_macros, in_inc_paths, file_path.filename().string());
}

scm::gl::program_ptr
LamureDevice::create_program(const shader_list& in_shaders,
    const std::string& in_program_name)
{
    return create_program(in_shaders, scm::gl::stream_capture_array(), false, in_program_name);
}

scm::gl::program_ptr
LamureDevice::create_program(const shader_list& in_shaders,
    const scm::gl::stream_capture_array& in_capture,
    bool                        in_rasterization_discard,
    const std::string& in_program_name)
{
    /*scm::gl::program_ptr new_program(new scm::gl::program(*this, in_shaders, in_capture, in_rasterization_discard));
    if (new_program->fail()) {
        if (new_program->bad()) {
            scm::gl::glerr() << "render_device::create_program(): unable to create shader object ("
                << "name: " << in_program_name << ", "
                << new_program->state().state_string() << ")." << scm::log::end;
        }
        else {
            scm::gl::glerr() << "render_device::create_program(): error during link operation ("
                << "name: " << in_program_name << ", "
                << new_program->state().state_string() << "):" << scm::log::nline
                << new_program->info_log() << scm::log::end;
        }
        return scm::gl::program_ptr();
    }
    else {
        if (!new_program->info_log().empty()) {
            scm::gl::glout() << scm::log::info << "render_device::create_program(): linker info ("
                << "name: " << in_program_name << ")" << scm::log::nline
                << new_program->info_log() << scm::log::end;
        }
        return new_program;
    }*/return scm::gl::program_ptr();
}

void
LamureDevice::register_resource(scm::gl::render_device_resource* res_ptr)
{
    { // protect this function from multiple thread access
        boost::mutex::scoped_lock lock(_mutex_impl->_mutex);

        _registered_resources.insert(res_ptr);
    }
}

void
LamureDevice::release_resource(scm::gl::render_device_resource* res_ptr)
{
    { // protect this function from multiple thread access
        boost::mutex::scoped_lock lock(_mutex_impl->_mutex);

        resource_ptr_set::iterator res_iter = _registered_resources.find(res_ptr);
        if (res_iter != _registered_resources.end()) {
            _registered_resources.erase(res_iter);
        }

        delete res_ptr;
    }
}

scm::gl::blend_state_ptr LamureDevice::create_blend_state(const scm::gl::blend_state_desc& in_desc)
{
    return scm::gl::blend_state_ptr();
}

scm::gl::blend_state_ptr LamureDevice::create_blend_state(bool in_enabled, scm::gl::blend_func in_src_rgb_func, scm::gl::blend_func in_dst_rgb_func, scm::gl::blend_func in_src_alpha_func, scm::gl::blend_func in_dst_alpha_func, scm::gl::blend_equation in_rgb_equation, scm::gl::blend_equation in_alpha_equation, unsigned in_write_mask, bool in_alpha_to_coverage)
{
    return scm::gl::blend_state_ptr();
}

scm::gl::blend_state_ptr LamureDevice::create_blend_state(const scm::gl::blend_ops_array& in_blend_ops, bool in_alpha_to_coverage)
{
    return scm::gl::blend_state_ptr();
}
