
// Copyright (c) 2012 Christopher Lux <christopherlux@gmail.com>
// Distributed under the Modified BSD License, see license.txt.

#include "LamureContext.h"
#include "LamureDevice.h"

//#include <scm/gl_core/config.h>
//#include <scm/gl_core/object_state.h>
//#include <scm/gl_core/buffer_objects.h>
#include <scm/gl_core/frame_buffer_objects.h>
//#include <scm/gl_core/query_objects.h>
//#include <scm/gl_core/shader_objects.h>
//#include <scm/gl_core/state_objects.h>
//#include <scm/gl_core/sync_objects.h>
//#include <scm/gl_core/texture_objects.h>
//#include <scm/gl_core/render_device/device.h>
//#include <scm/gl_core/render_device/opengl/gl_core.h>
//#include <scm/gl_core/render_device/opengl/util/assert.h>
//#include <scm/gl_core/render_device/opengl/util/binding_guards.h>
//#include <scm/gl_core/render_device/opengl/util/constants_helper.h>
//#include <scm/gl_core/render_device/opengl/util/data_type_helper.h>

#include <scm/gl_core/gl_core_fwd.h>
#include <scm/gl_core/frame_buffer_objects/viewport.h>
#include <scm/config.h>
#include <scm/core/math.h>
#include <scm/core/numeric_types.h>
#include <scm/gl_core/constants.h>
#include <scm/gl_core/data_types.h>
#include <scm/gl_core/data_formats.h>
#include <scm/core/platform/platform.h>
#include <scm/core/utilities/platform_warning_disable.h>

#include <boost/unordered/unordered_map.hpp>
#include <boost/unordered/unordered_set.hpp>


LamureContext::LamureContext(LamureDevice& in_device)
{
    _context = this;

    _device = &in_device;

    _active_transform_feedback_topology_mode_lmr = scm::gl::PRIMITIVE_POINTS;
}

void LamureContext::drawImplementation(osg::RenderInfo& renderInfo) const
{
    const unsigned int osgid = renderInfo.getState()->getContextID();
    osg::GLExtensions* glapi = new osg::GLExtensions(osgid);


    GLuint prog = glapi->getCurrentProgram();

    float positions[6] = { -300.0f, -300.0f, 0.0f, 300.0f, 300.0f, -300.0f };

    unsigned int buffer;
    glapi->glGenBuffers(1, &buffer);
    glapi->glBindBuffer(GL_ARRAY_BUFFER, buffer);
    glapi->glBufferData(GL_ARRAY_BUFFER, (6 * sizeof(float)), positions, GL_STATIC_DRAW);
    glapi->glEnableVertexAttribArray(0);
    glapi->glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);


    std::string vertexShader =
        "#version 330 core\n"
        "\n"
        "layout(location = 0) in vec4 position;"
        "\n"
        "void main()\n"
        "{\n"
        "   gl_Position = position;"
        "}\n";

    std::string fragmentShader =
        "#version 330 core\n"
        "\n"
        "layout(location = 0) out vec4 color;"
        "\n"
        "void main()\n"
        "{\n"
        "   color = vec4(1.0, 0.0, 0.0, 1.0);\n"
        "}\n";

    unsigned int program = CreateShader(vertexShader, fragmentShader, osgid);
    glapi->glUseProgram(program);
    //glapi->glDeleteProgram(program);
    renderInfo.getState()->disableVertexAttribPointer(0);
    //glapi->glDisableVertexAttribArray(0);
}


static unsigned int CreateShader(const std::string& vertexShader, const std::string& fragmentShader, unsigned int osgid)
{
    osg::GLExtensions* glapi = new osg::GLExtensions(osgid);
    unsigned int program = glapi->glCreateProgram();
    unsigned int vs = CompileShader(GL_VERTEX_SHADER, vertexShader, osgid);
    unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, fragmentShader, osgid);

    glapi->glAttachShader(program, vs);
    glapi->glAttachShader(program, fs);
    glapi->glLinkProgram(program);
    glapi->glValidateProgram(program);

    glapi->glDeleteShader(vs);
    glapi->glDeleteShader(fs);
    return program;
}


static unsigned int CompileShader(unsigned int type, const std::string& source, unsigned int osgid)
{
    osg::GLExtensions* glapi = new osg::GLExtensions(osgid);
    unsigned int id = glapi->glCreateShader(type);
    const char* src = source.c_str();
    glapi->glShaderSource(id, 1, &src, nullptr);
    glapi->glCompileShader(id);

    int result;
    glapi->glGetShaderiv(id, GL_COMPILE_STATUS, &result);
    if (result == false)
    {
        int length;
        glapi->glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
        char* message = (char*)alloca(length * sizeof(char));
        glapi->glGetShaderInfoLog(id, length, &length, message);
        std::cout << "Failed to compile " <<
            (type == GL_VERTEX_SHADER ? "vertex" : "fragment") << " shader!" << std::endl;
        std::cout << message << std::endl;
        glapi->glDeleteProgram(id);
        return 0;
    };
}



osg::Object* LamureContext::cloneType() const
{
    return new LamureContext(*_device);
}


osg::Object* LamureContext::clone(const osg::CopyOp&) const
{
    return new LamureContext(*_device);
}

void LamureContext::apply()
{
}

void LamureContext::reset()
{
}

void LamureContext::flush()
{
}

void LamureContext::sync()
{
}

void LamureContext::dispatch_compute(const scm::math::vec3ui& num_groups)
{
}

void LamureContext::dispatch_compute(const scm::math::vec3ui& num_groups, const scm::math::vec3ui& group_sizes)
{
}


LamureContext::~LamureContext()
{
}


LamureContext::index_buffer_binding::index_buffer_binding()
    : _primitive_topology(scm::gl::PRIMITIVE_POINT_LIST), _index_data_type(scm::gl::TYPE_UINT), _index_data_offset(0)
{
}

bool
LamureContext::index_buffer_binding::operator==(const index_buffer_binding& rhs) const
{
    return    (_index_buffer == rhs._index_buffer)
        && (_primitive_topology == rhs._primitive_topology)
        && (_index_data_type == rhs._index_data_type)
        && (_index_data_offset == rhs._index_data_offset);
}

bool
LamureContext::index_buffer_binding::operator!=(const index_buffer_binding& rhs) const
{
    return    (_index_buffer != rhs._index_buffer)
        || (_primitive_topology != rhs._primitive_topology)
        || (_index_data_type != rhs._index_data_type)
        || (_index_data_offset != rhs._index_data_offset);
}

bool
LamureContext::buffer_binding::operator==(const buffer_binding& rhs) const
{
    return    (_buffer == rhs._buffer)
        && (_offset == rhs._offset)
        && (_size == rhs._size);
}

bool
LamureContext::buffer_binding::operator!=(const buffer_binding& rhs) const
{
    return    (_buffer != rhs._buffer)
        || (_offset != rhs._offset)
        || (_size != rhs._size);
}

LamureContext::image_unit_binding::image_unit_binding()
    : _texture_image()
    , _format(scm::gl::FORMAT_NULL)
    , _access(scm::gl::ACCESS_READ_WRITE)
    , _level(0)
    , _layer(0)
{
}

bool
LamureContext::image_unit_binding::operator==(const image_unit_binding& rhs) const
{
    return    (_texture_image == rhs._texture_image)
        && (_format == rhs._format)
        && (_access == rhs._access)
        && (_level == rhs._level)
        && (_layer == rhs._layer);
}

bool
LamureContext::image_unit_binding::operator!=(const image_unit_binding& rhs) const
{
    return    (_texture_image != rhs._texture_image)
        || (_format != rhs._format)
        || (_access != rhs._access)
        || (_level != rhs._level)
        || (_layer != rhs._layer);
}

LamureContext::binding_state_type::binding_state_type()
    : _stencil_ref_value(0)
    , _line_width(1.0f)
    , _point_size(1.0f)
    , _blend_color(scm::math::vec4f(1.0f, 1.0f, 1.0f, 1.0f))
    , _default_framebuffer_target(scm::gl::FRAMEBUFFER_BACK)
    , _viewports(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec2ui(10, 10)))
{
}

void* LamureContext::map_buffer(const scm::gl::buffer_ptr& in_buffer, const scm::gl::access_mode in_access) const
{
    return nullptr;
}

void* LamureContext::map_buffer_range(const scm::gl::buffer_ptr& in_buffer, scm::size_t in_offset, scm::size_t in_size, const scm::gl::access_mode in_access) const
{
    return nullptr;
}

bool LamureContext::unmap_buffer(const scm::gl::buffer_ptr& in_buffer) const
{
    return false;
}

bool LamureContext::get_buffer_sub_data(const scm::gl::buffer_ptr& in_buffer, scm::size_t offset, scm::size_t size, void* const data) const
{
    return false;
}

bool LamureContext::copy_buffer_data(const scm::gl::buffer_ptr& in_dst_buffer, const scm::gl::buffer_ptr& in_src_buffer, scm::size_t in_dst_offset, scm::size_t in_src_offset, scm::size_t in_size) const
{
    return false;
}
