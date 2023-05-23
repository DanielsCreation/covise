
// Copyright (c) 2012 Christopher Lux <christopherlux@gmail.com>
// Distributed under the Modified BSD License, see license.txt.

#include "LamureContext.h"

#include <sstream>

#include <scm/gl_core/config.h>
#include <scm/gl_core/object_state.h>
#include <scm/gl_core/buffer_objects.h>
#include <scm/gl_core/frame_buffer_objects.h>
#include <scm/gl_core/query_objects.h>
#include <scm/gl_core/shader_objects.h>
#include <scm/gl_core/state_objects.h>
//#include <scm/gl_core/sync_objects.h>
//#include <scm/gl_core/texture_objects.h>
//#include <scm/gl_core/render_device/device.h>
//#include <scm/gl_core/render_device/opengl/gl_core.h>
//#include <scm/gl_core/render_device/opengl/util/assert.h>
//#include <scm/gl_core/render_device/opengl/util/binding_guards.h>
//#include <scm/gl_core/render_device/opengl/util/constants_helper.h>
//#include <scm/gl_core/render_device/opengl/util/data_type_helper.h>

//#include <scm/gl_core/log.h>
//#include <scm/config.h>


LamureContext::LamureContext(LamureDevice& in_device)
{
    _default_depth_stencil_state = in_device.create_depth_stencil_state(scm::gl::depth_stencil_state_desc());
    //_default_depth_stencil_state->force_apply(*this, _current_state._stencil_ref_value);
    _current_state._depth_stencil_state = _default_depth_stencil_state;
    _applied_state._depth_stencil_state = _default_depth_stencil_state;

    _default_rasterizer_state = in_device.create_rasterizer_state(scm::gl::rasterizer_state_desc());
    //_default_rasterizer_state->force_apply(*this, _current_state._line_width, _current_state._point_size);
    _current_state._rasterizer_state = _default_rasterizer_state;
    _applied_state._rasterizer_state = _default_rasterizer_state;

    _default_blend_state = in_device.create_blend_state(scm::gl::blend_state_desc());
    //_default_blend_state->force_apply(*this, _current_state._blend_color);
    _current_state._blend_state = _default_blend_state;
    _applied_state._blend_state = _default_blend_state;

    _current_state._texture_units.resize(in_device.capabilities()._max_texture_image_units);
    _applied_state._texture_units.resize(in_device.capabilities()._max_texture_image_units);
    //if (glapi.extension_EXT_shader_image_load_store
    //    && in_device.capabilities()._max_image_units > 0) {
    //    _current_state._image_units.resize(in_device.capabilities()._max_image_units);
    //    _applied_state._image_units.resize(in_device.capabilities()._max_image_units);
    //}

    _current_state._active_uniform_buffers.resize(in_device.capabilities()._max_uniform_buffer_bindings);
    _applied_state._active_uniform_buffers.resize(in_device.capabilities()._max_uniform_buffer_bindings);

    _current_state._active_atomic_counter_buffers.resize(in_device.capabilities()._max_atomic_counter_buffer_bindings);
    _applied_state._active_atomic_counter_buffers.resize(in_device.capabilities()._max_atomic_counter_buffer_bindings);

    _current_state._active_storage_buffers.resize(in_device.capabilities()._max_shader_storage_block_bindings);
    _applied_state._active_storage_buffers.resize(in_device.capabilities()._max_shader_storage_block_bindings);

    //glapi.glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    //glapi.glPixelStorei(GL_PACK_ALIGNMENT, 1);

    _debug_synchronous_reporting = true;

    _active_transform_feedback_topology_mode = scm::gl::PRIMITIVE_POINTS;
}




LamureContext::~LamureContext()
{
}

void LamureContext::debug_output::operator()(scm::gl::debug_source, scm::gl::debug_type, scm::gl::debug_severity, const std::string&) const
{
}
