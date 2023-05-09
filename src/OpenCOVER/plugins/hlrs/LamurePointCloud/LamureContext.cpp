
// Copyright (c) 2012 Christopher Lux <christopherlux@gmail.com>
// Distributed under the Modified BSD License, see license.txt.

#include "LamureContext.h"
#include "LamureDevice.h"

#include <sstream>

#include <scm/gl_core/config.h>
#include <scm/gl_core/object_state.h>
#include <scm/gl_core/buffer_objects.h>
#include <scm/gl_core/frame_buffer_objects.h>
#include <scm/gl_core/query_objects.h>
#include <scm/gl_core/shader_objects.h>
#include <scm/gl_core/state_objects.h>
#include <scm/gl_core/sync_objects.h>
#include <scm/gl_core/texture_objects.h>
#include <scm/gl_core/render_device/device.h>
#include <scm/gl_core/render_device/opengl/gl_core.h>
#include <scm/gl_core/render_device/opengl/util/assert.h>
#include <scm/gl_core/render_device/opengl/util/binding_guards.h>
#include <scm/gl_core/render_device/opengl/util/constants_helper.h>
#include <scm/gl_core/render_device/opengl/util/data_type_helper.h>

#include <scm/gl_core/log.h>
#include <scm/config.h>

#if SCM_ENABLE_CUDA_CL_SUPPORT
  #include <scm/cl_core/cuda/device.h>
  #include <scm/cl_core/opencl/device.h>
#endif




LamureContext::~LamureContext()
{
}
