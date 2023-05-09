
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

#if SCM_ENABLE_CUDA_CL_SUPPORT
#include <scm/cl_core/cuda/device.h>
#include <scm/cl_core/opencl/device.h>
#endif

LamureDevice::LamureDevice()
{
}

LamureDevice::~LamureDevice()
{
}
