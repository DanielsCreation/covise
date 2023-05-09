
// Copyright (c) 2012 Christopher Lux <christopherlux@gmail.com>
// Distributed under the Modified BSD License, see license.txt.

#ifndef CO_SCM_GL_CORE_DEVICE_H_INCLUDED
#define CO_SCM_GL_CORE_DEVICE_H_INCLUDED

//#include <scm/gl_core/render_device/device.h>

//#include <iosfwd>
//#include <limits>
//#include <list>
//#include <set>
//#include <utility>
//#include <vector>
//
//#include <boost/noncopyable.hpp>
//#include <boost/unordered_set.hpp>
//#include <boost/unordered_map.hpp>
//
//#include <scm/config.h>
//#include <scm/core/math.h>
//#include <scm/core/memory.h>
//
//#include <scm/gl_core/gl_core_fwd.h>
//#include <scm/gl_core/data_formats.h>
//#include <scm/gl_core/buffer_objects/buffer.h>
//#include <scm/gl_core/shader_objects/shader_objects_fwd.h>
//#include <scm/gl_core/shader_objects/shader_macro.h>
//#include <scm/gl_core/state_objects/blend_state.h>
//#include <scm/gl_core/state_objects/depth_stencil_state.h>
//#include <scm/gl_core/state_objects/rasterizer_state.h>
//
//#include <scm/core/platform/platform.h>
//#include <scm/core/utilities/platform_warning_disable.h>

class LamureDevice {
public:
    LamureDevice();
    virtual ~LamureDevice();
}; // class LamureDevice


#endif // CO_SCM_GL_CORE_DEVICE_H_INCLUDED
