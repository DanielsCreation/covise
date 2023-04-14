// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#ifndef CO_COMMON_MEMORY_H_
#define CO_COMMON_MEMORY_H_

//#include <lamure/platform.h>
#include <platform.h>
#include <cstddef>

const size_t get_total_memory();
const size_t get_available_memory(const bool use_buffers_cache = true);
const size_t get_process_used_memory();


#endif // CO_COMMON_MEMORY_H_

