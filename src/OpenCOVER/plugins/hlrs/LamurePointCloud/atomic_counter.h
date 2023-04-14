// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#ifndef CO_COMMON_ATOMIC_COUNTER_H_
#define CO_COMMON_ATOMIC_COUNTER_H_

#include <atomic>


template <typename INTEGRAL_OR_POINTER_T>
struct atomic_counter {
	std::atomic<INTEGRAL_OR_POINTER_T> head_;

	void initialize(INTEGRAL_OR_POINTER_T const& init_value) {
		head_.store(init_value);
	}
	// returns the value contained before increment
	INTEGRAL_OR_POINTER_T increment_head() {
		return head_++;
	}
};



#endif // CO_COMMON_ATOMIC_COUNTER_H_