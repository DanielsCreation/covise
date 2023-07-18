// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

//#include <lamure/ren/policy.h>
//#include <lamure/ren/controller.h>

#include <policy.h>
#include <config.h>


std::mutex policy_lmr::mutex_;
bool policy_lmr::is_instanced_ = false;
policy_lmr* policy_lmr::single_ = nullptr;

policy_lmr::
policy_lmr()
: reset_system_(true),
  max_upload_budget_in_mb_(LAMURE_DEFAULT_UPLOAD_BUDGET),
  render_budget_in_mb_(LAMURE_DEFAULT_VIDEO_MEMORY_BUDGET),
  out_of_core_budget_in_mb_(LAMURE_DEFAULT_MAIN_MEMORY_BUDGET),
  size_of_provenance_(LAMURE_DEFAULT_SIZE_OF_PROVENANCE),
  window_width_(800),
  window_height_(600) {

}

policy_lmr::
~policy_lmr() {
    std::lock_guard<std::mutex> lock(mutex_);
    is_instanced_ = false;
}

policy_lmr* policy_lmr::
get_instance() {
    if (!is_instanced_) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!is_instanced_) {
            single_ = new policy_lmr();
            is_instanced_ = true;
        }

        return single_;
    }
    else {
        return single_;
    }
}


