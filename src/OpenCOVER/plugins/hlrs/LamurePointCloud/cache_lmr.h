// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#ifndef CO_REN_CACHE_H_
#define CO_REN_CACHE_H_

#include <map>
#include <queue>
//#include <lamure/utils.h>

#include <memory>
#include <mutex>
#include <scm/core.h>
#include <scm/gl_core.h>

#include <policy.h>
#include <platform.h>
#include <utils.h>
#include <model_database.h>
#include <types.h>
#include <cache_index.h>


class cache_lmr
{
public:
                        cache_lmr(const cache_lmr&) = delete;
                        cache_lmr& operator=(const cache_lmr&) = delete;
    virtual             ~cache_lmr();

    const bool          is_node_resident(const model_t model_id, const node_t node_id);

    const slot_t        num_free_slots();
    const slot_t        slot_id(const model_t model_id, const node_t node_id);

    const slot_t        num_slots() const { return num_slots_; };
    const slot_t        slot_size() const { return slot_size_; };

    void                lock();
    void                unlock();

    void                aquire_node(const context_t context_id, const view_t view_id, const model_t model_id, const node_t node_id);
    void                release_node(const context_t context_id, const view_t view_id, const model_t model_id, const node_t node_id);
    const bool          release_node_invalidate(const context_t context_id, const view_t view_id, const model_t model_id, const node_t node_id);

protected:
                        cache_lmr(const slot_t num_slots);

    cache_index_lmr*        index_;
    std::mutex          mutex_;

private:
    /* data */

    slot_t              num_slots_;
    node_t              num_nodes_;
    size_t              slot_size_;


};


#endif // CO_REN_CACHE_H_
