// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

//#include <lamure/ren/cache.h>
#include <cache.h>


cache_lmr::
cache_lmr(const slot_t num_slots)
    : num_slots_(num_slots), slot_size_(0) {
    model_database_lmr* database = model_database_lmr::get_instance();

    slot_size_ = database->get_slot_size();
    index_ = new cache_index_lmr(database->num_models(), num_slots_);
}

cache_lmr::
~cache_lmr() {
    if (index_ != nullptr) {
        delete index_;
        index_ = nullptr;
    }
}

const bool cache_lmr::
is_node_resident(const model_t model_id, const node_t node_id) {
    return index_->is_node_indexed(model_id, node_id);
}

const slot_t cache_lmr::
num_free_slots() {
    return index_->num_free_slots();
}

const slot_t cache_lmr::
slot_id(const model_t model_id, const node_t node_id) {
    return index_->get_slot(model_id, node_id);
}

void cache_lmr::
aquire_node(const context_t context_id, const view_t view_id, const model_t model_id, const node_t node_id) {
    if (index_->is_node_indexed(model_id, node_id)) {
        uint32_t hash_id = ((((uint32_t)context_id) & 0xFFFF) << 16) | (((uint32_t)view_id) & 0xFFFF);
        index_->aquire_slot(hash_id, model_id, node_id);
    }
}

void cache_lmr::
release_node(const context_t context_id, const view_t view_id, const model_t model_id, const node_t node_id) {
    if (index_->is_node_indexed(model_id, node_id)) {
        uint32_t hash_id = ((((uint32_t)context_id) & 0xFFFF) << 16) | (((uint32_t)view_id) & 0xFFFF);
        index_->release_slot(hash_id, model_id, node_id);
    }

}

const bool cache_lmr::
release_node_invalidate(const context_t context_id, const view_t view_id, const model_t model_id, const node_t node_id) {
    if (index_->is_node_indexed(model_id, node_id)) {
        uint32_t hash_id = ((((uint32_t)context_id) & 0xFFFF) << 16) | (((uint32_t)view_id) & 0xFFFF);
        return index_->release_slot_invalidate(hash_id, model_id, node_id);
    }

    return false;
}

void cache_lmr::
lock() {
    mutex_.lock();
}

void cache_lmr::
unlock() {
    mutex_.unlock();
}

