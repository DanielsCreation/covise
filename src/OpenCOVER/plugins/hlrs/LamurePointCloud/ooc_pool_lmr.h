// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#ifndef CO_REN_OOC_POOL_H_
#define CO_REN_OOC_POOL_H_

//#include <lamure/ren/data_provenance.h>
//#include <lamure/ren/cache_index.h>
//#include <lamure/ren/cache_queue.h>
//#include <lamure/ren/config.h>
//#include <lamure/ren/lod_stream.h>
//#include <lamure/ren/model_database.h>
//#include <lamure/ren/provenance_stream.h>
//#include <lamure/types.h>
//#include <lamure/utils.h>
#include <map>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

//#include <lamure/semaphore.h>
//#include <lamure/utils.h>
//#include <lamure/types.h>
//#include <lamure/ren/config.h>
//#include <lamure/ren/model_database.h>
//#include <lamure/ren/lod_stream.h>
//#include <lamure/ren/cache_queue.h>
//#include <lamure/ren/cache_index.h>

#include <data_provenance.h>
#include <cache_index.h>
#include <cache_queue.h>
#include <lod_stream.h>
#include <semaphore.h>
#include <utils.h>
#include <types.h>
#include <config.h>
#include <model_database.h>
#include <lod_stream.h>

class ooc_pool_lmr
{
  public:
    ooc_pool_lmr(const uint32_t num_loader_threads, const size_t size_of_slot_in_bytes);
    ooc_pool_lmr(const uint32_t num_loader_threads, const size_t size_of_slot_in_bytes, const size_t size_of_slot_provenance_, Data_Provenance_lmr const &data_provenance);
    /*virtual*/ ~ooc_pool_lmr();

    const uint32_t num_threads() const { return num_threads_; };

    bool acknowledge_request(cache_queue_lmr::job job);
    void acknowledge_update(const model_t model_id, const node_t node_id, int32_t priority);

    cache_queue_lmr::query_result acknowledge_query(const model_t model_id, const node_t node_id);

    void resolve_cache_history(cache_index_lmr *index);
    void perform_queue_maintenance(cache_index_lmr *index);

    void lock();
    void unlock();

    void begin_measure();
    void end_measure();

  protected:
    void run();
    bool is_shutdown();

  private:
    bool locked_;
    semaphore_lmr semaphore_;
    size_t size_of_slot_;
    size_t size_of_slot_provenance_;
    std::mutex mutex_;

    uint32_t num_threads_;
    std::vector<std::thread> threads_;

    bool shutdown_;

    size_t bytes_loaded_;

    std::vector<cache_queue_lmr::job> history_;

    cache_queue_lmr priority_queue_;

    Data_Provenance_lmr _data_provenance;
};
#endif // CO_REN_OOC_POOL_H_
