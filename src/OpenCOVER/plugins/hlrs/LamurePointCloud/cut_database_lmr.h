// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#ifndef CO_REN_CUT_DATABASE_H_
#define CO_REN_CUT_DATABASE_H_

#include <map>
//#include <lamure/utils.h>
//#include <lamure/types.h>
#include <mutex>

#include <platform.h>
#include <utils.h>
#include <types.h>
#include <cut.h>
#include <cut_database_record.h>


class cut_update_pool_lmr;

class cut_database_lmr
{
public:

                        cut_database_lmr(const cut_database_lmr&) = delete;
                        cut_database_lmr& operator=(const cut_database_lmr&) = delete;
    virtual             ~cut_database_lmr();

    static cut_database_lmr* get_instance();

    void                reset();

    cut_lmr&                get_cut(const context_t context_id, const view_t view_id, const model_t model_id);
    std::vector<cut_database_record_lmr::slot_update_desc>& get_updated_set(const context_t context_id);

    const bool          is_front_modified(const context_t context_id);
    void                set_is_front_modified(const context_t context_id, const bool front_modified);
    const bool          is_swap_required(const context_t context_id);
    void                signal_upload_complete(const context_t context_id);

    const cut_database_record_lmr::temporary_buffer get_buffer(const context_t context_id);
    void                swap(const context_t context_id);
    void                send_camera(const context_t context_id, const view_t view_id, const camera_lmr& camera);
    void                send_height_divided_by_top_minus_bottom(context_t const context_id, view_t const view_id, const float& height_divided_by_top_minus_bottom);
    void                send_transform(const context_t context_id, const model_t model_id, const scm::math::mat4f& transform);
    void                send_rendered(const context_t context_id, const model_t model_id);
    void                send_threshold(const context_t context_id, const model_t model_id, const float threshold);

protected:
                        cut_database_lmr();
    static bool         is_instanced_;
    static cut_database_lmr* single_;

    friend class        cut_update_pool_lmr;

    void                expand(const context_t context_id);
    void                receive_cameras(const context_t context_id, std::map<view_t, camera_lmr>& cameras);
    void                receive_height_divided_by_top_minus_bottoms(const context_t context_id, std::map<view_t, float>& height_divided_by_top_minus_bottom);
    void                receive_transforms(const context_t context_id, std::map<model_t, scm::math::mat4f>& transforms);
    void                receive_rendered(const context_t context_id, std::set<model_t>& rendered);
    void                receive_importance(const context_t context_id, std::map<model_t, float>& importance);
    void                receive_thresholds(const context_t context_id, std::map<model_t, float>& thresholds);

    void                lock_record(const context_t context_id);
    void                unlock_record(const context_t context_id);

    void                set_buffer(const context_t context_id, const cut_database_record_lmr::temporary_buffer buffer);

    void                set_is_swap_required(const context_t context_id, const bool front_modified);

    void                set_updated_set(const context_t context_id, std::vector<cut_database_record_lmr::slot_update_desc>& updated_set);

    void                set_cut(const context_t context_id, const view_t view_id, const model_t model_id, cut_lmr& cut_lmr);

private:
    /* data */
    static std::mutex   mutex_;

    std::map<context_t, cut_database_record_lmr*> records_;

};


#endif // CO_REN_CUT_DATABASE_H_
