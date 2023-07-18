// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#ifndef CO_REN_MODEL_DATABASE_H_
#define CO_REN_MODEL_DATABASE_H_

#include <unordered_map>
#include <mutex>
#include <platform.h>
#include <utils.h>
#include <types.h>
#include <dataset.h>
#include <config.h>
#include <controller.h>
#include <scm/gl_core/query_objects.h>

class model_database_lmr
{
public:

                        model_database_lmr(const model_database_lmr&) = delete;
                        model_database_lmr& operator=(const model_database_lmr&) = delete;
    virtual             ~model_database_lmr();

    static model_database_lmr* get_instance();

    const model_t       add_model(const std::string& filepath, const std::string& model_key);
    dataset_lmr*            get_model(const model_t model_id);
    void                apply();

    const model_t       num_models() const { return num_datasets_; };

    const size_t        get_primitive_size(const bvh::primitive_type type) const;
    const size_t        get_node_size(const model_t model_id) const;

    const size_t        get_slot_size() const;
    const size_t        get_primitives_per_node() const;
    const size_t        get_primitives_per_node(const model_t model_id) const;

    static bool         contains_only_compressed_data_;
    static bool         contains_trimesh_;
protected:

                        model_database_lmr();
    static bool         is_instanced_;
    static model_database_lmr* single_;

private:
    static std::mutex   mutex_;

    std::unordered_map<model_t, dataset_lmr*> datasets_;

    model_t             num_datasets_;
    model_t             num_datasets_pending_;
    size_t              primitives_per_node_;
    size_t              primitives_per_node_pending_;


};


#endif // CO_REN_MODEL_DATABASE_H_
