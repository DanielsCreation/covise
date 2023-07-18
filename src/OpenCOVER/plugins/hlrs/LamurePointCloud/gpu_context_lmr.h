// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#ifndef CO_LAMURE_REN_GPU_CONTEXT_H
#define CO_LAMURE_REN_GPU_CONTEXT_H

//#include <lamure/ren/data_provenance.h>
//#include <lamure/ren/bvh.h>
//#include <lamure/ren/cut_database_record.h>
//#include <lamure/ren/gpu_access.h>
//#include <lamure/types.h>

#include <data_provenance.h>
#include <bvh.h>
#include <types.h>
#include <gpu_access.h>
#include <cut_database_record.h>


class gpu_context_lmr
{
  public:
    gpu_context_lmr(const context_t context_id);
    ~gpu_context_lmr();

    struct temporary_storages
    {
        temporary_storages(char *storage_a, char *storage_b) : storage_a_(storage_a), storage_b_(storage_b){};

        char *storage_a_;
        char *storage_b_;
    };
    struct fix_struct
    {
        char *fix_buffer_;
        char *fix_buffer_provenance_;
    };
    const context_t context_id() const { return context_id_; };
    const bool is_created() const { return is_created_; };

    temporary_storages get_temporary_storages() { return temporary_storages_; };
    temporary_storages get_temporary_storages_provenance() { return temporary_storages_provenance_; };

    scm::gl::buffer_ptr get_context_buffer(lmr_device device);
    scm::gl::buffer_ptr get_context_buffer(lmr_device device, Data_Provenance_lmr const &data_provenance);
    scm::gl::vertex_array_ptr get_context_memory(bvh::primitive_type type, lmr_device device);
    scm::gl::vertex_array_ptr get_context_memory(bvh::primitive_type type, lmr_device device, Data_Provenance_lmr const &data_provenance);

    const node_t upload_budget_in_nodes() const { return upload_budget_in_nodes_; };
    const node_t render_budget_in_nodes() const { return render_budget_in_nodes_; };

    void map_temporary_storage(const cut_database_record_lmr::temporary_buffer &buffer, lmr_device device);
    void map_temporary_storage(const cut_database_record_lmr::temporary_buffer &buffer, lmr_device device, Data_Provenance_lmr const &data_provenance);
    void unmap_temporary_storage(const cut_database_record_lmr::temporary_buffer &buffer, lmr_device device);
    void unmap_temporary_storage(const cut_database_record_lmr::temporary_buffer &buffer, lmr_device device, Data_Provenance_lmr const &data_provenance);
    bool update_primary_buffer(const cut_database_record_lmr::temporary_buffer &from_buffer, lmr_device device);
    bool update_primary_buffer_fix(const cut_database_record_lmr::temporary_buffer &from_buffer, lmr_device device, Data_Provenance_lmr const &data_provenance);

    fix_struct get_fix_a() { return fix_a_; };
    fix_struct get_fix_b() { return fix_b_; };

    void create(lmr_device device);
    void create(lmr_device device, Data_Provenance_lmr const &data_provenance);

private:

    void test_video_memory(lmr_device device);
    void test_video_memory(lmr_device device, Data_Provenance_lmr const &data_provenance);

    context_t context_id_;

    bool is_created_;

    gpu_access_lmr *temp_buffer_a_;
    gpu_access_lmr *temp_buffer_b_;

    fix_struct fix_a_;
    fix_struct fix_b_;

    gpu_access_lmr *primary_buffer_;

    temporary_storages temporary_storages_;
    temporary_storages temporary_storages_provenance_;
    node_t upload_budget_in_nodes_;
    node_t render_budget_in_nodes_;
};
#endif
