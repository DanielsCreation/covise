// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#ifndef CO_REN_ITEM_PROVENANCE_H_
#define CO_REN_ITEM_PROVENANCE_H_

//#include <lamure/ren/cache_index.h>
//#include <lamure/ren/cache_queue.h>
//#include <lamure/ren/config.h>
//#include <lamure/ren/lod_stream.h>
//#include <lamure/ren/model_database.h>
//#include <lamure/ren/provenance_stream.h>
//#include <lamure/semaphore.h>
//#include <lamure/types.h>
//#include <lamure/utils.h>
#include <map>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include <cache_index.h>
#include <cache_queue.h>
#include <config.h>
#include <lod_stream.h>
#include <model_database.h>
#include <provenance_stream.h>
#include <semaphore.h>
#include <types.h>
#include <utils.h>

class Item_Provenance
{
public:
    enum type_item
    {
        TYPE_INT,
        TYPE_FLOAT,
        TYPE_VEC3I,
        TYPE_VEC3F
    };

    enum visualization_item
    {
        VISUALIZATION_COLOR,
        VISUALIZATION_ARROW,
    };

    Item_Provenance(type_item type, visualization_item visualization) : _type(type), _visualization(visualization) {};

    int get_size_in_bytes() const
    {
        switch (_type)
        {
        case TYPE_INT:
            return sizeof(int);
            break;
        case TYPE_FLOAT:
            return sizeof(float);
            break;
        case TYPE_VEC3I:
            return sizeof(scm::math::vec3i);
            break;
        case TYPE_VEC3F:
            return sizeof(scm::math::vec3f);
            break;
        }
        return 0;
    };

    type_item get_type() const { return _type; };
    visualization_item get_visualization() const { return _visualization; };

private:
    visualization_item _visualization;
    type_item _type;
};
#endif // CO_REN_ITEM_PROVENANCE_H_
