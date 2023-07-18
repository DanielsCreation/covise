USING(LamurePointCloud)


set(SRC_DIR                         ${CMAKE_CURRENT_LIST_DIR})

macro(pcl_project_files)
    set(out_file_list       ${ARGV0})
    set(search_dir          ${ARGV1})
    set(in_glob_expr        ${ARGV})

    get_filename_component(src_path     ${SRC_DIR} ABSOLUTE)
    get_filename_component(search_path  ${search_dir} ABSOLUTE)

    message(${src_path})
    message(${search_path})

    # remove the leading output and input variable
    list(REMOVE_AT in_glob_expr 0 1)
    set(out_proj_files    "")

    set(SOURCE_GROUP_DELIMITER "/")
    

    foreach(glob_expr ${in_glob_expr})
        file(GLOB proj_files ${search_path}/${glob_expr})
        foreach(proj_file ${proj_files})
            file(RELATIVE_PATH rel_path ${src_path} ${proj_file})
            if (rel_path)
                file(TO_CMAKE_PATH ${rel_path} rel_path)
            endif (rel_path)

            message(${rel_path} " " ${proj_file})

            list(APPEND out_proj_files ${proj_file})

        endforeach(proj_file)
    endforeach(glob_expr)

    list(APPEND ${out_file_list} ${out_proj_files})

endmacro(scm_project_files)

set(LAMURE_INCLUDE_DIRS
    ${SRC_DIR}/virtual_texturing/include
    ${SRC_DIR}/rendering/include
    ${SRC_DIR}/pvs_common/include
    ${SRC_DIR}/common/include
    ${SRC_DIR}/pvs_preprocessing/include
    ${SRC_DIR}/provenance/include
    ${SRC_DIR}/quality_measurement/include
)

set(FREEGLUT_INCLUDE_DIR
    $ENV{EXTERNLIBS}/freeglut/include
)
set(FREEGLUT_LIBRARY
    $ENV{EXTERNLIBS}/freeglut/lib/x64/freeglut.lib
)

set(FREEIMAGE_INCLUDE_DIR
    $ENV{EXTERNLIBS}/FreeImage/include
)
set(FREEIMAGE_LIBRARY
    $ENV{EXTERNLIBS}/FreeImage/lib/FreeImage.lib
)
set(FREEIMAGE_LIBRARY_DEBUG
    $ENV{EXTERNLIBS}/FreeImage/lib/FreeImaged.lib
)
set(FREEIMAGEPLUS_LIBRARY
    $ENV{EXTERNLIBS}/FreeImage/lib/FreeImagePlus.lib
)
set(FREEIMAGEPLUS_LIBRARY_DEBUG
    $ENV{EXTERNLIBS}/FreeImage/lib/FreeImagePlusd.lib
)


pcl_project_files(SOURCE_FILES      ${SRC_DIR}/virtual_texturing/src                        *.cpp)
pcl_project_files(SOURCE_FILES      ${SRC_DIR}/virtual_texturing/src/..                     *.cpp)
pcl_project_files(HEADER_FILES      ${SRC_DIR}/virtual_texturing/include/lamure/vt          *.h)
pcl_project_files(HEADER_FILES      ${SRC_DIR}/virtual_texturing/include/lamure/vt/..       *.h)

pcl_project_files(SOURCE_FILES      ${SRC_DIR}/rendering/src                                *.cpp)
pcl_project_files(HEADER_FILES      ${SRC_DIR}/rendering/include/lamure/ren                 *.h)

pcl_project_files(SOURCE_FILES      ${SRC_DIR}/pvs_common/src                               *.cpp)
pcl_project_files(HEADER_FILES      ${SRC_DIR}/pvs_common/include/lamure/pvs                *.h)

pcl_project_files(SOURCE_FILES      ${SRC_DIR}/common/src                                   *.cpp)
pcl_project_files(HEADER_FILES      ${SRC_DIR}/common/include/lamure                        *.h)

pcl_project_files(SOURCE_FILES      ${SRC_DIR}/pvs_preprocessing/src                        *.cpp)
pcl_project_files(HEADER_FILES      ${SRC_DIR}/pvs_preprocessing/include/lamure/pvs         *.h)

pcl_project_files(SOURCE_FILES      ${SRC_DIR}/provenance/src                               *.cpp)
pcl_project_files(HEADER_FILES      ${SRC_DIR}/provenance/include/lamure/prov               *.h)

pcl_project_files(SOURCE_FILES      ${SRC_DIR}/quality_measurement/src                      *.cpp)
pcl_project_files(HEADER_FILES      ${SRC_DIR}/quality_measurement/include/lamure/qm        *.h)

SET(HEADERS ${HEADER_FILES}
  LamurePointCloud.h
  LamureGeometry.h
  LamureDrawable.h
  LamureContext.h
  LamureDevice.h
)
  #bvh_lmr.h
  #bvh_stream_lmr.h
  #cache_lmr.h
  #cache_index_lmr.h
  #cache_queue_lmr.h
  #camera_lmr.h
  #controller_lmr.h
  #cut_lmr.h
  #cut_database_lmr.h
  #cut_database_record_lmr.h
  #cut_update_index_lmr.h
  #cut_update_pool_lmr.h
  #cut_update_queue_lmr.h
  #dataset_lmr.h
  #gpu_access_lmr.h
  #gpu_cache_lmr.h
  #gpu_context_lmr.h
  #lod_stream_lmr.h
  #model_database_lmr.h
  #ooc_cache_lmr.h
  #ooc_pool_lmr.h
  #policy_lmr.h
  #provenance_stream_lmr.h
  #ray_lmr.h
  #trackball_lmr.h
  #bounding_box_lmr.h
  #memory_lmr.h
  #memory_status_lmr.h
  #semaphore_lmr.h
  #sphere_lmr.h
  #types_lmr.h
  #utils_lmr.h
  #version_lmr.h
  #config_lmr.h
  #item_provenance_lmr.h
  #platform_lmr.h
  #atomic_counter_lmr.h
  #data_provenance_lmr.h


SET(SOURCES ${SOURCE_FILES}
  LamurePointCloud.cpp
  LamureGeometry.cpp
  LamureDrawable.cpp
  #LamureContext.cpp
  #LamureDevice.cpp
)
  #bvh_lmr.cpp
  #bvh_stream_lmr.cpp
  #cache_lmr.cpp
  #cache_index_lmr.cpp
  #cache_queue_lmr.cpp
  #camera_lmr.cpp
  #controller_lmr.cpp
  #cut_lmr.cpp
  #cut_database_lmr.cpp
  #cut_database_record_lmr.cpp
  #cut_update_index_lmr.cpp
  #cut_update_pool_lmr.cpp
  #cut_update_queue_lmr.cpp
  #dataset_lmr.cpp
  #gpu_access_lmr.cpp
  #gpu_cache_lmr.cpp
  #gpu_context_lmr.cpp
  #lod_stream_lmr.cpp
  #model_database_lmr.cpp
  #ooc_cache_lmr.cpp
  #ooc_pool_lmr.cpp
  #policy_lmr.cpp
  #provenance_stream_lmr.cpp
  #ray_lmr.cpp
  #trackball_lmr.cpp
  #bounding_box_lmr.cpp
  #memory_lmr.cpp
  #memory_status_lmr.cpp
  #semaphore_lmr.cpp
  #sphere_lmr.cpp
  #utils_lmr.cpp


set(SCHISM_INCLUDE_DIRS
    $ENV{EXTERNLIBS}/../../schism/scm_core/src
    $ENV{EXTERNLIBS}/../../schism/scm_gl_core/src
    $ENV{EXTERNLIBS}/../../schism/scm_gl_util/src
    $ENV{EXTERNLIBS}/../../schism/scm_input/src
)

set(SCHISM_CORE_LIBRARY $ENV{EXTERNLIBS}/../../schism/lib/win_x64/Release/scm_core.lib)
set(SCHISM_CORE_LIBRARY_DEBUG $ENV{EXTERNLIBS}/../../schism/lib/win_x64/Debug/scm_core-gd.lib)

set(SCHISM_GL_CORE_LIBRARY $ENV{EXTERNLIBS}/../../schism/lib/win_x64/Release/scm_gl_core.lib)
set(SCHISM_GL_CORE_LIBRARY_DEBUG $ENV{EXTERNLIBS}/../../schism/lib/win_x64/Debug/scm_gl_core-gd.lib)

set(SCHISM_GL_UTIL_LIBRARY $ENV{EXTERNLIBS}/../../schism/lib/win_x64/Release/scm_gl_util.lib)
set(SCHISM_GL_UTIL_LIBRARY_DEBUG $ENV{EXTERNLIBS}/../../schism/lib/win_x64/Debug/scm_gl_util-gd.lib)

set(FREETYPE_LIBRARY_RELEASE C:/src/externlibs/zebu/freetype/lib/freetype.lib)
set(FREETYPE_LIBRARY_DEBUG C:/src/externlibs/zebu/freetype/lib/freetyped.lib)

set(FREETYPE_INCLUDE_DIR_freetype2 C:/src/externlibs/freetype/include)

link_directories(
    optimized $ENV{EXTERNLIBS}/schism/lib/win_x64/Release debug $ENV{EXTERNLIBS}/../../schism/lib/win_x64/Debug
)

include_directories(
#${REND_INCLUDE_DIR}
#${COMMON_INCLUDE_DIR}
#${PVS_COMMON_INCLUDE_DIR}
${FREEGLUT_INCLUDE_DIR}
${GLUT_INCLUDE_DIR}
${GLM_INCLUDE_DIR}
${FREEIMAGE_INCLUDE_DIR}
${LAMURE_INCLUDE_DIRS}
${SCHISM_INCLUDE_DIRS}
${GLEW_INCLUDE_DIR}
${Boost_INCLUDE_DIR}
)

include_directories(SYSTEM 
  ${SCHISM_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

cover_add_plugin(LamurePointCloud)
covise_wnoerror(LamurePointCloud)

TARGET_LINK_LIBRARIES(LamurePointCloud
    #optimized ${LAMURE_RENDERING_LIBRARY} debug ${LAMURE_RENDERING_LIBRARY_DEBUG}
    #optimized ${LAMURE_COMMON_LIBRARY} debug ${LAMURE_COMMON_LIBRARY_DEBUG}
    #optimized ${LAMURE_PVS_COMMON_LIBRARY} debug ${LAMURE_PVS_COMMON_LIBRARY_DEBUG}
    #optimized ${LAMUREVT_LIBRARY} debug ${LAMUREVT_LIBRARY_DEBUG}
    #optimized ${LAMURE_QUALITY_MEASUREMENT_LIBRARY} debug ${LAMURE_QUALITY_MEASUREMENT_LIBRARY_DEBUG}
    #optimized ${LAMURE_PVS_PREPROCESSING_LIBRARY} debug ${LAMURE_PVS_PREPROCESSING_LIBRARY_DEBUG}
    #optimized ${LAMURE_PREPROCESSING_LIBRARY} debug ${LAMURE_PREPROCESSING_LIBRARY_DEBUG}
    optimized ${SCHISM_CORE_LIBRARY} debug ${SCHISM_CORE_LIBRARY_DEBUG}
    optimized ${SCHISM_GL_CORE_LIBRARY} debug ${SCHISM_GL_CORE_LIBRARY_DEBUG}
    optimized ${SCHISM_GL_UTIL_LIBRARY} debug ${SCHISM_GL_UTIL_LIBRARY_DEBUG}
    optimized ${Boost_TIMER_LIBRARY_RELEASE} debug ${Boost_TIMER_LIBRARY_DEBUG}
    optimized ${FREETYPE_LIBRARY_RELEASE} debug ${FREETYPE_LIBRARY_DEBUG}
    ${FREEGLUT_LIBRARY}
    optimized ${FREEIMAGE_LIBRARY} debug ${FREEIMAGE_LIBRARY_DEBUG}
    optimized ${FREEIMAGEPLUS_LIBRARY} debug ${FREEIMAGEPLUS_LIBRARY_DEBUG}
)